#include <Arduino.h>
#include <math.h>
#include <esp_system.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "ble_ctrl.h"          // UUIDs + app hooks

#include "../drivers/dfplayer.h"
#include "../app/telemetry.h"
#include "../app/ui_iface.h"
#include "../app/app_fsm.h"
#include "../app/constants.h"

// ---- HALO globals owned by main/app (runtime mirrors) ----
extern float   qnh_hPa;
extern float   airfieldElev_ft;
extern uint8_t df_volume;
extern void    strobeEnable(bool);
extern void    strobeSet(uint16_t on_ms, uint16_t period_ms);

// ===== BLE objects =====
static BLEServer*        pServer = nullptr;
static BLEService*       pService = nullptr;
static BLECharacteristic
  *pFlashCharacteristic      = nullptr,
  *pTestCharacteristic       = nullptr,
  *pVolumeCharacteristic     = nullptr,
  *pElevationCharacteristic  = nullptr,
  *pQnhCharacteristic        = nullptr,
  *pResetCharacteristic      = nullptr,
  *pDataSourceCharacteristic = nullptr;

// ===== Runtime mirrors for BLE reads =====
static uint8_t  curVolume = 24;         // 0..30
static uint16_t curElevationFeet = 0;   // feet (uint16)
static uint16_t curQnhHpa = 1013;       // hPa (uint16)
static bool     curIsSoftRF = false;    // false=FLARM, true=SoftRF
static uint8_t  curBaudIdxSoft  = 1;    // 0=19200, 1=38400
static uint8_t  curBaudIdxFlarm = 0;    // 0=19200, 1=38400

// ===== Test sequence state =====
static bool     testActive = false;
static uint8_t  testSequenceStep = 0;
static uint32_t lastTestStepTime = 0;
static const uint32_t TEST_STEP_MS = 6000; // more headroom for audio

// ---------- small utils ----------
static inline void dbg(const char* s){ Serial.println(s); }
static void log_payload(const char* tag, const std::string& v){
  Serial.printf("[BLE] %s len=%u ascii='", tag, (unsigned)v.size());
  for (size_t i=0;i<v.size();++i) {
    char c = (char)v[i];
    Serial.printf("%c", (c>=32 && c<=126)?c:'.');
  }
  Serial.printf("' hex=");
  for (size_t i=0;i<v.size();++i) Serial.printf("%02X ", (uint8_t)v[i]);
  Serial.println();
}
static bool is_ascii_digits(const std::string& s){
  if (s.empty()) return false;
  for (char c : s){ if (c<'0'||c>'9') return false; }
  return true;
}

// --- Helpers: alert injection + speech ---
static void injectAlert(int level, float bearing_deg, float relV_m) {
  extern TrafficAlert alert; // from telemetry.h
  alert.active = true;
  alert.since  = millis();
  alert.alarm  = level;

  float rad = bearing_deg * 3.1415926f / 180.0f;
  float dist = 1000.0f;
  alert.relN_m = dist * cosf(rad);
  alert.relE_m = dist * sinf(rad);
  alert.relV_m = relV_m;
  alert.dist_m = hypotf(alert.relN_m, alert.relE_m);
  alert.bearing_deg = bearing_deg;

  ui_set_page(PAGE_TRAFFIC);
}

static void speakVerticalAndClock(int oclock, const char* vert) {
  uint16_t vtrk = 10; // LEVEL
  if (!strcmp(vert,"HIGH")) vtrk = 11;
  else if (!strcmp(vert,"LOW")) vtrk = 12;

  int oc = oclock;
  if (oc < 1 || oc > 12) oc = 12;
  uint16_t clockTrack = 20 + oc; // 1..12 -> 21..32

  dfp_stop_and_flush();
  delay(60);
  dfp_play_filename(vtrk);
  delay(140);
  dfp_play_filename(clockTrack);
}

// --- TEST sequence driver (looping) ---
static void runTestSequence(uint32_t now){
  if (!testActive) return;
  if (lastTestStepTime != 0 && (now - lastTestStepTime < TEST_STEP_MS)) {
    extern void app_demo_extend_land_inhibit(uint32_t ms);
    app_demo_extend_land_inhibit(8000);
    return;
  }
  lastTestStepTime = now;
  testSequenceStep++;

  switch (testSequenceStep) {
    case 1:
      dbg("[TEST] Step 1: Takeoff");
      app_demo_force_flying();
      break;
    case 2:
      dbg("[TEST] Step 2: Alert HIGH @ 2 o'clock");
      injectAlert(2, 60.0f, +70.0f);
      speakVerticalAndClock(2, "HIGH");
      break;
    case 3:
      dbg("[TEST] Step 3: Alert LOW @ 10 o'clock");
      injectAlert(3, 300.0f, -70.0f);
      speakVerticalAndClock(10, "LOW");
      break;
    case 4:
      dbg("[TEST] Step 4: Alert LEVEL @ 12 o'clock");
      injectAlert(1, 0.0f, 0.0f);
      speakVerticalAndClock(12, "LEVEL");
      break;
    case 5:
      dbg("[TEST] Step 5: Landing");
      { extern TrafficAlert alert; alert = {}; }
      tele.sog_kts = 0.0f;
      app_demo_force_landing();
      testActive = false;
      break;
    default:
      dbg("[TEST] Looping");
      testSequenceStep = 0;
      break;
  }
}

// --- Data-source hot switch helper ---
static void applyBaudFromIndices() {
  const uint8_t idx = curIsSoftRF ? curBaudIdxSoft : curBaudIdxFlarm;
  const uint32_t baud = (idx == 0) ? 19200UL : 38400UL;
  halo_apply_nav_baud(baud);
  Serial.printf("[BLE] UART set: %s @ %lu\n",
    curIsSoftRF ? "SoftRF" : "FLARM", (unsigned long)baud);
}

// --- BLE callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { Serial.println("[BLE] client connected"); }
  void onDisconnect(BLEServer*) override {
    Serial.println("[BLE] client disconnected");
    pServer->getAdvertising()->start();
  }
};

class MyCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();

    if (c == pFlashCharacteristic) {
      Serial.println("[BLE] FLASH");
      strobeEnable(true);
      strobeSet(STROBE_ON_MS, 250);
      delay(150);
      strobeEnable(false);
      return;
    }

    if (c == pTestCharacteristic) {
      log_payload("TEST write", v);
      bool start = true;
      if (v.size() >= 1 && (uint8_t)v[0] == 0x00) start = false;

      if (start) {
        testActive = true;
        testSequenceStep = 0;
        lastTestStepTime = 0;
        Serial.println("[BLE] TEST sequence START");
      } else {
        testActive = false;
        Serial.println("[BLE] TEST sequence STOP -> return to BOOT");
        extern TrafficAlert alert; alert = {};
        dfp_stop_and_flush();
        strobeEnable(false);
        app_fsm_init();
        ui_markAllUndrawn();
        ui_set_page(PAGE_BOOT);
      }
      uint8_t state = testActive ? 1 : 0;
      pTestCharacteristic->setValue(&state, 1);
      return;
    }

    if (c == pVolumeCharacteristic) {
      log_payload("VOLUME write", v);
      uint16_t u16 = 0;
      // Accept ASCII "0..30", single byte, or LE u16
      if (v.size()==1) u16 = (uint8_t)v[0];
      else if (is_ascii_digits(v)) {
        char buf[8]={0}; size_t n=min(v.size(), sizeof(buf)-1); memcpy(buf,v.data(),n);
        u16 = (uint16_t)strtoul(buf,nullptr,10);
      } else if (v.size()>=2) {
        u16 = (uint16_t)((uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8));
      }
      uint8_t vol = (uint8_t)constrain((int)u16, 0, 30);
      curVolume = vol;
      halo_set_volume_runtime_and_persist(vol);
      pVolumeCharacteristic->setValue(&curVolume, 1);
      Serial.printf("[BLE] VOL=%u (saved)\n", curVolume);
      return;
    }

    if (c == pElevationCharacteristic) {
      log_payload("ELEV write", v);
      uint16_t feet = 0;

      if (v.size()==1) {
        // Single raw byte = tens-of-feet (0x39=57 -> 570 ft)
        feet = (uint16_t)((uint8_t)v[0]) * 10u;
      } else if (is_ascii_digits(v)) {
        // ASCII string: "570" => 570 ft; "57" => 570 ft (treat <400 as tens)
        char buf[12]={0}; size_t n=min(v.size(), sizeof(buf)-1); memcpy(buf,v.data(),n);
        unsigned long val = strtoul(buf,nullptr,10);
        feet = (val < 400) ? (uint16_t)(val * 10u) : (uint16_t)val;
      } else if (v.size()>=2) {
        // LE u16: if small, assume tens
        uint16_t u = (uint16_t)((uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8));
        feet = (u < 400) ? (uint16_t)(u * 10u) : u;
      }

      feet = (uint16_t)constrain((int)feet, 0, 30000);
      curElevationFeet = feet;
      halo_set_elev_runtime_and_persist(curElevationFeet);
      pElevationCharacteristic->setValue((uint8_t*)&feet, 2);
      Serial.printf("[BLE] ELEV=%u ft (saved)\n", feet);
      return;
    }

    if (c == pQnhCharacteristic) {
  log_payload("QNH write", v);

  auto has_dot = [](const std::string& s){ return s.find('.') != std::string::npos; };
  uint16_t hpa = 1013;

  if (v.size() == 1) {
    // Slider index: 0..200 => 800..1200 hPa in 2 hPa steps
    uint8_t idx = (uint8_t)v[0];
    hpa = (uint16_t)(800 + (uint16_t)idx * 2u);
    Serial.printf("[BLE] QNH from slider idx=%u -> %u hPa\n", idx, hpa);
  } else if (is_ascii_digits(v) || has_dot(v)) {
    // Accept ASCII "1016" (hPa) or "101.6" (×10)
    char buf[16] = {0};
    size_t n = min(v.size(), sizeof(buf)-1);
    memcpy(buf, v.data(), n);
    if (has_dot(v)) {
      float f = strtof(buf, nullptr);
      hpa = (uint16_t)lroundf(f * 10.0f);      // 101.6 -> 1016
    } else {
      unsigned long val = strtoul(buf, nullptr, 10);
      hpa = (uint16_t)val;                      // 1016 -> 1016
    }
  } else if (v.size() >= 2) {
    // LE u16 fallback: if it's clearly a slider index (< 400), map it; else assume hPa
    uint16_t u = (uint16_t)((uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8));
    hpa = (u < 400) ? (uint16_t)(800 + u * 2u) : u;
  }

  // Keep within sane range
  if (hpa < 800) hpa = 800;
  if (hpa > 1200) hpa = 1200;

  curQnhHpa = hpa;
  halo_set_qnh_runtime_and_persist(curQnhHpa);          // persists + re-anchors baseline on ground
  pQnhCharacteristic->setValue((uint8_t*)&hpa, 2);      // echo the stored value
  Serial.printf("[BLE] QNH=%u hPa (saved)\n", hpa);
  return;
}


    if (c == pResetCharacteristic) {
      Serial.println("[BLE] RESET requested");
      delay(50);
      esp_restart();
      return;
    }

    if (c == pDataSourceCharacteristic) {
      log_payload("DATASRC write", v);
      bool isSoftRF = false;
      // Accept ASCII "0/1", "FLARM/SOFTRF", or binary 0x00/0x01
      if (!v.empty()) {
        if (v == std::string("FLARM")) isSoftRF = false;
        else if (v == std::string("SOFTRF")) isSoftRF = true;
        else if (v[0] == '0') isSoftRF = false;
        else if (v[0] == '1') isSoftRF = true;
        else isSoftRF = ((uint8_t)v[0]) != 0;
      }
      uint8_t idx = 0;
      if (v.size() >= 2) {
        // Second byte (or ASCII digit) as baud index
        if (v[1] >= '0' && v[1] <= '9') idx = (uint8_t)(v[1]-'0');
        else idx = (uint8_t)v[1];
      }
      if (idx > 1) idx = 1;

      curIsSoftRF = isSoftRF;
      if (isSoftRF) curBaudIdxSoft = idx; else curBaudIdxFlarm = idx;

      halo_set_datasource_and_baud(curIsSoftRF,
        (uint8_t)(curIsSoftRF ? curBaudIdxSoft : curBaudIdxFlarm));
      applyBaudFromIndices();

      uint8_t payload[2] = {
        (uint8_t)(curIsSoftRF ? 1 : 0),
        (uint8_t)(curIsSoftRF ? curBaudIdxSoft : curBaudIdxFlarm)
      };
      pDataSourceCharacteristic->setValue(payload, 2);
      Serial.printf("[BLE] DS=%s, idx=%u (saved)\n",
        curIsSoftRF ? "SoftRF" : "FLARM", (unsigned)payload[1]);
      return;
    }
  }

  void onRead(BLECharacteristic* c) override {
    if (c == pTestCharacteristic) {
      uint8_t state = testActive ? 1 : 0;
      c->setValue(&state, 1);
    } else if (c == pVolumeCharacteristic) {
      pVolumeCharacteristic->setValue(&curVolume, 1);
    } else if (c == pElevationCharacteristic) {
      pElevationCharacteristic->setValue((uint8_t*)&curElevationFeet, 2);
    } else if (c == pQnhCharacteristic) {
      pQnhCharacteristic->setValue((uint8_t*)&curQnhHpa, 2);
    } else if (c == pDataSourceCharacteristic) {
      uint8_t payload[2] = {
        (uint8_t)(curIsSoftRF ? 1 : 0),
        (uint8_t)(curIsSoftRF ? curBaudIdxSoft : curBaudIdxFlarm)
      };
      c->setValue(payload, 2);
    }
  }
}; // close class

static BLECharacteristic* mkChar(BLEService* s, const char* uuid, uint32_t props) {
  BLECharacteristic* c = s->createCharacteristic(uuid, props);
  static MyCharCallbacks cb; // static lifetime
  c->setCallbacks(&cb);
  return c;
}

static void initializeBLE() {
  Serial.println("[BLE] init...");
  BLEDevice::init("HALO Control");
  static MyServerCallbacks scb;
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&scb);
  pService = pServer->createService(SERVICE_UUID);

  pFlashCharacteristic      = mkChar(pService, FLASH_CHARACTERISTIC_UUID,     BLECharacteristic::PROPERTY_WRITE);
  pTestCharacteristic       = mkChar(pService, TEST_CHARACTERISTIC_UUID,      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pVolumeCharacteristic     = mkChar(pService, VOLUME_CHARACTERISTIC_UUID,    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pElevationCharacteristic  = mkChar(pService, ELEVATION_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pQnhCharacteristic        = mkChar(pService, QNH_CHARACTERISTIC_UUID,       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pResetCharacteristic      = mkChar(pService, RESET_CHARACTERISTIC_UUID,     BLECharacteristic::PROPERTY_WRITE);
  pDataSourceCharacteristic = mkChar(pService, DATASOURCE_CHAR_UUID,          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
}

static void seedValuesFromRuntime() {
  curVolume         = df_volume;
  curElevationFeet  = (uint16_t)max(0.0f, airfieldElev_ft);
  curQnhHpa         = (uint16_t)max(0.0f, qnh_hPa);

  curIsSoftRF       = false;
  curBaudIdxFlarm   = 0;
  curBaudIdxSoft    = 1;

  pVolumeCharacteristic->setValue(&curVolume, 1);
  pElevationCharacteristic->setValue((uint8_t*)&curElevationFeet, 2);
  pQnhCharacteristic->setValue((uint8_t*)&curQnhHpa, 2);

  uint8_t payload[2] = {
    (uint8_t)(curIsSoftRF ? 1 : 0),
    (uint8_t)(curIsSoftRF ? curBaudIdxSoft : curBaudIdxFlarm)
  };
  pDataSourceCharacteristic->setValue(payload, 2);

  uint8_t st = testActive ? 1 : 0;
  pTestCharacteristic->setValue(&st, 1);
}

static void finalizeBLE() {
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] service started & advertising");
}

// ---- Public API ----
void bleInit() {
  initializeBLE();
  seedValuesFromRuntime();
  finalizeBLE();
}

void bleTick(uint32_t now){
  runTestSequence(now);
}

void bleCancelTests(){
  testActive = false;
  testSequenceStep = 0;
  lastTestStepTime = 0;
  Serial.println("[BLE] TEST sequence cancelled");
}
