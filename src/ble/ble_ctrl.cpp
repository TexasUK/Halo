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
static bool parse_u16_any(const std::string& v, uint16_t& out){
  if (v.empty()) return false;

  // Prefer ASCII if it looks like ASCII digits (common with mobile apps)
  if (is_ascii_digits(v)) {
    // std::string isn't null-terminated, so copy
    char buf[16] = {0};
    size_t n = min(v.size(), sizeof(buf)-1);
    memcpy(buf, v.data(), n);
    unsigned long val = strtoul(buf, nullptr, 10);
    out = (uint16_t)min(val, 655UL);
    // allow large inputs but clamp sensibly for QNH/ELEV contexts in caller
    return true;
  }

  // If at least 2 bytes, assume little-endian 16-bit
  if (v.size() >= 2) {
    out = (uint16_t)((uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8));
    return true;
  }

  // If a single byte, use it
  if (v.size() == 1) {
    out = (uint16_t)(uint8_t)v[0];
    return true;
  }

  return false;
}
static bool parse_bool_any(const std::string& v, bool& out){
  if (v.empty()) return false;
  if (is_ascii_digits(v)) { out = (v[0] != '0'); return true; }
  if (v == std::string("FLARM")) { out = false; return true; }
  if (v == std::string("SOFTRF")) { out = true; return true; }
  out = ((uint8_t)v[0]) != 0;
  return true;
}

// --- Helpers: alert injection + speech ---
static void injectAlert(int level, float bearing_deg, float relV_m) {
  extern TrafficAlert alert; // from telemetry.h
  alert.active = true;
  alert.since  = millis();
  alert.alarm  = level;

  // Place target ~1km at requested bearing (E=sin, N=cos)
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
    // keep bench "flying" so auto-landing doesn't steal the show
    extern void app_demo_extend_land_inhibit(uint32_t ms);
    app_demo_extend_land_inhibit(8000);
    return;
  }
  lastTestStepTime = now;
  testSequenceStep++;

  switch (testSequenceStep) {
    case 1:
      dbg("[TEST] Step 1: Takeoff");
      app_demo_force_flying();  // FSM will chime track 3 on entry
      break;

    case 2:
      dbg("[TEST] Step 2: Alert HIGH @ 2 o'clock");
      injectAlert(2, 60.0f, +70.0f);    // HIGH ≈ +70 m
      speakVerticalAndClock(2, "HIGH");
      break;

    case 3:
      dbg("[TEST] Step 3: Alert LOW @ 10 o'clock");
      injectAlert(3, 300.0f, -70.0f);   // LOW ≈ -70 m
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
      tele.sog_kts = 0.0f;              // allow ST_LANDED gate to complete
      app_demo_force_landing();         // FSM plays landing chime 7
      testActive = false;               // stop after one cycle
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
  halo_apply_nav_baud(baud); // app-level hook reopens UART(2)
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
      // 0x00 = stop, anything else (or empty) = start+reset.
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
      app_fsm_init();       // resets FSM & strobes
      ui_markAllUndrawn();  // full redraw next frame
      ui_set_page(PAGE_BOOT);
    }
      uint8_t state = testActive ? 1 : 0;
      pTestCharacteristic->setValue(&state, 1);
      return;
    }

    if (c == pVolumeCharacteristic) {
      log_payload("VOLUME write", v);
      uint16_t u16;
      if (parse_u16_any(v, u16)) {
        uint8_t vol = (uint8_t)constrain((int)u16, 0, 30);
        curVolume = vol;
        halo_set_volume_runtime_and_persist(vol);  // updates df_volume + NVS + live DFPlayer (via main)
        pVolumeCharacteristic->setValue(&curVolume, 1);
        Serial.printf("[BLE] VOL=%u (saved)\n", curVolume);
      } else {
        Serial.println("[BLE] VOL parse failed");
      }
      return;
    }

    if (c == pElevationCharacteristic) {
      log_payload("ELEV write", v);
      uint16_t feet;
    if (parse_u16_any(v, feet)) {
      // Android app sends tens-of-feet → convert to feet
      uint16_t feetScaled = (uint16_t)min(65535, (int)feet * 10);
      feetScaled = (uint16_t)constrain((int)feetScaled, 0, 30000); // sensible clamp
      curElevationFeet = feetScaled;
      halo_set_elev_runtime_and_persist(curElevationFeet);         // runtime + NVS
      pElevationCharacteristic->setValue((uint8_t*)&feetScaled, 2); // echo back stored value
      Serial.printf("[BLE] ELEV=%u ft (saved)\n", feetScaled);
    } else {
      Serial.println("[BLE] ELEV parse failed");
    }
      return;
    }

    if (c == pQnhCharacteristic) {
      log_payload("QNH write", v);
      uint16_t hpa;
    if (parse_u16_any(v, hpa)) {
      // Android app sends hPa/10 → convert to hPa
      uint16_t hpaScaled = (uint16_t)min(65535, (int)hpa * 10);
      hpaScaled = (uint16_t)constrain((int)hpaScaled, 800, 1100);  // keep realistic
      curQnhHpa = hpaScaled;
      halo_set_qnh_runtime_and_persist(curQnhHpa);                  // runtime + NVS
      pQnhCharacteristic->setValue((uint8_t*)&hpaScaled, 2);        // echo stored
      Serial.printf("[BLE] QNH=%u hPa (saved)\n", hpaScaled);
    } else {
      Serial.println("[BLE] QNH parse failed");
    }
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
      // Accept ASCII "0/1", "FLARM/SOFTRF", or binary 0x00/0x01; optional second byte/ASCII for baud index (0/1)
      bool isSoftRF;
      if (!parse_bool_any(v, isSoftRF)) { isSoftRF = false; }
      uint8_t idx = 0;
      if (v.size() >= 2) {
        // second byte as idx (binary or ASCII digit)
        if (is_ascii_digits(v.substr(1,1))) idx = (uint8_t)(v[1]-'0');
        else idx = (uint8_t)v[1];
      }
      if (idx > 1) idx = 1;

      curIsSoftRF = isSoftRF;
      if (isSoftRF) curBaudIdxSoft = idx; else curBaudIdxFlarm = idx;

      // Persist selection and apply baud via app hook
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

  // default mirrors (you can pre-seed from NVS via main before bleInit())
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
