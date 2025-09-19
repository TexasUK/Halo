#include "test.h"
#include "../nav/flarm.h"
#include "../app/telemetry.h"
#include "../app/constants.h"

enum Phase { TP_IDLE, TP_ARM, TP_TAKEOFF, TP_CRUISE, TP_A1, TP_A2, TP_A3, TP_DESCEND, TP_DONE };
static Phase phase = TP_IDLE;
static uint32_t t0 = 0;

// heartbeat timer
static uint32_t last_hb_ms = 0;

// baro sim
static bool     have_baseline   = false;
static float    base_msl_m      = 0.0f;   // copy of baselineAlt_m at test start
static float    sim_alt_m       = 0.0f;   // simulated current MSL altitude
static float    climb_rate_mps  = 0.0f;   // +/-, applied each tick

// one-shot flags for PFLAA sends
static bool sent_a1=false, sent_a2=false, sent_a3=false;

// External telemetry globals
extern Telemetry tele;
extern bool  baselineSet;
extern float baselineAlt_m;

// Helpers to generate NMEA quickly
static void send_rmc(float sog_kts, float cog_deg, bool valid=true){
  char line[128];
  snprintf(line, sizeof(line),
    "$GNRMC,120000.000,%c,,,,,,%.1f,%.1f,010101,,,A*00\n",
    valid?'A':'V', sog_kts, cog_deg);
  nav_inject_nmea(line);
}
static void send_gga(int sats){
  char line[128];
  snprintf(line, sizeof(line),
    "$GNGGA,120000.000,,,,,1,%d,1.0,0.0,M,0.0,M,,*00\n", sats);
  nav_inject_nmea(line);
}
static void send_pflaa(int alarm, float rn, float re, float rv){
  char line[128];
  snprintf(line, sizeof(line),
    "$PFLAA,%d,%.0f,%.0f,%.0f,1234,0,0,0,0,0*00\n", alarm, rn, re, rv);
  nav_inject_nmea(line);
}

// Heartbeat every 500 ms to keep navValid() solid
static void heartbeat(uint32_t now){
  if(now - last_hb_ms < 500) return;
  last_hb_ms = now;
  // keep a reasonable track & speed depending on phase
  float sog = 0.0f;
  switch(phase){
    case TP_TAKEOFF: sog = 25.0f; break;
    case TP_CRUISE:
    case TP_A1: case TP_A2: case TP_A3: sog = 80.0f; break;
    case TP_DESCEND: sog = 8.0f; break;
    default: sog = 0.0f; break;
  }
  send_gga(8);
  send_rmc(sog, 90.0f, true);
}

// Update simulated baro altitude AFTER updateBMP() has run
static void update_sim_alt(uint32_t now){
  static uint32_t last_ms = 0;
  if(!have_baseline){
    if(baselineSet && !isnan(baselineAlt_m)){
      have_baseline = true;
      base_msl_m    = baselineAlt_m;
      sim_alt_m     = base_msl_m;
    } else {
      return; // wait until baseline captured by BMP
    }
  }
  if(last_ms == 0) last_ms = now;
  float dt = (now - last_ms) / 1000.0f;
  last_ms = now;

  // integrate climb/descent
  sim_alt_m += climb_rate_mps * dt;

  // write into telemetry AFTER updateBMP (loop() calls test_tick after updateBMP)
  tele.alt_m = sim_alt_m;
}

bool test_is_running(){ return phase != TP_IDLE && phase != TP_DONE; }

void test_start(){
  phase = TP_ARM; t0 = millis();
  last_hb_ms = 0;
  have_baseline = false;
  sim_alt_m = 0.0f;
  climb_rate_mps = 0.0f;
  sent_a1=sent_a2=sent_a3=false;
  Serial.println(F("[TEST] Starting scripted scenario"));
}

void test_stop(){
  phase = TP_DONE;
  Serial.println(F("[TEST] Stopping test"));
}

void test_tick(uint32_t now){
  if(!test_is_running()) return;

  heartbeat(now);     // keep nav valid
  update_sim_alt(now);// drive baro AGL logic

  switch(phase){
    case TP_ARM:
      // wait 1s with nav valid on Boot
      if(now - t0 > 1000){
        phase = TP_TAKEOFF; t0 = now;
        // start climb ~ +3 m/s (~600 ft/min)
        climb_rate_mps = 3.0f;
      }
      break;

    case TP_TAKEOFF:
      // sustain >20 kts for 4s → takeoff logic will trigger; keep climbing
      if(now - t0 > 4000){
        phase = TP_CRUISE; t0 = now;
        // stop climbing, hold level
        climb_rate_mps = 0.0f;
      }
      break;

    case TP_CRUISE:
      // show Cruise for 2000 ms, no alerts
      if(now - t0 > 2000){
        phase = TP_A1; t0 = now;
      }
      break;

    case TP_A1:
      if(!sent_a1){
        sent_a1 = true;
        Serial.println(F("[TEST] A1: LEVEL alert"));
        send_pflaa(1, 800, 600, 0);     // level
      }
      if(now - t0 > 2500){
        phase = TP_A2; t0 = now;
      }
      break;

    case TP_A2:
      if(!sent_a2){
        sent_a2 = true;
        Serial.println(F("[TEST] A2: HIGH alert"));
        send_pflaa(2, 600, -800, 120);  // high (≈ +394 ft)
      }
      if(now - t0 > 2500){
        phase = TP_A3; t0 = now;
      }
      break;

    case TP_A3:
      if(!sent_a3){
        sent_a3 = true;
        Serial.println(F("[TEST] A3: LOW alert"));
        send_pflaa(3, -500, 900, -150); // low (≈ -492 ft)
      }
      if(now - t0 > 3000){
        phase = TP_DESCEND; t0 = now;
        // descend gently to AGL <= 200 ft
        climb_rate_mps = -2.5f;  // ~ -500 ft/min
      }
      break;

    case TP_DESCEND: {
      // target AGL ~ 50 m (≈ 164 ft)
      if(have_baseline){
        float agl_m = sim_alt_m - base_msl_m;
        if(agl_m <= 50.0f){
          // stop moving; let landing debounce & hold logic take over
          climb_rate_mps = 0.0f;
          // after landing hold window, finish test
          if(now - t0 > (LANDING_HOLD_MS + 3000)){
            phase = TP_DONE;
            Serial.println(F("[TEST] Done."));
          }
        }
      }
    } break;

    default: break;
  }
}
