#include "app_fsm.h"
#include "constants.h"
#include "telemetry.h"
#include "ui_iface.h"

#include "../drivers/dfplayer.h"
#include "../nav/flarm.h"
#include "../storage/nvs_store.h"   // nvs_record_flight()

// ---- Externals owned elsewhere ----
extern Telemetry    tele;
extern TrafficAlert alert;
extern bool         baselineSet;
extern float        baselineAlt_m;
extern void         strobeEnable(bool en);
extern void         strobeSet(uint16_t on_ms, uint16_t period_ms);  // cadence control

// ---- Global FSM state ----
AppState g_state = ST_BOOT;

// ---- Startup grace: ignore takeoff triggers briefly after init ----
static uint32_t fsm_init_ms = 0;
static const uint32_t STARTUP_GRACE_MS = 4000;  // 4 seconds

// ---- Landing/Landed holds ----
static const uint32_t LANDING_AGL_HOLD_MS   = 2000;  // AGL ≤ threshold for this long -> LANDING
static const uint32_t LANDED_SLOW_HOLD_MS   = 3000;  // kts < 5 for this long -> LANDED

// ---- Bench/test helpers ----
static volatile bool demo_force_landing = false;
static uint32_t      demo_land_inhibit_until = 0;    // block landing checks during bench “force flying”
void app_demo_force_landing(){ demo_force_landing = true; }

// ---- Flight stats/timers ----
static uint32_t flightStart_ms     = 0;
static uint32_t lastFlightDur_ms   = 0;
static uint16_t lastFlightAlerts   = 0;

static uint32_t ktsHiStart_ms    = 0;  // > TAKEOFF_KTS hold
static uint32_t altHiStart_ms    = 0;  // > TAKEOFF_ALT_FT (AGL) hold fallback
static uint32_t landLowStart_ms  = 0;  // ≤ LANDING_ALT_FT (AGL) hold
static uint32_t landedSlow_ms    = 0;  // < 5 kts hold
static uint32_t landingShown_ms  = 0;  // when Landing page shown
static uint32_t trafficHold_ms   = 0;  // min hold on TRAFFIC page
static uint32_t lastAlertStamp   = 0;  // de-dupe alert entries

// ---- Helpers ----
static inline float ft_from_m(float m){ return m * 3.28084f; }
static inline float agl_ft(){
  if(!baselineSet || isnan(tele.alt_m)) return NAN;
  return ft_from_m(tele.alt_m - baselineAlt_m);
}

// ---- Strobe cadence management ----
static const uint16_t STROBE_STD_ON_MS  = STROBE_ON_MS; // 120ms
static const uint16_t STROBE_STD_PERIOD = 2000;

static const uint16_t STROBE_L1_PERIOD  = 1400;  // gentle
static const uint16_t STROBE_L2_PERIOD  = 900;   // faster
static const uint16_t STROBE_L3_PERIOD  = 500;   // fastest

static int last_strobe_level = 0; // 0=std, 1..3=alert levels

static inline void strobe_std(){
  strobeSet(STROBE_STD_ON_MS, STROBE_STD_PERIOD);
  last_strobe_level = 0;
  Serial.printf("[STROBE] standard cadence: on=%ums period=%ums\n",
                (unsigned)STROBE_STD_ON_MS, (unsigned)STROBE_STD_PERIOD);
}
static inline void strobe_alert_level(int lvl){
  uint16_t per = (lvl>=3)?STROBE_L3_PERIOD : (lvl==2)?STROBE_L2_PERIOD : STROBE_L1_PERIOD;
  strobeSet(STROBE_STD_ON_MS, per);
  last_strobe_level = lvl;
  Serial.printf("[STROBE] alert L%d cadence: on=%ums period=%ums\n",
                lvl, (unsigned)STROBE_STD_ON_MS, (unsigned)per);
}

// ---- Audio guards ----
static bool takeoffChimed = false;

// ---- Unified state entries ----
static void fsm_enter_flying(uint32_t now) {
  g_state = ST_FLYING;

  // entry actions
  strobeEnable(true);
  strobe_std();              // reset cadence baseline
  ui_set_page(PAGE_COMPASS); // Cruise

  // chime once per flight
  if (!takeoffChimed) {
    dfp_stop_and_flush();
    dfp_play_filename(3);    // takeoff
    takeoffChimed = true;
  }

  if (flightStart_ms == 0) flightStart_ms = now;
  Serial.println("[FSM] enter ST_FLYING");
}

static void fsm_enter_alert(uint32_t now) {
  g_state = ST_ALERT;
  ui_set_page(PAGE_TRAFFIC);

  // bump stats (dedupe by time, simple 250ms edge)
  if (now - lastAlertStamp > 250) {
    lastFlightAlerts++;
    lastAlertStamp = now;
  }

  // strobe cadence by level
  int lvl = alert.alarm;
  if (lvl < 1) lvl = 1;
  if (lvl > 3) lvl = 3;
  if (last_strobe_level != lvl) strobe_alert_level(lvl);

  // minimum page hold for smooth UX
  trafficHold_ms = now + ALERT_HOLD_MS;

  Serial.println("[FSM] enter ST_ALERT");
}

static void fsm_enter_landing(uint32_t now) {
  g_state = ST_LANDING;
  ui_set_page(PAGE_LANDING);
  landingShown_ms = now;

  // Stop strobes immediately on landing phase
  strobeEnable(false);

  // landing chime
  dfp_stop_and_flush();
  dfp_play_filename(7);

  Serial.println("[FSM] enter ST_LANDING");
}

static void fsm_enter_landed(uint32_t now) {
  g_state = ST_LANDED;
  ui_set_page(PAGE_LANDED);

  // finalize stats
  if (flightStart_ms != 0) {
    lastFlightDur_ms = now - flightStart_ms;
  }
  // persist snapshot to NVS
  nvs_record_flight(lastFlightDur_ms, lastFlightAlerts,
                    tele.utc_hour, tele.utc_min);

  // reset takeoff chime for next flight
  takeoffChimed = false;

  Serial.println("[FSM] enter ST_LANDED (stats recorded)");
}

// ---- Public API ----
void app_fsm_init(){
  g_state = ST_PREFLIGHT;
  ui_set_page(PAGE_BOOT);

  fsm_init_ms = millis();         // start grace window
  demo_force_landing = false;
  demo_land_inhibit_until = 0;

  ktsHiStart_ms = altHiStart_ms = landLowStart_ms = landedSlow_ms =
  landingShown_ms = trafficHold_ms = lastAlertStamp = 0;

  flightStart_ms    = 0;
  lastFlightDur_ms  = 0;
  lastFlightAlerts  = 0;
  takeoffChimed     = false;

  strobeEnable(false);            // on ground, no strobes
  strobe_std();                   // reset cadence baseline

  Serial.println("[BOOT] app_fsm_init complete");
}

static inline bool new_alert_active(uint32_t now) {
  // Consider alert "active" if alert.active is true and the data is fresh
  // (nav layer should keep alert.since updated). We can also require minimal
  // freshness if needed; for now trust alert.active.
  (void)now;
  return alert.active && (alert.alarm > 0);
}

void app_fsm_tick(uint32_t now){
  // --- Handle demo bench flags first ---
  if (demo_force_landing) {
    demo_force_landing = false;
    fsm_enter_landing(now);
  }

  // --- Compute AGL and speed guards ---
  float kts = tele.sog_kts;
  float agl = agl_ft();

  // --- Startup grace prevents instant fake takeoff right after boot ---
  bool in_grace = (now - fsm_init_ms) < STARTUP_GRACE_MS;

  switch (g_state) {
    case ST_PREFLIGHT: {
      // Takeoff by kts > TAKEOFF_KTS for TAKEOFF_HOLD_MS
      if (!in_grace && kts == kts && kts > TAKEOFF_KTS) {
        if (ktsHiStart_ms == 0) ktsHiStart_ms = now;
        if (now - ktsHiStart_ms >= TAKEOFF_HOLD_MS) {
          fsm_enter_flying(now);
          break;
        }
      } else {
        ktsHiStart_ms = 0;
      }

      // Fallback: AGL > TAKEOFF_ALT_FT for TAKEOFF_HOLD_MS
      if (!in_grace && agl == agl && agl > TAKEOFF_ALT_FT) {
        if (altHiStart_ms == 0) altHiStart_ms = now;
        if (now - altHiStart_ms >= TAKEOFF_HOLD_MS) {
          fsm_enter_flying(now);
          break;
        }
      } else {
        altHiStart_ms = 0;
      }
    } break;

    case ST_FLYING: {
      // Traffic arrival -> ALERT
      if (new_alert_active(now)) {
        fsm_enter_alert(now);
        break;
      }

      // Landing: AGL ≤ LANDING_ALT_FT for LANDING_AGL_HOLD_MS
      if (agl == agl && agl <= LANDING_ALT_FT && now >= demo_land_inhibit_until) {
        if (landLowStart_ms == 0) landLowStart_ms = now;
        if (now - landLowStart_ms >= LANDING_AGL_HOLD_MS) {
          fsm_enter_landing(now);
          break;
        }
      } else {
        landLowStart_ms = 0;
      }
    } break;

    case ST_ALERT: {
      // Maintain TRAFFIC page minimum hold
      bool min_hold_done = (now >= trafficHold_ms);

      // If alert cleared and min hold elapsed -> back to FLYING screen/cadence
      if (( !new_alert_active(now) ) && min_hold_done) {
        ui_set_page(PAGE_COMPASS);
        // reset to standard cadence when no alert is active
        if (last_strobe_level != 0) strobe_std();
        g_state = ST_FLYING; // state returns to FLYING
      }

      // Landing rule still applies during ALERT
      if (agl == agl && agl <= LANDING_ALT_FT && now >= demo_land_inhibit_until) {
        if (landLowStart_ms == 0) landLowStart_ms = now;
        if (now - landLowStart_ms >= LANDING_AGL_HOLD_MS) {
          fsm_enter_landing(now);
          break;
        }
      } else {
        landLowStart_ms = 0;
      }

      // Keep cadence in sync with current alert level
      if (new_alert_active(now)) {
        int lvl = alert.alarm;
        if (lvl < 1) lvl = 1;
        if (lvl > 3) lvl = 3;
        if (last_strobe_level != lvl) strobe_alert_level(lvl);
      }
    } break;

    case ST_LANDING: {
      // After showing LANDING for LANDING_HOLD_MS, we allow LANDED detection
      bool landing_ui_done = (now - landingShown_ms) >= LANDING_HOLD_MS;

      // When <5 kts for LANDED_SLOW_HOLD_MS -> LANDED
      if (landing_ui_done) {
        if (kts == kts && kts < 5.0f) {
          if (landedSlow_ms == 0) landedSlow_ms = now;
          if (now - landedSlow_ms >= LANDED_SLOW_HOLD_MS) {
            fsm_enter_landed(now);
            break;
          }
        } else {
          landedSlow_ms = 0;
        }
      }
    } break;

    case ST_LANDED: {
      // Idle; waiting for new session. Nothing to do here.
    } break;

    default:
      break;
  }
}

// ---- Landed stats exposure ----
uint32_t app_last_flight_duration_ms(){ return lastFlightDur_ms; }
uint16_t app_last_flight_alerts()    { return lastFlightAlerts; }

// ---- Bench/demo helpers ----
void app_demo_force_flying(){
  uint32_t now = millis();

  // while bench-testing, briefly inhibit landing checks so alerts can be tested
  demo_land_inhibit_until = now + 8000; // 8s
  
  // enter flying state (will play chime if not yet played)
  fsm_enter_flying(now);
}

void app_demo_extend_land_inhibit(uint32_t ms){
  uint32_t now = millis();
  uint32_t tgt = now + ms;
  if (tgt > demo_land_inhibit_until) demo_land_inhibit_until = tgt;
}
