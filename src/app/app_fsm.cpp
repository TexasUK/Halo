#include "app_fsm.h"
#include "constants.h"
#include "telemetry.h"
#include "ui_iface.h"
#include "nav/flarm.h"
#include "drivers/dfplayer.h"
#include "storage/nvs_store.h"

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

// ---- Bench/test helpers ----
static volatile bool demo_force_landing = false;
static uint32_t demo_land_inhibit_until = 0;    // block landing checks during bench “force flying”
void app_demo_force_landing(){ demo_force_landing = true; }
void app_demo_extend_land_inhibit(uint32_t ms){
  uint32_t now = millis();
  uint32_t until = now + ms;
  if (until > demo_land_inhibit_until) demo_land_inhibit_until = until;
}

// ---- Flight stats/timers ----
static uint32_t flightStart_ms     = 0;
static uint32_t lastFlightDur_ms   = 0;
static uint16_t flightAlertCount   = 0;

static uint32_t ktsHiStart_ms    = 0;  // > TAKEOFF_KTS hold
static uint32_t altHiStart_ms    = 0;  // > TAKEOFF_ALT_FT hold (fallback)
static uint32_t landLowStart_ms  = 0;  // <= LANDING_ALT_FT hold
static uint32_t landedSlow_ms    = 0;  // < 5 kts hold
static uint32_t landingShown_ms  = 0;  // when Landing page shown
static uint32_t trafficHold_ms   = 0;  // min hold on TRAFFIC page
static uint32_t lastAlertStamp   = 0;  // de-dupe alert entries

// Require a real climb before landing is allowed
static bool  landing_armed = false;

// ---- Helpers ----
static inline float ft_from_m(float m){ return m * 3.28084f; }
static inline float agl_ft(){
  if(!baselineSet || isnan(tele.alt_m)) return NAN;
  return ft_from_m(tele.alt_m - baselineAlt_m);
}

// ---- Strobe cadence management ----
static const uint16_t STROBE_STD_ON_MS  = STROBE_ON_MS; // from constants.h
static const uint16_t STROBE_L0_PERIOD  = 2000;
static const uint16_t STROBE_L1_PERIOD  = 1400;
static const uint16_t STROBE_L2_PERIOD  =  900;
static const uint16_t STROBE_L3_PERIOD  =  500;

static int last_strobe_level = 0; // 0=std, 1..3=alert levels

static inline void strobe_std(){
  strobeSet(STROBE_STD_ON_MS, STROBE_L0_PERIOD);
  last_strobe_level = 0;
  Serial.printf("[STROBE] standard cadence: on=%ums period=%ums\n",
                (unsigned)STROBE_STD_ON_MS, (unsigned)STROBE_L0_PERIOD);
}
static inline void strobe_alert_level(int lvl){
  uint16_t per = (lvl>=3)?STROBE_L3_PERIOD : (lvl==2)?STROBE_L2_PERIOD : STROBE_L1_PERIOD;
  strobeSet(STROBE_STD_ON_MS, per);
  last_strobe_level = lvl;
  Serial.printf("[STROBE] alert L%d cadence: on=%ums period=%ums\n",
                lvl, (unsigned)STROBE_STD_ON_MS, (unsigned)per);
}

void app_fsm_init(){
  g_state = ST_PREFLIGHT;
  ui_set_page(PAGE_BOOT);

  fsm_init_ms = millis();         // start grace window
  demo_force_landing = false;
  demo_land_inhibit_until = 0;

  ktsHiStart_ms = altHiStart_ms = landLowStart_ms = landedSlow_ms =
  landingShown_ms = trafficHold_ms = lastAlertStamp = 0;

  lastFlightDur_ms = 0;
  flightAlertCount = 0;
  landing_armed    = false;

  strobeEnable(false);            // on ground, no strobes
  strobe_std();                   // reset cadence baseline
}

// Force FLYING so test alerts actually flow through the FSM (and strobe cadence updates)
void app_demo_force_flying(){
  uint32_t now = millis();
  g_state = ST_FLYING;
  strobeEnable(true);
  strobe_std();                   // standard cadence on “takeoff”
  ui_set_page(PAGE_COMPASS);

  // Start a fresh flight so LANDED duration works in tests
  flightStart_ms   = now;
  flightAlertCount = 0;
  landing_armed    = false;

  // BENCH: inhibit landing detection briefly so ground AGL doesn’t end the flight
  demo_land_inhibit_until = now + 8000;  // 8 seconds

  dfp_play_filename(3);   // takeoff cue
}

void app_fsm_tick(uint32_t now){
  const bool  nav_ok      = navValid();
  const float kts         = tele.sog_kts;
  const float agl         = agl_ft();
  const bool  alert_alive = alert.active && (now - alert.since) < ALERT_HOLD_MS;

  switch(g_state){

    case ST_PREFLIGHT: {
      // Bench force-landing shortcut
      if (demo_force_landing) {
        demo_force_landing = false;
        g_state = ST_LANDING;
        strobeEnable(false);
        dfp_play_filename(7);               // Landing cue
        ui_set_page(PAGE_LANDING);
        landingShown_ms = now;
        break;
      }

      // Ignore takeoff triggers right after startup
      if (now - fsm_init_ms < STARTUP_GRACE_MS) {
        ktsHiStart_ms = altHiStart_ms = 0;
        break;
      }

      // Primary takeoff: nav valid + speed
      if (nav_ok && !isnan(kts) && kts > TAKEOFF_KTS) {
        if (!ktsHiStart_ms) ktsHiStart_ms = now;
        if (now - ktsHiStart_ms >= TAKEOFF_HOLD_MS) {
          g_state = ST_FLYING;
          strobeEnable(true);
          strobe_std();                     // standard cadence on departure
          dfp_play_filename(3);             // Takeoff
          ui_set_page(PAGE_COMPASS);        // we use PAGE_COMPASS for “Cruise” UI
          flightStart_ms   = now;
          flightAlertCount = 0;
          landing_armed    = false;         // not armed yet
          demo_land_inhibit_until = now + 3000; // small inhibit even in real path
        }
      } else ktsHiStart_ms = 0;

      // Fallback takeoff: AGL high (if nav not valid)
      if (!nav_ok && !isnan(agl) && agl > TAKEOFF_ALT_FT) {
        if (!altHiStart_ms) altHiStart_ms = now;
        if (now - altHiStart_ms >= TAKEOFF_HOLD_MS) {
          g_state = ST_FLYING;
          strobeEnable(true);
          strobe_std();
          dfp_play_filename(3);
          ui_set_page(PAGE_COMPASS);
          flightStart_ms   = now;
          flightAlertCount = 0;
          landing_armed    = false;         // must climb past threshold to arm
          demo_land_inhibit_until = now + 3000;
        }
      } else altHiStart_ms = 0;
    } break;

    case ST_FLYING: {
      // Arm landing once we’ve seen AGL > TAKEOFF_ALT_FT at least once
      if (!landing_armed && !isnan(agl) && agl > TAKEOFF_ALT_FT) landing_armed = true;

      // Bench force-landing
      if (demo_force_landing) {
        demo_force_landing = false;
        g_state = ST_LANDING;
        strobeEnable(false);
        dfp_play_filename(7);
        ui_set_page(PAGE_LANDING);
        landingShown_ms = now;
        break;
      }

      // Enter ALERT (and hold TRAFFIC for a minimum time)
      if (alert_alive && alert.since != lastAlertStamp) {
        lastAlertStamp = alert.since;
        flightAlertCount++;
        g_state = ST_ALERT;
        ui_set_page(PAGE_TRAFFIC);
        trafficHold_ms = max(now + 1800u, alert.since + ALERT_HOLD_MS); // min show time

        // Speed up strobe by alert level
        if (last_strobe_level != alert.alarm) strobe_alert_level(alert.alarm);
      }

      // Landing detection by AGL (honor bench inhibit) — only if armed
      if (landing_armed && now >= demo_land_inhibit_until && !isnan(agl) && agl <= LANDING_ALT_FT) {
        if (!landLowStart_ms) landLowStart_ms = now;
        if (now - landLowStart_ms >= 2000) {
          g_state = ST_LANDING;
          strobeEnable(false);
          dfp_play_filename(7);
          ui_set_page(PAGE_LANDING);
          landingShown_ms = now;
        }
      } else landLowStart_ms = 0;
    } break;

    case ST_ALERT: {
      // Keep strobe rate in sync with current alert level
      if (last_strobe_level != alert.alarm) strobe_alert_level(alert.alarm);

      // Exit ALERT when expired & hold satisfied
      if (!alert_alive && now >= trafficHold_ms) {
        g_state = ST_FLYING;
        ui_set_page(PAGE_COMPASS);
        strobe_std();                        // revert cadence when alert clears
      }

      // Allow landing inside ALERT (honor bench inhibit) — only if armed
      if (landing_armed && now >= demo_land_inhibit_until && !isnan(agl) && agl <= LANDING_ALT_FT) {
        if (!landLowStart_ms) landLowStart_ms = now;
        if (now - landLowStart_ms >= 2000) {
          g_state = ST_LANDING;
          strobeEnable(false);
          dfp_play_filename(7);
          ui_set_page(PAGE_LANDING);
          landingShown_ms = now;
        }
      } else landLowStart_ms = 0;
    } break;

    case ST_LANDING: {
      // Transition to LANDED when slow <5 kts for 3 s
      if (!isnan(kts) && kts < 5.0f) {
        if (!landedSlow_ms) landedSlow_ms = now;
        if (now - landedSlow_ms >= 3000) {
          g_state = ST_LANDED;
          if (flightStart_ms) lastFlightDur_ms = now - flightStart_ms;
          ui_set_page(PAGE_LANDED);

          // Persist this flight in NVS (duration, alert count, UTC snapshot)
          nvs_record_flight(
            lastFlightDur_ms,
            flightAlertCount,
            tele.utc_hour, tele.utc_min
          );
        }
      } else landedSlow_ms = 0;

      // No automatic path back to Cruise here.
    } break;

    case ST_LANDED: {
      // Persistent until power-off (or a manual reset you add later)
    } break;

    case ST_BOOT:
    default:
      g_state = ST_PREFLIGHT;
      ui_set_page(PAGE_BOOT);
      break;
  }
}

// ---- Landed stats getters ----
uint32_t app_last_flight_duration_ms(){ return lastFlightDur_ms; }
uint16_t app_last_flight_alerts(){ return flightAlertCount; }
