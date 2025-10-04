#pragma once
#include <Arduino.h>

enum AppState : uint8_t {
  ST_BOOT,
  ST_PREFLIGHT,
  ST_FLYING,
  ST_ALERT,
  ST_LANDING,
  ST_LANDED
};

extern AppState g_state;

void app_fsm_init();              // call once devices are up
void app_fsm_tick(uint32_t now);  // call each loop after sensors/nav

// Landed stats exposure (used by Landed screen)
uint32_t app_last_flight_duration_ms();
uint16_t app_last_flight_alerts();

// Bench/demo helpers
void app_demo_force_landing();                 // force LANDING
void app_demo_force_flying();                  // force FLYING so alerts change cadence
void app_demo_extend_land_inhibit(uint32_t ms);// bench: hold off LANDING checks briefly

// NEW: tell FSM it’s safe to allow AGL fallback takeoff (baseline anchored this boot, or QNH adjusted on ground)
void app_preflight_mark_baseline_ok();
