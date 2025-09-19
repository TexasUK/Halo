#pragma once
#include <Arduino.h>

// --- Live telemetry from sensors/NMEA ---
struct Telemetry {
  float tC        = NAN;
  float p_hPa     = NAN;
  float alt_m     = NAN;     // MSL meters (AGL computed elsewhere from baseline)
  bool  bmp_ok    = false;

  float sog_kts   = NAN;     // speed over ground (kn)
  float track_deg = NAN;     // course/track (deg)

  uint32_t last_nmea_ms = 0;
  float    vs_ms        = 0.0f;  // vertical speed (m/s), derived

  // UTC from RMC (HH:MM). -1 means unknown.
  int   utc_hour  = -1;     // 0..23
  int   utc_min   = -1;     // 0..59
};
extern Telemetry tele;

// --- Traffic alert snapshot from PFLAA ---
struct TrafficAlert {
  bool     active      = false;
  uint32_t since       = 0;     // millis() when last alert data arrived
  float    relN_m      = 0;     // relative North (m)
  float    relE_m      = 0;     // relative East (m)
  float    relV_m      = 0;     // relative vertical (m)
  float    dist_m      = 0;     // planar distance (m)
  float    bearing_deg = 0;     // absolute bearing (deg)
  int      alarm       = 0;     // 0..3(+)
};
extern TrafficAlert alert;

// --- AGL baseline (captured on ground) ---
extern bool  baselineSet;
extern float baselineAlt_m;  // meters MSL at capture

// Optional external: sea-level pressure you use as QNH elsewhere
extern float seaLevel_hPa;
