#pragma once
#include <Arduino.h>

enum HaloDataSource : uint8_t {
  HALO_SRC_FLARM  = 0,   // 19200 baud
  HALO_SRC_SOFTRF = 1    // 38400 baud
};

struct HaloSettings {
  float   qnh_hPa         = 1013.25f;
  float   airfieldElev_ft = 0.0f;
  uint8_t volume0_30      = 24;
  bool    baselineSet     = false;
  float   baselineAlt_m   = NAN;

  // NEW: persisted data source
  HaloDataSource data_source = HALO_SRC_FLARM;
};

struct HaloFlightStats {
  uint32_t flights_count   = 0;
  uint64_t total_time_ms   = 0;
  uint32_t total_alerts    = 0;
  uint32_t last_flight_ms  = 0;
  uint16_t last_alerts     = 0;
  int8_t   last_utc_hour   = -1;
  int8_t   last_utc_min    = -1;
};

bool nvs_init();
bool nvs_load_settings(HaloSettings& out);
bool nvs_save_settings(const HaloSettings& cfg);
bool nvs_load_flight(HaloFlightStats& out);
bool nvs_save_flight(const HaloFlightStats& st);
bool nvs_record_flight(uint32_t flight_ms, uint16_t alerts, int utc_hour, int utc_min);
bool nvs_factory_reset();
