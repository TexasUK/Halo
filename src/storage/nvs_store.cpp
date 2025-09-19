#include "nvs_store.h"
#include <Preferences.h>

static Preferences prefs;
static const char* NS        = "halo";
static const char* K_VER     = "ver";
static const uint8_t SCHEMA  = 1;

// settings keys
static const char* K_QNH     = "qnh";
static const char* K_ELEVFT  = "elevft";
static const char* K_VOL     = "vol";
static const char* K_BSET    = "bset";
static const char* K_BALT    = "balt";
static const char* K_DSRC    = "datasrc";

// flight stat keys
static const char* K_FCNT = "fcnt";
static const char* K_TMS  = "tms";
static const char* K_TAL  = "tal";
static const char* K_LMS  = "lms";
static const char* K_LAL  = "lal";
static const char* K_LH   = "lh";
static const char* K_LM   = "lm";

bool nvs_init(){
  if (!prefs.begin(NS, false)) return false;
  uint8_t cur = prefs.getUChar(K_VER, 0);
  if (cur != SCHEMA) {
    prefs.clear();
    prefs.putUChar(K_VER, SCHEMA);
  }
  prefs.end();
  return true;
}

bool nvs_load_settings(HaloSettings& out){
  if (!prefs.begin(NS, true)) return false;
  out.qnh_hPa         = prefs.getFloat (K_QNH,   out.qnh_hPa);
  out.airfieldElev_ft = prefs.getFloat (K_ELEVFT,out.airfieldElev_ft);
  out.volume0_30      = prefs.getUChar (K_VOL,   out.volume0_30);
  out.baselineSet     = prefs.getBool  (K_BSET,  out.baselineSet);
  out.baselineAlt_m   = prefs.getFloat (K_BALT,  out.baselineAlt_m);
  out.data_source     = (HaloDataSource)prefs.getUChar(K_DSRC,(uint8_t)out.data_source);
  prefs.end();
  return true;
}

bool nvs_save_settings(const HaloSettings& cfg){
  if (!prefs.begin(NS, false)) return false;
  prefs.putFloat (K_QNH,   cfg.qnh_hPa);
  prefs.putFloat (K_ELEVFT,cfg.airfieldElev_ft);
  prefs.putUChar (K_VOL,   cfg.volume0_30);
  prefs.putBool  (K_BSET,  cfg.baselineSet);
  prefs.putFloat (K_BALT,  cfg.baselineAlt_m);
  prefs.putUChar (K_DSRC,  (uint8_t)cfg.data_source);
  prefs.end();
  return true;
}

bool nvs_load_flight(HaloFlightStats& out){
  if (!prefs.begin(NS, true)) return false;
  out.flights_count  = prefs.getUInt(K_FCNT, 0);
  out.total_time_ms  = prefs.getULong64(K_TMS, 0);
  out.total_alerts   = prefs.getUInt(K_TAL, 0);
  out.last_flight_ms = prefs.getUInt(K_LMS, 0);
  out.last_alerts    = prefs.getUInt(K_LAL, 0);
  out.last_utc_hour  = prefs.getChar(K_LH, -1);
  out.last_utc_min   = prefs.getChar(K_LM, -1);
  prefs.end();
  return true;
}

bool nvs_save_flight(const HaloFlightStats& st){
  if (!prefs.begin(NS, false)) return false;
  prefs.putUInt   (K_FCNT, st.flights_count);
  prefs.putULong64(K_TMS,  st.total_time_ms);
  prefs.putUInt   (K_TAL,  st.total_alerts);
  prefs.putUInt   (K_LMS,  st.last_flight_ms);
  prefs.putUInt   (K_LAL,  st.last_alerts);
  prefs.putChar   (K_LH,   st.last_utc_hour);
  prefs.putChar   (K_LM,   st.last_utc_min);
  prefs.end();
  return true;
}

// Convenience: update cumulative totals + last flight in one shot
bool nvs_record_flight(uint32_t flight_ms, uint16_t alerts, int utc_hour, int utc_min){
  HaloFlightStats cur;
  nvs_load_flight(cur);

  cur.flights_count++;
  cur.total_time_ms += flight_ms;
  cur.total_alerts  += alerts;
  cur.last_flight_ms = flight_ms;
  cur.last_alerts    = alerts;
  cur.last_utc_hour  = utc_hour;
  cur.last_utc_min   = utc_min;

  return nvs_save_flight(cur);
}

bool nvs_factory_reset(){
  if (!prefs.begin(NS, false)) return false;
  prefs.clear();
  prefs.end();
  return true;
}
