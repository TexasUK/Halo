#include "flarm.h"
#include "../app/telemetry.h"
#include "../app/constants.h"
#include <math.h>
#include <string.h>

static HardwareSerial* fl_port = nullptr;
static int   fl_rx_pin = -1;
static uint32_t fl_baud = 0;

static bool rmc_valid = false;   // RMC 'A' = valid
static uint32_t rmc_ms = 0;
static int  gga_sats = 0;
static uint32_t gga_ms = 0;

// ---------- Hazard classification thresholds (tune to taste) ----------
// Level bands: L3 = closest, L2 = nearer, L1 = awareness
static const float L3_RNG_M   = 450.0f;   // red: very close
static const float L2_RNG_M   = 800.0f;   // amber
static const float L1_RNG_M   = 1500.0f;  // green awareness

// Vertical windows (|Δalt| in feet)
static const float VERT_L3_FT = 200.0f;
static const float VERT_L2_FT = 400.0f;
static const float VERT_L1_FT = 800.0f;

// Clear hysteresis: require range to open by this much beyond last hazardous range
static const float    CLR_HYST_M   = 250.0f;
// Minimum time after last raise before early-clear can occur
static const uint32_t MIN_CLEAR_MS = 1800;

// Aggregation window: coalesce multiple PFLAA in a short window, pick best
static const uint32_t AGG_WINDOW_MS = 150;   // 100–200ms is fine

// Track last hazard raise to support hysteresis / non-sticky behaviour
static uint32_t last_raise_ms = 0;
static float    last_rng_m    = 1e9f;
static int      last_level    = 0;

// Track most recent seen contact range even if non-hazardous (for early-clear decision)
static float    last_seen_rng_m = 1e9f;
static uint32_t last_seen_ms    = 0;

// ---- Aggregator state ----
struct AggBest {
  bool  has = false;
  int   level = 0;     // 1..3
  float rng_m = 1e9f;
  float rn = 0, re = 0, rv = 0;
  float brg_deg = NAN;
};
static AggBest    agg_best;
static uint32_t   agg_window_start = 0;

// --------- Helpers ----------
static inline int clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }

static int hazard_level(float rng_m, float dAlt_ft) {
  auto in = [&](float R, float Z){ return rng_m <= R && fabsf(dAlt_ft) <= Z; };
  if (in(L3_RNG_M, VERT_L3_FT)) return 3;
  if (in(L2_RNG_M, VERT_L2_FT)) return 2;
  if (in(L1_RNG_M, VERT_L1_FT)) return 1;
  return 0;
}

bool navValid(){
  uint32_t now = millis();
  return rmc_valid && (gga_sats >= 4)
      && (now - rmc_ms  < 2500)
      && (now - gga_ms  < 3500);
}

static void handleRMC(const char* s){
  // Fields (0-based after talker+type):
  // 1: hhmmss.sss  2: Status A/V  7: SOG(knots)  8: COG(deg)
  int field=0; const char* p=s; char tok[32]; int ti=0;
  float sog=-1, cog=-1; bool valid=false;

  // Temporary for time
  int utc_hh = -1, utc_mm = -1;
  bool saw_time_field = false;

  while(*p){
    if(*p==','||*p=='*'){
      tok[ti]=0;

      if(field==1){ // time field "hhmmss.sss" (or "hhmmss")
        if (tok[0] && tok[1] && tok[2] && tok[3] && tok[4] && tok[5]) {
          int hh = (tok[0]-'0')*10 + (tok[1]-'0');
          int mm = (tok[2]-'0')*10 + (tok[3]-'0');
          if (hh>=0 && hh<24 && mm>=0 && mm<60) { utc_hh = hh; utc_mm = mm; }
        }
        saw_time_field = true;
      }
      if(field==2) valid = (tok[0]=='A');  // status
      if(field==7) sog   = atof(tok);      // speed(kn)
      if(field==8) cog   = atof(tok);      // course

      field++; ti=0; if(*p=='*') break;
    } else if(ti< (int)sizeof(tok)-1){
      tok[ti++]=*p;
    }
    ++p;
  }

  rmc_valid = valid; rmc_ms = millis();
  if(valid){
    if(sog>=0) tele.sog_kts   = sog;
    if(cog>=0) tele.track_deg = fmodf(cog,360.0f);

    if (saw_time_field) {
      tele.utc_hour = utc_hh;   // may be -1 if malformed
      tele.utc_min  = utc_mm;   // may be -1 if malformed
    }

    tele.last_nmea_ms = millis();
  }
}

static void handleGGA(const char* s){
  int field=0; const char* p=s; char tok[24]; int ti=0; int sats=0;
  while(*p){
    if(*p==','||*p=='*'){ tok[ti]=0;
      if(field==7) sats = atoi(tok);
      field++; ti=0; if(*p=='*') break;
    } else if(ti< (int)sizeof(tok)-1) tok[ti++]=*p;
    ++p;
  }
  gga_sats = sats; gga_ms = millis();
}

static void publish_agg_best(uint32_t now){
  // Push selected best hazard to the global alert and refresh 'since'
  alert.active      = true;
  alert.alarm       = agg_best.level;
  alert.relN_m      = agg_best.rn;
  alert.relE_m      = agg_best.re;
  alert.relV_m      = agg_best.rv;
  alert.dist_m      = agg_best.rng_m;
  alert.bearing_deg = agg_best.brg_deg;
  alert.since       = now;

  last_raise_ms = now;
  last_rng_m    = agg_best.rng_m;
  last_level    = agg_best.level;

  // reset aggregator window
  agg_best = {};
  agg_window_start = 0;
}

static void handlePFLAA(const char* s){
  // PFLAA fields (after "$PFLAA,"):
  // 0: alarm(0..3)  1: relN(m)  2: relE(m)  3: relV(m)  ... (we use first 4)
  int field=0; const char* p=s; char tok[24]; int ti=0;
  int alarm_in=0; float rn=0,re=0,rv=0;

  while(*p){
    if(*p==','||*p=='*'){ tok[ti]=0;
      if(field==0) alarm_in = atoi(tok);
      if(field==1) rn       = atof(tok);
      if(field==2) re       = atof(tok);
      if(field==3) rv       = atof(tok);
      field++; ti=0; if(*p=='*') break;
    } else if(ti< (int)sizeof(tok)-1) tok[ti++]=*p;
    ++p;
  }

  // Derived geometry
  float dist_m = sqrtf(rn*rn + re*re);
  float brgN   = atan2f(re, rn) * 180.0f / 3.1415926f; if(brgN<0) brgN += 360.0f;
  float dAlt_ft = rv * 3.28084f;

  // Always remember what we most recently saw (even if non-hazard)
  last_seen_rng_m = dist_m;
  last_seen_ms    = millis();

  // Classify hazard level from geometry (do not blindly trust incoming alarm)
  int lvl_geom = hazard_level(dist_m, dAlt_ft);
  int lvl_in   = clampi(alarm_in, 0, 3);
  int lvl      = (lvl_geom > lvl_in) ? lvl_geom : lvl_in;

  // Start a new aggregation window if needed
  if (agg_window_start == 0) {
    agg_window_start = last_seen_ms;
    agg_best = {};
  }

  // Only aggregate hazardous contacts (level > 0)
  if (lvl > 0) {
    if (!agg_best.has) {
      agg_best.has   = true;
      agg_best.level = lvl;
      agg_best.rng_m = dist_m;
      agg_best.rn    = rn;
      agg_best.re    = re;
      agg_best.rv    = rv;
      agg_best.brg_deg = brgN;
    } else {
      // Prefer higher level; tie-break by nearest
      bool better = (lvl > agg_best.level) || ((lvl == agg_best.level) && (dist_m < agg_best.rng_m));
      if (better) {
        agg_best.level = lvl;
        agg_best.rng_m = dist_m;
        agg_best.rn    = rn;
        agg_best.re    = re;
        agg_best.rv    = rv;
        agg_best.brg_deg = brgN;
      }
    }
  }
}

static void parse_line(const char* line){
  if(!line || !line[0]) return;
  if(!strncmp(line,"$GPRMC",6) || !strncmp(line,"$GNRMC",6)) handleRMC(line);
  else if(!strncmp(line,"$GPGGA",6)|| !strncmp(line,"$GNGGA",6)) handleGGA(line);
  else if(!strncmp(line,"$PFLAA",6)) handlePFLAA(line);
}

void nav_inject_nmea(const char* s){
  parse_line(s);
}

void nav_begin(HardwareSerial& port, int rxPin, uint32_t baud){
  fl_port  = &port; fl_rx_pin = rxPin; fl_baud = baud;
  fl_port->begin(fl_baud, SERIAL_8N1, fl_rx_pin, -1);

  rmc_valid=false; rmc_ms=0; gga_sats=0; gga_ms=0;

  // initialize UTC to unknown
  tele.utc_hour = -1; tele.utc_min = -1;

  // reset hazard memory & aggregator
  last_raise_ms = 0;
  last_rng_m    = 1e9f;
  last_level    = 0;

  last_seen_rng_m = 1e9f;
  last_seen_ms    = 0;

  agg_best = {};
  agg_window_start = 0;
}

void nav_tick(){
  if(!fl_port) return;
  static char buf[200]; static uint8_t len=0;
  while(fl_port->available()){
    char c = (char)fl_port->read();
    if(c=='\r') continue;
    if(c=='\n'){
      buf[len]=0;
      if(len) parse_line(buf);
      len=0;
    } else if(len < sizeof(buf)-1){
      buf[len++] = c;
    } else {
      len=0; // overflow guard
    }
  }

  // Aggregation publish / housekeeping
  uint32_t now = millis();

  // Publish best hazard once per short window
  if (agg_window_start != 0 && (now - agg_window_start) >= AGG_WINDOW_MS) {
    if (agg_best.has) {
      publish_agg_best(now);
    } else {
      // No hazardous candidates in this window; do nothing (natural expiry)
      agg_best = {};
      agg_window_start = 0;
    }
  }

  // Early-clear: If an alert is active but latest seen traffic is clearly farther away,
  // and enough time has elapsed since the last hazard raise, clear immediately.
  if (alert.active) {
    bool have_recent_seen = (last_seen_ms != 0) && ((now - last_seen_ms) < 1500);
    if (have_recent_seen) {
      bool far_enough  = (last_seen_rng_m > (last_rng_m + CLR_HYST_M));
      bool long_enough = (now - last_raise_ms) > MIN_CLEAR_MS;
      if (far_enough && long_enough) {
        alert.active = false;
      }
    }
  }
}
