#include "flarm.h"
#include "../app/telemetry.h"
#include "../app/constants.h"

static HardwareSerial* fl_port = nullptr;
static int   fl_rx_pin = -1;
static uint32_t fl_baud = 0;

static bool rmc_valid = false;   // RMC 'A' = valid
static uint32_t rmc_ms = 0;
static int  gga_sats = 0;
static uint32_t gga_ms = 0;

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
        // only parse if we see at least 6 chars
        if (tok[0] && tok[1] && tok[2] && tok[3] && tok[4] && tok[5]) {
          int hh = (tok[0]-'0')*10 + (tok[1]-'0');
          int mm = (tok[2]-'0')*10 + (tok[3]-'0');
          // seconds not needed now
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

    // Update UTC only if we actually saw the time field in this sentence
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

static void handlePFLAA(const char* s){
  int field=0; const char* p=s; char tok[24]; int ti=0; int alarm=0; float rn=0,re=0,rv=0;
  while(*p){
    if(*p==','||*p=='*'){ tok[ti]=0;
      if(field==0) alarm = atoi(tok);
      if(field==1) rn    = atof(tok);
      if(field==2) re    = atof(tok);
      if(field==3) rv    = atof(tok);
      field++; ti=0; if(*p=='*') break;
    } else if(ti< (int)sizeof(tok)-1) tok[ti++]=*p;
    ++p;
  }
  float dist = sqrtf(rn*rn + re*re);
  float brgN = atan2f(re, rn) * 180.0f / 3.1415926f; if(brgN<0) brgN += 360.0f;

  alert.active = true; alert.since = millis();
  alert.relN_m = rn; alert.relE_m = re; alert.relV_m = rv;
  alert.dist_m = dist; alert.bearing_deg = brgN; alert.alarm = alarm;
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
}
