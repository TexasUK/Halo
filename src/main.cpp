#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BMP280.h>
#include <pgmspace.h>
#include <ctype.h>

#include "version.h"
#include "splash_image.h"

#include "app/constants.h"
#include "app/telemetry.h"
#include "app/ui_iface.h"
#include "app/app_fsm.h"

#include "drivers/dfplayer.h"
#include "nav/flarm.h"
#include "storage/nvs_store.h"

#include "ble/ble_ctrl.h"   // BLE control plane + app hooks declarations

// ---------------- Pins ----------------
#define I2C_SDA   4
#define I2C_SCL   5
#define TFT_SCLK  1
#define TFT_MOSI  2
#define TFT_MISO -1
#define TFT_CS   10
#define TFT_DC   11
#define TFT_RST  13
#define TFT_BL    3
// DFPlayer Mini (TX-only + BUSY)
#define DF_TX_PIN       9
#define DF_BAUD         9600
#define DF_BUSY_PIN     7    // active LOW while playing
// FLARM / SoftRF RX-only UART
#define FLARM_RX_PIN    8
// Strobe MOSFET gate
#define STROBE_PIN      6

// ---------------- Devices ----------------
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP280  bmp;
HardwareSerial   DFSerial(1);
HardwareSerial   FLARM(2);

// --- DFPlayer raw command helpers (no re-init needed) ---
static void df_send_cmd(uint8_t cmd, uint16_t param) {
  // DFPlayer 10-byte frame: 7E FF 06 <cmd> 00 <hi> <lo> <chkH> <chkL> EF
  uint8_t buf[10];
  buf[0]=0x7E; buf[1]=0xFF; buf[2]=0x06; buf[3]=cmd; buf[4]=0x00;
  buf[5]=(uint8_t)(param >> 8); buf[6]=(uint8_t)(param & 0xFF);
  uint16_t sum = 0;
  for (int i=1; i<=6; ++i) sum += buf[i];
  uint16_t chk = 0xFFFF - sum + 1;  // two's complement
  buf[7]=(uint8_t)(chk >> 8); buf[8]=(uint8_t)(chk & 0xFF); buf[9]=0xEF;
  DFSerial.write(buf, sizeof(buf));
  DFSerial.flush();
}

static inline void df_set_volume_immediate(uint8_t vol0_30) {
  if (vol0_30 > 30) vol0_30 = 30;
  df_send_cmd(0x06, vol0_30);  // DFPlayer 'Set Volume' command
}


// ---------------- Backlight / Colors ----------------
const int BL_CH=0, BL_FREQ=5000, BL_BITS=8;
static inline void setBrightness(uint8_t v){ ledcWrite(BL_CH, v); }
static inline uint16_t COL(uint8_t r,uint8_t g,uint8_t b){ return tft.color565(b,g,r); }

static const uint16_t COL_BG         = COL(0,0,0);
static const uint16_t COL_HEADER_BG  = COL(30,30,60);
static const uint16_t COL_HEADER_FG  = COL(255,255,255);
static const uint16_t COL_ACCENT     = COL(60,60,90);
static const uint16_t COL_FG         = COL(255,255,255);
static const uint16_t COL_BADGE_ON   = COL(0,220,0);
static const uint16_t COL_BADGE_OFF  = COL(140,140,140);
static const uint16_t TINT_L1        = COL(0,140,0);
static const uint16_t TINT_L2        = COL(255,180,0);
static const uint16_t TINT_L3        = COL(220,0,0);

// ---------------- Persisted settings (NVS) ----------------
static HaloSettings g_cfg;
static uint32_t     g_nav_baud = 19200;

// ---------------- App live variables ----------------
Telemetry   tele;
TrafficAlert alert = {};

float   qnh_hPa         = 1013.25f;
float   airfieldElev_ft = 0.0f;
uint8_t df_volume       = 24;

bool  baselineSet   = false;
float baselineAlt_m = NAN;

// Extern for baro math
float seaLevel_hPa = 1013.25f;

// Plays track 2 once when nav becomes valid
static bool navWasValid = false;

// ---------------- Strobe ----------------
struct StrobeCfg { uint16_t on_ms=120, period_ms=2000; } stb;
static uint32_t strobe_zero_t = 0;
bool strobe_enabled = false;
static inline void strobeApply(bool on){ digitalWrite(STROBE_PIN, on ? HIGH : LOW); }
void strobeSet(uint16_t on_ms, uint16_t period_ms){ stb.on_ms=on_ms; stb.period_ms=period_ms; strobe_zero_t=millis(); }
void strobeEnable(bool en){ strobe_enabled=en; if(!en) strobeApply(false); }
static void strobeTickSimple(){
  if(!strobe_enabled){ strobeApply(false); return; }
  uint32_t now=millis();
  uint32_t per = (stb.period_ms==0)?1:stb.period_ms;
  uint32_t phase=(now-strobe_zero_t)%per;
  strobeApply(phase<stb.on_ms);
}

// Nav-valid chime gating
static uint32_t lastNavChime_ms = 0;
static const uint32_t NAV_VALID_CONFIRM_MS   = 1500;  // must be valid this long before chime
static const uint32_t NAV_REARM_INVALID_MS   = 9000;  // must be invalid this long to rearm (>= FLARM_TIMEOUT_MS)
static const uint32_t NAV_CHIME_COOLDOWN_MS  = 30000; // min gap between chimes

enum NavEdgeState { NAV_INV=0, NAV_WAIT_VALID, NAV_VALID };
static NavEdgeState navEdge = NAV_INV;
static uint32_t     navEdge_t = 0;


// ---------------- UI page state ----------------
static bool pageDrawn[5] = {false,false,false,false,false};
static inline void markAllUndrawn_local(){ for(int i=0;i<5;i++) pageDrawn[i]=false; }
Page g_current_page = PAGE_BOOT;

void ui_set_page(Page p){
  g_current_page = p;
  markAllUndrawn_local();
  const char* pname =
    (p==PAGE_BOOT)?"BOOT":(p==PAGE_COMPASS)?"CRUISE":(p==PAGE_TRAFFIC)?"TRAFFIC":(p==PAGE_LANDING)?"LANDING":"LANDED";
  Serial.printf("[UI ] page -> %s\n", pname);
}
void ui_markAllUndrawn(){ markAllUndrawn_local(); }

// ---------------- Timing ----------------
uint32_t bootShownSince_ms = 0;
uint32_t lastSensor=0, lastUI=0;

// ---------------- Splash ----------------
enum SplashState { SPLASH_START, SPLASH_SHOW_IMG, SPLASH_HOLD_IMG, SPLASH_SHOW_VER, SPLASH_HOLD_VER, SPLASH_DONE };
SplashState splash = SPLASH_START; uint32_t splash_t = 0;

// WDT-friendly blit (yield periodically)
static void drawSplashImageProgmem(){
  tft.startWrite();
  tft.setAddrWindow(0,0,160,128);
  for(int16_t y=0; y<128; ++y){
    if ((y & 7) == 0) yield();
    for(int16_t x=0; x<160; ++x){
      uint16_t c = pgm_read_word(&splash_img[y*160 + x]);
      tft.writePixel(x,y,c);
    }
  }
  tft.endWrite();
}
static void drawVersionCard(){
  tft.fillScreen(COL_BG);
  tft.drawRoundRect(6,6,tft.width()-12,tft.height()-12,8, COL(200,200,255));
  tft.setTextColor(COL_FG);
  tft.setTextSize(2); tft.setCursor(12,20); tft.print("HALO");
  tft.setTextSize(1); tft.setCursor(12,38); tft.print(APP_NAME);
  tft.setCursor(12,50); tft.print("v"); tft.print(APP_VERSION);
  tft.setCursor(12,62); tft.print(APP_BUILD_DATE);
  tft.setCursor(12,74); tft.print("git "); tft.print(APP_GIT_HASH);
}
static void splash_tick(){
  static const uint32_t IMG_HOLD_MS = 1200;
  static const uint32_t VER_HOLD_MS = 1200;
  uint32_t now=millis();
  switch(splash){
    case SPLASH_START: setBrightness(220); tft.fillScreen(COL_BG); splash=SPLASH_SHOW_IMG; break;
    case SPLASH_SHOW_IMG: drawSplashImageProgmem(); splash_t=now; splash=SPLASH_HOLD_IMG; break;
    case SPLASH_HOLD_IMG: if(now - splash_t >= IMG_HOLD_MS){ splash=SPLASH_SHOW_VER; } break;
    case SPLASH_SHOW_VER: drawVersionCard(); splash_t=now; splash=SPLASH_HOLD_VER; break;
    case SPLASH_HOLD_VER: if(now - splash_t >= VER_HOLD_MS){ splash=SPLASH_DONE; } break;
    default: break;
  }
}

// --- FLARM badge (top-right of header) ---
static bool boot_last_nav_ok = false;

static void drawFlarmBadge(bool ok){
  const char* label = "FLARM";
  const int  h  = 14;
  const int  w  = 6 * 5 + 8;   // 5 chars * 6px + padding
  const int  x  = tft.width() - w - 4;
  const int  y  = 2;
  const uint16_t bg = ok ? COL(32,168,72) : COL(64,64,72);
  const uint16_t fg = ok ? COL(255,255,255) : COL(200,200,210);

  // background + outline
  tft.fillRoundRect(x, y, w, h, 3, bg);
  tft.drawRoundRect(x, y, w, h, 3, fg);

  // text
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);
  tft.setCursor(x + 3, y + 3);
  tft.print(label);
}

// ---------------- Header & badges ----------------
static void drawHeaderStrip(const __FlashStringHelper* title){
  tft.fillRect(0,0,tft.width(),14,COL_HEADER_BG);
  tft.drawFastHLine(0,14,tft.width(),COL_FG);
  tft.setTextColor(COL_HEADER_FG);
  tft.setTextSize(1);
  tft.setCursor(4,3);
  tft.print(title);
}
static void drawBadgeHeader(int x,int y,int w,int h, const char* txt, bool ok){
  uint16_t col = ok ? COL_BADGE_ON : COL_BADGE_OFF;
  tft.fillRect(x-1, y-1, w+2, h+2, COL_HEADER_BG);
  tft.drawRoundRect(x, y, w, h, 3, col);
  tft.setTextColor(col); tft.setTextSize(1);
  int tw = (int)strlen(txt)*6; tft.setCursor(x + (w - tw)/2, y+2); tft.print(txt);
}
static void drawHeaderBadges(bool flarm_ok){
  int w=tft.width(), h=12, by=1, wF=48, wS=56, gap=4;
  int bxF = w - wF - 2, bxS = bxF - gap - wS;
  drawBadgeHeader(bxS, by, wS, h, "STROBE", strobe_enabled);
  drawBadgeHeader(bxF, by, wF, h, "FLARM", flarm_ok);
}

// ---------------- BOOT ----------------
// --- BOOT (bigger value font, right-aligned) ---

static void drawBootStatic(){
  tft.fillScreen(COL_BG);
  drawHeaderStrip(F("Pre-Flight Values"));
  boot_last_nav_ok = navValid();
  drawFlarmBadge(boot_last_nav_ok);

  const int xLabel = 6;
  const int xValueLeft = 64;          // start of the value column (for cleanup)
  const int y0 = 26;
  const int dy = 26;                   // a bit more spacing for size-2 values

  tft.setTextColor(COL(170,200,255));
  tft.setTextSize(1);
  int y = y0;
  tft.setCursor(xLabel,y); tft.print(F("Temperature:"));        y += dy;
  tft.setCursor(xLabel,y); tft.print(F("QNH:"));                y += dy;
  tft.setCursor(xLabel,y); tft.print(F("Airfield Elev:"));      y += dy;
  tft.setCursor(xLabel,y); tft.print(F("Volume:"));

  // Clear the value column (taller/wider to fit size-2 text)
  tft.fillRect(xValueLeft, y0-4, tft.width()-xValueLeft-6, dy*4+10, COL_BG);
}

static void updBoot(){
  const int y0 = 26;
  const int dy = 26;
  const int marginR = 6;
  const int charW = 12;               // 6px base * size 2
  bool ok = navValid();
if (ok != boot_last_nav_ok){
  drawFlarmBadge(ok);
  boot_last_nav_ok = ok;
}
  auto printRight = [&](const char* s, int y){
    int x = tft.width() - marginR - (int)strlen(s) * charW;
    if (x < 0) x = 0;
    tft.setCursor(x, y);
    tft.print(s);
  };

  tft.setTextColor(COL_FG, COL_BG);
  tft.setTextSize(2);                  // bigger values (labels stay size 1)

  char buf[24];
  int y = y0;

  // Temperature
  if (tele.bmp_ok && !isnan(tele.tC)) {
    snprintf(buf, sizeof(buf), "%dC", (int)lroundf(tele.tC));
  } else {
    snprintf(buf, sizeof(buf), "--C");
  }
  printRight(buf, y); y += dy;

  // QNH (no space so it fits neatly)
  snprintf(buf, sizeof(buf), "%dhPa", (int)lroundf(qnh_hPa));
  printRight(buf, y); y += dy;

  // Airfield elevation
  snprintf(buf, sizeof(buf), "%dft", (int)lroundf(airfieldElev_ft));
  printRight(buf, y); y += dy;

  // Volume
  snprintf(buf, sizeof(buf), "%u", (unsigned)df_volume);
  printRight(buf, y);
}


// ---------------- Compass helpers ----------------
static int norm360(int a){ a%=360; if(a<0) a+=360; return a; }
static const char* label45(int d){
  d=((d%360)+360)%360;
  switch(d){ case 0:return "N"; case 45:return "NE"; case 90:return "E"; case 135:return "SE"; case 180:return "S"; case 225:return "SW"; case 270:return "W"; case 315:return "NW"; default:return nullptr; }
}

// ---------------- Cruise ----------------
static void drawCompassTape(float heading_deg){
  const int yTop=34, h=26;
  const float pxPerDeg=1.25f;
  int w=tft.width(), cx=w/2;

  tft.fillRect(0,yTop,w,h,COL_BG);
  tft.drawFastHLine(0,yTop,w,COL_ACCENT);
  tft.drawFastHLine(0,yTop+h-1,w,COL_ACCENT);

  int minDeg=(int)floor(heading_deg - (w/2)/pxPerDeg);
  int maxDeg=(int)ceil(heading_deg + (w/2)/pxPerDeg);

  for(int deg=minDeg-(minDeg%10); deg<=maxDeg; deg+=10){
    float dx=(deg-heading_deg)*pxPerDeg;
    int x=(int)roundf(cx+dx);
    if(x<0||x>=w) continue;

    int ddeg=norm360(deg);
    bool is30=(ddeg%30==0);
    int tickH=is30?(h/2):(h/4);
    tft.drawFastVLine(x, yTop+(h-tickH), tickH, COL_FG);

    if(ddeg%45==0){
      const char* lab=label45(ddeg);
      if(lab){
        tft.setTextSize(1);
        tft.setTextColor(COL_FG, COL_BG);
        int tw=(int)strlen(lab)*6;
        tft.setCursor(x - tw/2, yTop+2);
        tft.print(lab);
      }
    }
  }

  // Chevron and numeric heading
  int tipX=cx, tipY=yTop+h;
  tft.fillTriangle(tipX,tipY, tipX-6,tipY+8, tipX+6,tipY+8, COL_FG);

  int hdgInt=norm360((int)lroundf(heading_deg));
  char hbuf[8];
  snprintf(hbuf,sizeof(hbuf),"%03d",hdgInt);

  tft.setTextSize(2);
  tft.setTextColor(COL_FG, COL_BG);
  tft.setCursor(tipX-18, tipY+10);
  tft.print(hbuf);

  // Degree mark (tiny dot)
  int16_t x = tft.getCursorX();
  int16_t y = tft.getCursorY();
  tft.fillCircle(x + 2, y - 6, 2, COL_FG);
}
static void drawCruiseStatic(){
  tft.fillScreen(COL_BG);
  drawHeaderStrip(F("Cruise"));
  drawHeaderBadges(navValid());
  float hdg = (isnan(tele.track_deg)? 0.f : tele.track_deg); drawCompassTape(hdg);
}
static void updCruise(){
  drawHeaderBadges(navValid());
  float hdg=(isnan(tele.track_deg)?0.f:tele.track_deg); drawCompassTape(hdg);
  const int yText=tft.height()-22; tft.setTextColor(COL_FG, COL_BG); tft.setTextSize(2);
  tft.setCursor(6,yText);
  if(!isnan(tele.sog_kts)){ int skts=(int)lroundf(tele.sog_kts); tft.print(skts); tft.print("kts"); } else tft.print("---kts");
  char buf[16]; if(!isnan(tele.alt_m)){ int ft=(int)lroundf(tele.alt_m*3.28084f); snprintf(buf,sizeof(buf),"%dft",ft); } else snprintf(buf,sizeof(buf),"---ft");
  int tw=strlen(buf)*12; tft.setCursor(tft.width()-6-tw,yText); tft.print(buf);
}

// ---------------- Traffic helpers + renderer ----------------
static void drawGliderGlyph(int cx, int cy, uint16_t col) {
  tft.drawFastVLine(cx, cy-6, 12, col);
  tft.drawFastHLine(cx-10, cy, 20, col);
  tft.drawLine(cx, cy+5, cx+4, cy+8, col);
  tft.drawLine(cx, cy+5, cx-4, cy+8, col);
}
static void drawArrowOnRing(int cx, int cy, int R, float bearing_deg, uint16_t col) {
  const int   Rbase = R + 8;
  const float rad   = bearing_deg * 3.1415926f / 180.0f;

  const float rx = cx + (Rbase) * sinf(rad);
  const float ry = cy - (Rbase) * cosf(rad);

  float vx = (float)cx - rx, vy = (float)cy - ry;
  const float vlen = sqrtf(vx*vx + vy*vy);
  if (vlen < 1e-3f) return;
  vx /= vlen; vy /= vlen;

  const float px = -vy, py = vx;

  const float tipLen = 22.0f;
  const float baseW  = 16.0f;
  const float baseIn = 5.0f;

  const float tx  = rx + vx * tipLen;
  const float ty  = ry + vy * tipLen;
  const float blx = rx + px*(baseW*0.5f) - vx*baseIn;
  const float bly = ry + py*(baseW*0.5f) - vy*baseIn;
  const float brx = rx - px*(baseW*0.5f) - vx*baseIn;
  const float bry = ry - py*(baseW*0.5f) - vy*baseIn;

  tft.fillTriangle((int)tx,(int)ty,(int)blx,(int)bly,(int)brx,(int)bry,col);
}
static void drawVertIndicatorRight(int x, int y, bool above, bool below, uint16_t col){
  if (above){
    tft.fillTriangle(x, y-16, x-9, y+4, x+9, y+4, col);
  } else if (below){
    tft.fillTriangle(x, y+16, x-9, y-4, x+9, y-4, col);
  } else {
    tft.fillCircle(x, y, 5, col);
  }
}
struct TrafficDrawCache { bool alive; int alarm; float bearing_deg; float dist_m; float relV_m; uint32_t since; };
static TrafficDrawCache trafLast = {false, 0, NAN, NAN, NAN, 0};

static void drawTrafficStatic(){
  tft.fillScreen(COL_BG);
  drawHeaderStrip(F("Traffic"));
  tft.drawFastHLine(0,16,tft.width(),COL_ACCENT);
}
static void renderTrafficDynamic(bool force){
  const int cx = tft.width()/2 - 12;
  const int cy = 84;
  const int R  = 38;

  const bool alive = alert.active && (millis() - alert.since) < ALERT_HOLD_MS;

  bool changed = force ||
                 (alive != trafLast.alive) ||
                 (alive && (alert.alarm != trafLast.alarm ||
                            alert.since != trafLast.since ||
                            alert.bearing_deg != trafLast.bearing_deg ||
                            alert.dist_m != trafLast.dist_m ||
                            alert.relV_m != trafLast.relV_m));
  if(!changed) return;

  tft.fillRect(0,17, tft.width(), tft.height()-17, COL_BG);
  drawHeaderBadges(navValid());

  tft.setTextSize(1);
  tft.setTextColor(COL(220,220,220), COL_BG);

  if (!alive) {
    tft.setCursor(6,18); tft.print(F("No recent targets"));
  } else {
    float dist_km = alert.dist_m / 1000.0f;
    char  distbuf[16]; dtostrf(dist_km, 0, 1, distbuf);
    tft.setCursor(6,18); tft.print(distbuf); tft.print(" km");

    char bbuf[12];
    int brg = (int)(alert.bearing_deg + 0.5f);
    snprintf(bbuf, sizeof(bbuf), "%d", brg);
    int tw_brg = (int)strlen(bbuf) * 6;
    tft.setCursor(tft.width()-6 - tw_brg - 6, 18);
    tft.print(bbuf);
    int16_t bx = tft.getCursorX();
    int16_t by = tft.getCursorY();
    tft.fillCircle(bx + 2, by - 6, 2, COL_FG);

    char vbuf[18];
    int dAlt_ft = (int)lroundf(alert.relV_m * 3.28084f);
    snprintf(vbuf, sizeof(vbuf), "dAlt %d ft", dAlt_ft);
    int tw_v = (int)strlen(vbuf) * 6;
    int x_v  = max(6, (tft.width() - tw_v) / 2);
    tft.setCursor(x_v, 18);
    tft.print(vbuf);
  }

  uint16_t tint = COL_BG;
  if (alive) {
    if      (alert.alarm >= 3) tint = TINT_L3;
    else if (alert.alarm == 2) tint = TINT_L2;
    else                       tint = TINT_L1;
  }
  const uint16_t fg = COL_FG;
  for (int i=0;i<2;i++) tft.drawCircle(cx, cy, R-i, fg);
  if (alive){
    tft.fillCircle(cx, cy, R-3, tint);
    for (int i=0;i<2;i++) tft.drawCircle(cx, cy, R-i, fg);
  }
  drawGliderGlyph(cx, cy, fg);

  if (alive){
    drawArrowOnRing(cx, cy, R, alert.bearing_deg, fg);
    const float maxRange = 1500.0f;
    const float clamp    = (alert.dist_m > maxRange) ? (maxRange/alert.dist_m) : 1.0f;
    const float r_pix    = (alert.dist_m * clamp) * ((R-6)/maxRange);
    const float ang      = alert.bearing_deg * 3.1415926f / 180.0f;
    const int   tx       = cx + (int)roundf(r_pix * sinf(ang));
    const int   ty       = cy - (int)roundf(r_pix * cosf(ang));
    tft.fillCircle(tx, ty, 3, fg);

    const float dAlt_ft = alert.relV_m * 3.28084f;
    const bool above = dAlt_ft >  200.0f;
    const bool below = dAlt_ft < -200.0f;
    drawVertIndicatorRight(cx + R + 22, cy, above, below, fg);
  }

  trafLast.alive       = alive;
  trafLast.alarm       = alert.alarm;
  trafLast.bearing_deg = alert.bearing_deg;
  trafLast.dist_m      = alert.dist_m;
  trafLast.relV_m      = alert.relV_m;
  trafLast.since       = alert.since;
}

// ---------------- Landing / Landed ----------------
static void drawLandingStatic(){
  tft.fillScreen(COL_BG);
  drawHeaderStrip(F("Landing"));
  tft.setTextColor(COL(170,200,255));
  tft.setTextSize(1);
  tft.setCursor(6,28); tft.print(F("Speed"));
  tft.setCursor(6,64); tft.print(F("Altitude (ft)"));
}
static void updLanding(){
  tft.setTextColor(COL_FG, COL_BG);
  tft.setTextSize(3);
  int xv = 6;
  tft.setCursor(xv,38);
  if(!isnan(tele.sog_kts)){ int kts=(int)lroundf(tele.sog_kts); tft.print(kts); tft.print("kts"); } else tft.print("---kts");
  tft.setCursor(xv,74);
  if(!isnan(tele.alt_m)){
    int alt_ft = (int)lroundf(tele.alt_m * 3.28084f);
    tft.print(alt_ft); tft.print("ft");
  } else {
    tft.print("---ft");
  }
}
static void drawLandedStatic(){
  tft.fillScreen(COL_BG);
  drawHeaderStrip(F("Landed"));
  tft.setTextColor(COL(170,200,255));
  tft.setTextSize(1);
  tft.setCursor(6,24);  tft.print(F("Duration"));
  tft.setCursor(6,64);  tft.print(F("UTC Time"));
  tft.setCursor(6,104); tft.print(F("Alerts"));
}
static void updLanded(){
  tft.setTextColor(COL_FG, COL_BG);
  tft.setTextSize(3);
  uint32_t ms = app_last_flight_duration_ms();
  uint32_t sec = ms / 1000u;
  uint32_t hh = sec / 3600u;
  uint32_t mm = (sec % 3600u) / 60u;
  char dur[8]; snprintf(dur, sizeof(dur), "%lu:%02lu", (unsigned long)hh, (unsigned long)mm);
  tft.setCursor(6,34); tft.print(dur);

  int uh = tele.utc_hour, um = tele.utc_min;
  char utcbuf[8];
  if (uh>=0 && um>=0) snprintf(utcbuf, sizeof(utcbuf), "%02d:%02d", uh, um);
  else                snprintf(utcbuf, sizeof(utcbuf), "--:--");
  tft.setCursor(6,74); tft.print(utcbuf);

  tft.setTextSize(2);
  uint16_t nAlerts = app_last_flight_alerts();
  tft.setCursor(6,110); tft.print(nAlerts);
}

// ---------------- Page router ----------------
static void drawPage(){
  switch(g_current_page){
    case PAGE_BOOT:
      if(!pageDrawn[PAGE_BOOT]){ drawBootStatic(); pageDrawn[PAGE_BOOT]=true; }
      updBoot(); break;
    case PAGE_COMPASS:
      if(!pageDrawn[PAGE_COMPASS]){ drawCruiseStatic(); pageDrawn[PAGE_COMPASS]=true; }
      updCruise(); break;
    case PAGE_TRAFFIC:
      if(!pageDrawn[PAGE_TRAFFIC]){ drawTrafficStatic(); pageDrawn[PAGE_TRAFFIC]=true; }
      renderTrafficDynamic(false); break;
    case PAGE_LANDING:
      if(!pageDrawn[PAGE_LANDING]){ drawLandingStatic(); pageDrawn[PAGE_LANDING]=true; }
      updLanding(); break;
    case PAGE_LANDED:
      if(!pageDrawn[PAGE_LANDED]){ drawLandedStatic(); pageDrawn[PAGE_LANDED]=true; }
      updLanded(); break;
  }
}

// ---------------- Telemetry defaults ----------------
static void tele_init_defaults(){
  tele.tC = NAN; tele.p_hPa = NAN; tele.alt_m = NAN;
  tele.bmp_ok = false;
  tele.sog_kts = NAN; tele.track_deg = NAN;
  tele.last_nmea_ms = 0;
  tele.vs_ms = 0;
  tele.utc_hour = -1; tele.utc_min = -1;
}

// ---------------- Sensors ----------------
static void updateBMP(){
  if(!tele.bmp_ok) return;
  float seaLevel = qnh_hPa;

  float tC=bmp.readTemperature();
  float pPa=bmp.readPressure();
  if(!isnan(tC) && !isnan(pPa)){
    tele.tC=tC; tele.p_hPa=pPa/100.0f; tele.alt_m=bmp.readAltitude(seaLevel);
    if(!baselineSet && !isnan(tele.alt_m)){
      baselineAlt_m=tele.alt_m; baselineSet=true;
      Serial.print(F("[BMP] baseline alt m=")); Serial.println(baselineAlt_m,1);
    }
  }
  uint32_t now=millis();
  static float lastAlt = NAN; static uint32_t lastAltT=0;
  if(!isnan(tele.alt_m)){
    if(!isnan(lastAlt)){ float dt=(now-lastAltT)/1000.0f; if(dt>0.001f) tele.vs_ms=(tele.alt_m-lastAlt)/dt; }
    lastAlt=tele.alt_m; lastAltT=now;
  }
}

// ---------------- App hooks for BLE persistence/hot-switch ----------------
void halo_set_volume_runtime_and_persist(uint8_t vol0_30){
  df_volume = constrain(vol0_30, 0, 30);
  g_cfg.volume0_30 = df_volume;
  nvs_save_settings(g_cfg);

  // Light-weight: set volume directly (no dfp_begin reboot).
  df_set_volume_immediate(df_volume);
  Serial.printf("[AUDIO] volume now %u (persisted)\n", (unsigned)df_volume);
}
void halo_set_qnh_runtime_and_persist(uint16_t hpa){
  qnh_hPa = (float)hpa;
  g_cfg.qnh_hPa = qnh_hPa;
  nvs_save_settings(g_cfg);

  // Recompute altitude at the new QNH
  updateBMP();

  // If not airborne, anchor baseline so AGL ≈ 0 (prevents FSM misfires)
  extern AppState g_state;
  bool onGround = (g_state != ST_FLYING && g_state != ST_ALERT && g_state != ST_LANDING);
  if (onGround && !isnan(tele.alt_m)) {
    baselineAlt_m = tele.alt_m;
    baselineSet   = true;
    g_cfg.baselineAlt_m = baselineAlt_m;
    g_cfg.baselineSet   = true;
    nvs_save_settings(g_cfg);
    Serial.printf("[QNH] baseline anchored to %.2fm (AGL stabilized)\n", baselineAlt_m);
  }

  // UI will pick up qnh_hPa on the next BOOT tick
}
void halo_set_elev_runtime_and_persist(uint16_t feet){
  airfieldElev_ft = (float)feet;
  g_cfg.airfieldElev_ft = airfieldElev_ft;
  nvs_save_settings(g_cfg);
}
void halo_set_datasource_and_baud(bool isSoftRF, uint8_t baudIndex){
  g_cfg.data_source = isSoftRF ? HALO_SRC_SOFTRF : HALO_SRC_FLARM;
  nvs_save_settings(g_cfg);

  uint32_t baud = (baudIndex==0) ? 19200u : 38400u;
  halo_apply_nav_baud(baud);   // re-open UART2

  // Drain any stale bytes and force nav age to "unknown"
  delay(20);
  while (FLARM.available()) (void)FLARM.read();
  tele.last_nmea_ms = 0;

  // Force header redraw so the badge reflects navValid() as soon as frames arrive
  ui_markAllUndrawn();

  Serial.printf("[NAV] source=%s, baud=%lu (flushed; awaiting fresh NMEA)\n",
                isSoftRF ? "SoftRF" : "FLARM", (unsigned long)baud);
}


void halo_apply_nav_baud(uint32_t baud){
  g_nav_baud = baud;
  nav_begin(FLARM, FLARM_RX_PIN, g_nav_baud); // re-open Serial2 at new baud
  Serial.printf("[NAV] UART reinit @ %lu\n", (unsigned long)g_nav_baud);
}

// ---------------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200);
  uint32_t t0=millis(); while(!Serial && millis()-t0<1000){ delay(10); }

  // Display / backlight
  ledcSetup(BL_CH, BL_FREQ, BL_BITS); ledcAttachPin(TFT_BL, BL_CH); setBrightness(255);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.initR(INITR_GREENTAB); tft.setRotation(1); tft.setSPISpeed(12000000);
  tft.fillScreen(COL_BG); tft.setTextColor(COL_FG); tft.setTextSize(1);

  // Telemetry defaults & splash
  tele_init_defaults();
  splash = SPLASH_START; splash_t=millis();

  // I2C / BMP280
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  bool found=false; uint8_t addr=0x76;
  for(uint8_t a:{(uint8_t)0x76,(uint8_t)0x77}){ Wire.beginTransmission(a); if(Wire.endTransmission()==0){ addr=a; found=true; break; } }
  if(found && bmp.begin(addr)){
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X8,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
    tele.bmp_ok=true;
  }

  // Strobe GPIO
  pinMode(STROBE_PIN, OUTPUT); strobeApply(false); strobeSet(120,2000); strobeEnable(false);

  // NVS
  nvs_init();
  nvs_load_settings(g_cfg);

  // Apply to runtime
  qnh_hPa         = g_cfg.qnh_hPa;
  airfieldElev_ft = g_cfg.airfieldElev_ft;
  df_volume       = g_cfg.volume0_30;
  baselineSet     = g_cfg.baselineSet;
  baselineAlt_m   = g_cfg.baselineAlt_m;

  // Choose nav baud from data source (0=FLARM, 1=SoftRF)
  g_nav_baud = (g_cfg.data_source != 0) ? 38400u : 19200u;
}

void loop(){
  uint32_t now=millis();

  // SPLASH / VERSION
  if(splash!=SPLASH_DONE){
    splash_tick();
    if(splash==SPLASH_DONE){
      // Bring systems up
      dfp_begin(DFSerial, DF_TX_PIN, DF_BUSY_PIN, DF_BAUD, df_volume);
      nav_begin(FLARM, FLARM_RX_PIN, g_nav_baud);
      app_fsm_init();
      ui_set_page(PAGE_BOOT);
      bootShownSince_ms = millis();
      Serial.println("[BOOT] init complete");

      // Give the DFPlayer a beat to wake, then push saved volume explicitly.
      delay(200);
      df_set_volume_immediate(df_volume);
      delay(40);

      dfp_stop_and_flush();
      dfp_play_filename(1);        // boot chime (now at saved volume)
      trafLast = {false,0,NAN,NAN,NAN,0};
      bleInit();                   // BLE after init
    }
    strobeTickSimple();
    dfp_tick();
    return;
  }

  // cadence
  if(now-lastSensor>=250){ updateBMP(); lastSensor=now; }
  nav_tick();

// Debounced nav-valid chime (track 2)
bool nv = navValid();
switch (navEdge) {
  case NAV_INV:
    if (nv) { navEdge = NAV_WAIT_VALID; navEdge_t = now; }
    break;

  case NAV_WAIT_VALID:
    if (!nv) {
      navEdge = NAV_INV; // lost it before confirmation window
    } else if (now - navEdge_t >= NAV_VALID_CONFIRM_MS) {
      // Confirmed valid long enough; chime if cooldown passed
      if (now - lastNavChime_ms >= NAV_CHIME_COOLDOWN_MS) {
        Serial.println("[AUDIO] navValid (debounced) -> track 2");
        // Optional: skip if audio is already playing
        // if (digitalRead(DF_BUSY_PIN) == HIGH)  // HIGH = idle on your wiring
        dfp_play_filename(2);
        lastNavChime_ms = now;
      }
      navEdge = NAV_VALID;
      navEdge_t = 0;
    }
    break;

  case NAV_VALID:
    if (!nv) {
      if (navEdge_t == 0) navEdge_t = now;
      // Only re-arm if we’ve been truly invalid for the full window
      if (now - navEdge_t >= NAV_REARM_INVALID_MS) {
        navEdge = NAV_INV; navEdge_t = 0;
      }
    } else {
      navEdge_t = 0; // still valid; keep latch
    }
    break;
}


app_fsm_tick(now);


  // change-only rendering
  if(now-lastUI>=160){
    switch(g_current_page){
      case PAGE_TRAFFIC: renderTrafficDynamic(false); break;
      default: drawPage(); break;
    }
    lastUI=now;
  }

  // dfplayer/strobe/ble
  dfp_tick();
  strobeTickSimple();
  bleTick(now);

  // ---- Console test keys (drain; C = hard reset to boot) ----
  while (Serial.available() > 0) {
    int raw = Serial.read();
    if (raw < 0) break;
    if (raw == '\r' || raw == '\n') continue;

    char printable = (raw >= 32 && raw < 127) ? (char)raw : '.';
    Serial.printf("[KEYDBG] rx=0x%02X '%c'\n", (unsigned)raw, printable);

    int ch = toupper((unsigned char)raw);
    switch (ch) {
      case 'J':
        Serial.println("[KEY] J -> play 3");
        dfp_stop_and_flush();
        dfp_play_filename(3);
        break;

      case 'T': {
        Serial.println("[KEY] T -> DEMO: force FLYING");
        tele.sog_kts   = 25.0f;
        tele.track_deg = 0.0f;
        app_demo_force_flying();    // FSM path so alerts change strobe cadence
      } break;

      case 'L': {
        Serial.println("[KEY] L -> DEMO: landing");
        if (!baselineSet && !isnan(tele.alt_m)) { baselineAlt_m = tele.alt_m; baselineSet=true; }
        tele.sog_kts = 0.0f;
        app_demo_force_landing();
      } break;

      case 'R': {
        if (!isnan(tele.alt_m)) {
          baselineAlt_m = tele.alt_m; baselineSet = true;
          g_cfg.baselineSet = baselineSet;
          g_cfg.baselineAlt_m = baselineAlt_m;
          nvs_save_settings(g_cfg);
          Serial.printf("[KEY] R -> baselineAlt_m=%.1f m (saved)\n", baselineAlt_m);
        } else Serial.println("[KEY] R -> cannot set baseline (alt_m is NaN)");
      } break;

      case '1':
      case '2':
      case '3': {
        int lvl = ch - '0';
        Serial.printf("[KEY] %c -> DEMO alert L%d\n", ch, lvl);

        alert.active = true;
        alert.since  = millis();
        alert.alarm  = lvl;

        // 60° absolute bearing -> sector 2 (2 o’clock)
        alert.relN_m = 500;
        alert.relE_m = 866;
        alert.relV_m = (lvl==1 ? 0 : (lvl==2 ? +70 : -70));
        alert.dist_m = sqrtf(alert.relN_m*alert.relN_m + alert.relE_m*alert.relE_m);
        float brgN   = atan2f(alert.relE_m, alert.relN_m) * 180.0f / 3.1415926f;
        if (brgN < 0) brgN += 360.0f;
        alert.bearing_deg = brgN;

        ui_set_page(PAGE_TRAFFIC);
        renderTrafficDynamic(true);

        uint16_t vtrk;
        float dAlt_ft = alert.relV_m * 3.28084f;
        if (dAlt_ft >  200.0f) vtrk = 11; else if (dAlt_ft < -200.0f) vtrk = 12; else vtrk = 10;
        dfp_stop_and_flush();
        dfp_play_filename(vtrk);
        delay(140);
        dfp_play_filename(22);
      } break;

      case 'C': {
        Serial.println("[KEY] C -> HARD RESET to BOOT");
        bleCancelTests();
        dfp_stop_and_flush();
        alert = {};
        strobeEnable(false);
        app_fsm_init();
        ui_markAllUndrawn();
        ui_set_page(PAGE_BOOT);
        navWasValid = false;   // <-- so we’ll chime again when nav becomes valid next time
      } break;


      default:
        Serial.printf("[KEY] unhandled: 0x%02X\n", (unsigned)raw);
        break;
    }
  }
}
