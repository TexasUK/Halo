#include "dfplayer.h"

// Minimal non-blocking DFPlayer protocol (clone-friendly)
static HardwareSerial* dfp = nullptr;
static int  df_tx = -1, df_busy = -1;
static uint8_t df_vol = 20;
static uint32_t df_playCmdAt = 0;

static bool busyNow=false, busyPrev=false;
static uint32_t busyEdgeT=0;

static uint16_t df_index = 1; // last index sent
static uint32_t df_t=0;

enum : uint8_t {
  M_OFF=0, M_SETTLE, M_STOP1, M_STOP2, M_DEVICE, M_VOL, M_MONITOR,
  M_PLAY_STOP, M_PLAY_SEND, M_WAIT_START
};
static uint8_t  m_state = M_OFF;

static const int QSIZE=8;
static uint16_t qbuf[QSIZE];
static uint8_t qh=0, qt=0;
static bool qfull=false;

static inline bool qEmpty(){ return (qh==qt) && !qfull; }
static inline void qClear(){ qh=qt=0; qfull=false; }
static inline bool qEnq(uint16_t n){ if(qfull) return false; qbuf[qh]=n; qh=(qh+1)%QSIZE; if(qh==qt) qfull=true; return true; }
static inline bool qDeq(uint16_t &n){ if(qEmpty()) return false; n=qbuf[qt]; qt=(qt+1)%QSIZE; qfull=false; return true; }

static void df_send(uint8_t cmd, uint16_t param){
  if(!dfp) return;
  uint8_t b[10]; b[0]=0x7E; b[1]=0xFF; b[2]=0x06; b[3]=cmd; b[4]=0x00; b[5]=(param>>8)&0xFF; b[6]=param&0xFF;
  uint16_t sum=0xFF+0x06+cmd+0x00+b[5]+b[6]; uint16_t cs=0xFFFF-sum+1; b[7]=(cs>>8)&0xFF; b[8]=cs&0xFF; b[9]=0xEF;
  dfp->write(b,10);
}

static inline void df_play_current(){ df_send(0x12, df_index); df_playCmdAt=millis(); }

void dfp_play_filename(uint16_t n){
  if(n<1) n=1; if(n>3000) n=3000;
  qEnq(n);
}

void dfp_begin(HardwareSerial& serial, int txPin, int busyPin, uint32_t baud, uint8_t volume0_30){
  dfp=&serial; df_tx=txPin; df_busy=busyPin; df_vol = volume0_30;
  dfp->begin(baud, SERIAL_8N1, -1, df_tx);
  pinMode(df_busy, INPUT_PULLUP);
  busyPrev=busyNow=(digitalRead(df_busy)==LOW);
  m_state = M_SETTLE; df_t=millis();
}

void dfp_tick(){
  if(!dfp || m_state==M_OFF) return;
  uint32_t now=millis();
  // debounced busy
  bool raw=(digitalRead(df_busy)==LOW);
  if(raw!=busyNow && (now-busyEdgeT)>6){ busyPrev=busyNow; busyNow=raw; busyEdgeT=now; }

  switch(m_state){
    case M_SETTLE: if(now-df_t>=800){ m_state=M_STOP1; } break;

    case M_STOP1:
      df_send(0x16,0);            // STOP
      df_t=now; m_state=M_STOP2;
      break;

    case M_STOP2:
      if(now-df_t>=200){
        df_send(0x09,0x0002);     // DEVICE=TF
        df_t=now; m_state=M_DEVICE;
      }
      break;

    case M_DEVICE:
      if(now-df_t>=600){
        df_send(0x06, df_vol);    // VOLUME
        df_t=now; m_state=M_VOL;
      }
      break;

    case M_VOL:
      if(now-df_t>=220){
        m_state=M_MONITOR;
        // intentionally do not auto-play a startup track
      }
      break;

    case M_MONITOR:
      if(!busyNow && !qEmpty() && (now-df_t>=60)){
        uint16_t n; qDeq(n); df_index=n;
        df_send(0x16,0);          // polite STOP before new play
        df_t=now; m_state=M_PLAY_STOP;
      }
      break;

    case M_PLAY_STOP:
      if(now-df_t>=120){
        df_play_current();
        m_state=M_WAIT_START;
      }
      break;

    case M_WAIT_START:
      if(busyNow){
        m_state=M_MONITOR;        // started, allow queueing of next after it ends
      }else if(now-df_playCmdAt>=1500){
        // timeout safety; consider it done
        m_state=M_MONITOR;
      }
      break;

    default: break;
  }
}

// ---- Public control helpers ----

void dfp_stop_and_flush(){
  // hard reset: stop playback and clear queued items
  qClear();
  df_send(0x16,0);          // STOP
  df_t = millis();
  m_state = M_MONITOR;
}

void dfp_clear_queue(){
  // compatibility: only clear pending queue
  qClear();
}

void dfp_stop(){
  // compatibility: only send STOP (do not clear queue)
  df_send(0x16,0);
  df_t = millis();
  m_state = M_MONITOR;
}
