#pragma once
#include <Arduino.h>

// Simple GPS-only wind estimator + airspeed estimator.
//  - Ingests GPS ground-speed (knots) + track (deg true).
//  - Estimates wind vector by:
//      (a) "thermal circle" vector-mean of ground velocities when heading dispersion is high,
//      (b) up/downwind extremes over recent straight legs.
//  - Smooths updates with EMA.
//  - Provides AS estimate: |Vg - Vw|.
//
// Coordinates: track 0° = North, 90° = East.
// Vector components are "towards" (direction the wind is blowing to).

class WindEstimator {
public:
  WindEstimator();

  // Update with a fresh GPS sample.
  void update(uint32_t now_ms, float sog_kts, float track_deg);

  // Query wind (returns "from" direction in degrees, and speed in knots).
  bool getWind(float& speed_kts, float& dir_from_deg) const;

  // Return true if we have a reasonably fresh estimate.
  bool windValid(uint32_t now_ms = 0) const;

  // Airspeed estimate (knots) given current GPS SOG/track.
  // Returns NAN if wind not valid or inputs invalid.
  float getAirspeedEstimate(float sog_kts, float track_deg) const;

  // Reset state.
  void reset();

private:
  struct Sample {
    float vx_ms;     // ground velocity components (m/s), East
    float vy_ms;     // North
    float sog_ms;    // speed over ground (m/s)
    float track_deg; // 0=N, 90=E
    uint32_t t_ms;
  };

  // Ring buffer
  static constexpr int   MAX_SAMPLES       = 160;   // ~80 s @ 2 Hz typical
  Sample                 buf_[MAX_SAMPLES];
  int                    head_  = 0;
  int                    count_ = 0;

  // Wind vector (towards) in m/s
  float                  wx_ms_ = NAN;
  float                  wy_ms_ = NAN;
  bool                   have_w_ = false;
  uint32_t               w_last_ms_ = 0;

  // Smoothing
  float                  alpha_ = 0.25f;           // EMA weight

  // Internal helpers
  static float  kts_to_ms(float k){ return k * 0.514444f; }
  static float  ms_to_kts(float v){ return v * 1.943844f; }
  static float  deg_to_rad(float d){ return d * 0.017453292519943295f; }
  static float  wrap360(float d){ while(d<0) d+=360; while(d>=360)d-=360; return d; }
  static float  ang_diff_abs(float a, float b){
    float d = fmodf(a-b,360.0f); if(d<-180) d+=360; if(d>180) d-=360; return fabsf(d);
  }

  void   push_(const Sample& s);
  bool   circleUpdate_(uint32_t now_ms);
  bool   straightLegUpdate_(uint32_t now_ms);
  void   applyCandidate_(float wx_ms, float wy_ms, uint32_t now_ms);
};
