#include "wind_estimator.h"
#include <math.h>

WindEstimator::WindEstimator(){ reset(); }

void WindEstimator::reset(){
  head_ = 0; count_ = 0;
  wx_ms_ = wy_ms_ = NAN;
  have_w_ = false;
  w_last_ms_ = 0;
}

void WindEstimator::push_(const Sample& s){
  buf_[head_] = s;
  head_ = (head_ + 1) % MAX_SAMPLES;
  if (count_ < MAX_SAMPLES) count_++;
}

bool WindEstimator::windValid(uint32_t now_ms) const{
  if (!have_w_) return false;
  if (now_ms == 0) return true;
  // Consider fresh if updated in last 3 minutes
  return (now_ms - w_last_ms_) < 180000UL;
}

bool WindEstimator::getWind(float& speed_kts, float& dir_from_deg) const{
  if(!have_w_ || isnan(wx_ms_) || isnan(wy_ms_)){ speed_kts = NAN; dir_from_deg = NAN; return false; }
  float spd_ms = sqrtf(wx_ms_*wx_ms_ + wy_ms_*wy_ms_);
  // Towards-direction angle (track-like): 0=N, 90=E
  float theta_towards = atan2f(wx_ms_, wy_ms_) * 57.2957795f; // atan2(E, N)
  float theta_from    = wrap360(theta_towards + 180.0f);
  speed_kts           = ms_to_kts(spd_ms);
  dir_from_deg        = theta_from;
  return true;
}

float WindEstimator::getAirspeedEstimate(float sog_kts, float track_deg) const{
  if(!have_w_ || isnan(sog_kts) || isnan(track_deg)) return NAN;
  float spd_ms = kts_to_ms(sog_kts);
  float th = deg_to_rad(track_deg);
  float vgx = spd_ms * sinf(th); // East
  float vgy = spd_ms * cosf(th); // North
  float vax = vgx - wx_ms_;
  float vay = vgy - wy_ms_;
  float va_ms = sqrtf(vax*vax + vay*vay);
  return ms_to_kts(va_ms);
}

void WindEstimator::applyCandidate_(float wx_ms, float wy_ms, uint32_t now_ms){
  // Sanity cap wind speed (e.g., 0..40 m/s)
  float m = sqrtf(wx_ms*wx_ms + wy_ms*wy_ms);
  if (!(m >= 0.0f) || m > 40.0f) return;

  if (!have_w_ || isnan(wx_ms_) || isnan(wy_ms_)) {
    wx_ms_ = wx_ms; wy_ms_ = wy_ms; have_w_ = true;
  } else {
    wx_ms_ = (1.0f - alpha_) * wx_ms_ + alpha_ * wx_ms;
    wy_ms_ = (1.0f - alpha_) * wy_ms_ + alpha_ * wy_ms;
  }
  w_last_ms_ = now_ms;
}

bool WindEstimator::circleUpdate_(uint32_t now_ms){
  // Use last ~15 s window
  const uint32_t WIN_MS = 15000;
  const uint32_t MIN_MS = 6000;
  // Circular statistics of heading: Rbar small => headings well spread (likely circling)
  float sum_cos = 0, sum_sin = 0;
  float sum_vx = 0, sum_vy = 0;
  int   n = 0;

  for (int i = 0, idx = (head_ - 1 + MAX_SAMPLES) % MAX_SAMPLES; i < count_; ++i, idx = (idx - 1 + MAX_SAMPLES) % MAX_SAMPLES) {
    const Sample& s = buf_[idx];
    if (now_ms - s.t_ms > WIN_MS) break;
    n++;
    float th = deg_to_rad(s.track_deg);
    sum_cos += cosf(th);
    sum_sin += sinf(th);
    sum_vx  += s.vx_ms;
    sum_vy  += s.vy_ms;
  }

  if (n < 12) return false;
  // Ensure we have at least some duration
  if (now_ms - buf_[(head_ - n + MAX_SAMPLES) % MAX_SAMPLES].t_ms < MIN_MS) return false;

  float R = sqrtf(sum_cos*sum_cos + sum_sin*sum_sin) / (float)n; // 0..1; small means disperse
  if (R < 0.3f) {
    float wx = sum_vx / n;
    float wy = sum_vy / n;
    applyCandidate_(wx, wy, now_ms);
    return true;
  }
  return false;
}

bool WindEstimator::straightLegUpdate_(uint32_t now_ms){
  // Use last ~60 s to find upwind/downwind extremes
  const uint32_t WIN_MS = 60000;

  float vmax = -1e9f, vmax_track = NAN;
  float vmin =  1e9f, vmin_track = NAN;

  // Also accumulate a crude mean heading to avoid noise
  uint32_t newest_ms = 0, oldest_ms = now_ms;

  for (int i = 0, idx = (head_ - 1 + MAX_SAMPLES) % MAX_SAMPLES; i < count_; ++i, idx = (idx - 1 + MAX_SAMPLES) % MAX_SAMPLES) {
    const Sample& s = buf_[idx];
    if (now_ms - s.t_ms > WIN_MS) break;

    if (s.sog_ms > vmax){ vmax = s.sog_ms; vmax_track = s.track_deg; }
    if (s.sog_ms < vmin){ vmin = s.sog_ms; vmin_track = s.track_deg; }

    if (s.t_ms > newest_ms) newest_ms = s.t_ms;
    if (s.t_ms < oldest_ms) oldest_ms = s.t_ms;
  }

  // Need a reasonable window & separation
  if (!isfinite(vmax) || !isfinite(vmin)) return false;
  if (newest_ms - oldest_ms < 8000) return false; // not enough spread
  if (ang_diff_abs(vmax_track, vmin_track) < 120.0f) return false;

  float vw_ms = (vmax - vmin) * 0.5f;
  if (vw_ms < 0.0f) return false; // numerical oddity

  // Downwind direction ~ heading of max ground speed
  float theta = deg_to_rad(vmax_track);
  float wx = vw_ms * sinf(theta); // East
  float wy = vw_ms * cosf(theta); // North

  applyCandidate_(wx, wy, now_ms);
  return true;
}

void WindEstimator::update(uint32_t now_ms, float sog_kts, float track_deg){
  if (isnan(sog_kts) || isnan(track_deg)) return;
  if (sog_kts < 3.0f) return; // too slow to be informative

  Sample s{};
  float th = deg_to_rad(track_deg);
  s.sog_ms    = kts_to_ms(sog_kts);
  s.vx_ms     = s.sog_ms * sinf(th); // East
  s.vy_ms     = s.sog_ms * cosf(th); // North
  s.track_deg = track_deg;
  s.t_ms      = now_ms;
  push_(s);

  // Try circle-based update first; if not, try straight-leg extremes
  if (!circleUpdate_(now_ms)) {
    straightLegUpdate_(now_ms);
  }
}
