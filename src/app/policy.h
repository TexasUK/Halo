#pragma once
#include <stdint.h>

// --- Strobe periods by alert level (ms) ---
// Make this header self-contained so it doesn’t rely on other headers.
static constexpr uint16_t STROBE_PERIOD_L0 = 2000;  // standard
static constexpr uint16_t STROBE_PERIOD_L1 = 1400;  // gentle
static constexpr uint16_t STROBE_PERIOD_L2 =  900;  // faster
static constexpr uint16_t STROBE_PERIOD_L3 =  500;  // fastest

// Map alarm -> strobe period
inline uint16_t strobe_period_for_level(int alarm){
  if (alarm >= 3) return STROBE_PERIOD_L3;
  if (alarm == 2) return STROBE_PERIOD_L2;
  if (alarm == 1) return STROBE_PERIOD_L1;
  return STROBE_PERIOD_L0;
}

// 12-o’clock sector from absolute bearing and own heading
inline int clock_from_bearings(float target_abs_deg, float own_heading_deg){
  if (!(own_heading_deg==own_heading_deg)) own_heading_deg = 0.0f; // NaN guard
  float rel = target_abs_deg - own_heading_deg;
  while (rel < 0)      rel += 360.0f;
  while (rel >= 360.0f) rel -= 360.0f;
  int sector = ((int)((rel + 15.0f) / 30.0f)) % 12;
  return sector == 0 ? 12 : sector;
}

// Vertical category (ft)
enum class VertCat : uint8_t { Below, Level, Above };
inline VertCat vertical_category_ft(float rel_ft){
  if (rel_ft >  200.0f) return VertCat::Above;
  if (rel_ft < -200.0f) return VertCat::Below;
  return VertCat::Level;
}
