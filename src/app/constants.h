#pragma once

// ---- Timing / thresholds ----
static constexpr float    TAKEOFF_KTS            = 20.0f;
static constexpr float    LANDING_KTS            = 15.0f;     // not used for landing rule now
static constexpr float    TAKEOFF_ALT_FT         = 200.0f;    // baro fallback takeoff threshold
static constexpr float    LANDING_ALT_FT         = 200.0f;    // authoritative landing threshold (AGL <= 200ft)
static constexpr uint32_t TAKEOFF_HOLD_MS        = 3000;      // 3s hold for speed/alt edges
static constexpr uint32_t LANDING_HOLD_MS        = 8000;      // show Landing screen this long

// ---- FLARM timing ----
static constexpr uint32_t FLARM_TIMEOUT_MS       = 8000;
static constexpr uint32_t FLARM_EDGE_HYST_MS     = 500;

// ---- Audio sequencing ----
static constexpr uint32_t AUDIO_PART2_GUARD_MS   = 1200;

// ---- UI ----
static constexpr uint32_t ALERT_HOLD_MS          = 8000;

// ---- Strobe ----
static constexpr uint16_t STROBE_ON_MS           = 120;

// ---- Shared tiny helpers ----
static inline float m_to_ft(float m) { return m * 3.28084f; }
