#pragma once
#include <Arduino.h>

// Initialize DFPlayer (TX pin to module RX, module BUSY pin is active-LOW)
void dfp_begin(HardwareSerial& serial, int txPin, int busyPin, uint32_t baud, uint8_t volume0_30);

// Non-blocking tick (call every loop)
void dfp_tick();

// Enqueue a filename index (1..3000) to play (maps to /MP3/00NN.mp3)
void dfp_play_filename(uint16_t n);

// Stop current playback and drop any queued items (compat: use if you want a hard reset)
void dfp_stop_and_flush();

// ---- Compatibility helpers for existing code ----
// Clear the pending queue but do not send STOP to the module.
void dfp_clear_queue();

// Send STOP to the module (does not clear queued items).
void dfp_stop();
