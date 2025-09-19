#pragma once
#include <Arduino.h>

void nav_begin(HardwareSerial& port, int rxPin, uint32_t baud);
void nav_tick();
bool navValid();

// For test harness: inject a full NMEA sentence (e.g. "$GNRMC,...\n")
void nav_inject_nmea(const char* line);
