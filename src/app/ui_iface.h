#pragma once
#include <Arduino.h>

enum Page : uint8_t {
  PAGE_BOOT = 0,
  PAGE_COMPASS = 1,
  PAGE_TRAFFIC = 2,
  PAGE_LANDING = 3,
  PAGE_LANDED  = 4,   // NEW
};

// Implemented in main.cpp
void ui_set_page(Page p);
void ui_markAllUndrawn();
