#pragma once
#include <stdint.h>

enum class AppEvent : uint8_t {
  None = 0,
  FlarmConnected,
  FlarmLost,
  FlightStarted,
  FlightLanded,
  TrafficAlert,     // alarm>0 arrived
};

enum class AudioState : uint8_t {
  Idle = 0,
  AwaitingPart2,    // waiting to play the clock direction
};
