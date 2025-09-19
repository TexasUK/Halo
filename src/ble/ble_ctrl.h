#pragma once
#include <Arduino.h>

// ===== UUIDs (kept from your legacy build) =====
#define SERVICE_UUID                  "4fafc201-1fb5-459e-8fcc-c5c9c331914c"
#define FLASH_CHARACTERISTIC_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TEST_CHARACTERISTIC_UUID      "d7a2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9a"
#define VOLUME_CHARACTERISTIC_UUID    "f7a2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9b"
#define ELEVATION_CHARACTERISTIC_UUID "a8b2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9c"
#define QNH_CHARACTERISTIC_UUID       "b9c2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9d"
#define RESET_CHARACTERISTIC_UUID     "c8b2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9e"
#define DATASOURCE_CHAR_UUID          "d8b2d055-5c6a-4b8a-8c0d-2e1e1c6f4b9f"

// ===== App hooks (implemented in main.cpp) =====
void halo_set_volume_runtime_and_persist(uint8_t vol0_30);
void halo_set_qnh_runtime_and_persist(uint16_t hpa);
void halo_set_elev_runtime_and_persist(uint16_t feet);
void halo_set_datasource_and_baud(bool isSoftRF, uint8_t baudIndex); // 0=19200, 1=38400
void halo_apply_nav_baud(uint32_t baud);

// Call once (after splash, when the system is up)
void bleInit();

// Call every loop with millis()
void bleTick(uint32_t now);

// Cancel any running test sequence (call from your 'C' panic)
void bleCancelTests();
