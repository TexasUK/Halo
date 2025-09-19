#pragma once
#include <Arduino.h>

void test_start();
void test_stop();
bool test_is_running();
void test_tick(uint32_t now_ms);
