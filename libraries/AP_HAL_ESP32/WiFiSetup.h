#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"

#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 5
#endif

void wifi_init_softap(void);

void initialize_wifi();

