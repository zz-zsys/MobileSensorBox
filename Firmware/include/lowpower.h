// lowpower.h

#ifndef _LOWPOWER_h
#define _LOWPOWER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include "driver/rtc_io.h"
#include "globals.h"

typedef __PACKED_PRE struct LP_CONFIG_Ttag {
	bool		debug;
} __PACKED_POST LP_CONFIG_T;

void lp_debug(bool debug = false, HardwareSerial &port = Serial);
void lp_isolateGPIO();
void lp_disableHoldGPIO();
void lp_turnOffBluetooth();
void lp_turnOffWifi();
void lp_printWakeupTouchpad();
void lp_printWakeupReason();
esp_err_t lp_init(bool debug = false, HardwareSerial &port = Serial);
esp_err_t lp_deep_sleep(gpio_num_t gpio_num = GPIO_NUM_39, int level = 0);
esp_err_t lp_deep_sleep_timer(uint64_t time_in_us, gpio_num_t gpio_num = GPIO_NUM_39, int level = 0);
void lp_taskInit(uint32_t ulStackDepth = 2048, UBaseType_t uxPriority = 5, const BaseType_t xCoreID = 1);

#endif

