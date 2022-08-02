// watchdog.h

#ifndef _WATCHDOG_h
#define _WATCHDOG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include "globals.h"

typedef  __PACKED_PRE struct WD_CONFIG_Ttag {
	bool		debug;
} __PACKED_POST WD_CONFIG_T;

void watchdog_debug(bool debug = false, HardwareSerial &port = Serial);
esp_err_t watchdog_init(bool debug = false, HardwareSerial &debugPort = Serial);

void watchdogTask(void * parameter);
void watchdog_taskInit(uint32_t ulStackDepth = 2048, UBaseType_t uxPriority = 5, const BaseType_t xCoreID = 0);


#endif

