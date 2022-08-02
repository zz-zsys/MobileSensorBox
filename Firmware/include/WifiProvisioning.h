// WifiProvisioning.h

#ifndef _WIFIPROVISIONING_h
#define _WIFIPROVISIONING_h

#include <Arduino.h>
#include "globals.h"

typedef struct WPS_CONFIG_Ttag {
	bool		debug;
} __attribute__((packed)) WPS_CONFIG_T;

void wps_debug(bool debug = false, HardwareSerial &port = Serial);
void wps_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 3, const BaseType_t xCoreID = 1);

#endif

