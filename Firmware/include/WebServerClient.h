#ifndef _WEBSERVERCLIENT_h
#define _WEBSERVERCLEINT_h

#include <Arduino.h>
#include "globals.h"

typedef struct WSC_CONFIG_Ttag {
	bool		debug;
	uint32_t	sendIntervalInMs;
} __attribute__((packed)) WSC_CONFIG_T;

void wsc_debug(bool debug = false, HardwareSerial &port = Serial);
void wsc_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 5, const BaseType_t xCoreID = 1);
void wsc_taskDelete();

#endif

