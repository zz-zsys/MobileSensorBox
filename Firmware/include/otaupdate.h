#ifndef _OTAUPDATE_h
#define _OTAUPDATE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include "globals.h"

extern const char* otaupdate;

void ota_initWifi();
void ota_initServer();
void otaTask(void * parameter);
void ota_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 3, const BaseType_t xCoreID = 1);

#endif

