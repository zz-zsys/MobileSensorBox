// no2_sensor.h

#ifndef _NO2_SENSOR_h
#define _NO2_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include <DGS.h>
#include "globals.h"

extern DGS dgs;

typedef __PACKED_PRE struct DGS_CONFIG_Ttag {
	bool		debug;
	int			ser_port_nr;
	uint32_t	baud;
	uint32_t	ser_config;
	int8_t		rxPin;
	int8_t		txPin;
	uint32_t	sendIntervalMs;
} __PACKED_POST DGS_CONFIG_T;

void dgs_debug(bool debug = false, bool libDebug = false, HardwareSerial &port = Serial);
esp_err_t dgs_init(bool debug = false, bool libDebug = false);
esp_err_t dgs_readAll();
void dgs_resetModule();
void dgs_enableSleep(bool enable = true);
void dgs_cancelWarmUp();
void dgsTask(void *parameter);
void dgs_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 1);



#endif

