// pmsensor.h

#ifndef _PMSENSOR_h
#define _PMSENSOR_h

#include <Arduino.h>
#include <sps30.h>
#include "globals.h"


//#define SP30_COMMS I2C_COMMS

extern SPS30 sps30;

typedef __PACKED_PRE struct SPS30_CONFIG_Ttag {
	uint8_t		debug;
	serial_port ser_com;
	int			sdapin;
	int			sclpin;
	uint32_t	freq;
	uint32_t	sendIntervalMs;
} __PACKED_POST SPS30_CONFIG_T;

void sps_errToMess(const char *mess, uint8_t r);
void sps_errorLoop(const char *mess, uint8_t r);
void sps_debug(uint8_t debug = 0, uint8_t libDebug = 0, HardwareSerial &port = Serial);
esp_err_t sps_init(uint8_t debug = 0, uint8_t libDebug = 0, HardwareSerial &debugPort = Serial);
void sps_getDeviceInfo();
uint8_t sps_readAll();
esp_err_t sps_enableSleep(bool enable = true);

void spsTask(void * parameter);
void sps_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 1);




#endif

