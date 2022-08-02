#ifndef _GPS_h
#define _GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include <ublox_ubx.h>
#include "globals.h"

extern UbxPacketHandler gps;				// Make UbxPacketHandler instance globally available
extern HardwareSerial	gpsSerial;

typedef struct GPS_CONFIG_Ttag {
	bool		debug;				
	int			ser_port_nr;
	uint32_t	baud;
	uint32_t	ser_config;
	int8_t		rxPin;
	int8_t		txPin;
	uint32_t	sendIntervalMs;
} __attribute__((packed)) GPS_CONFIG_T;

esp_err_t gps_init(bool debug = false, HardwareSerial &debugPort = Serial);
void gps_debug(bool debug = false, HardwareSerial &port = Serial);
void sys_printCurrentTime(time_t input_time);
void sys_setupTime(time_t setTime, const char* value = tz_berlin);
time_t printTime(const char* source, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds);
bool gps_trackDevice();
void gps_enableSleep(uint32_t duration = 0);
void gps_wakeUp(void);
void gpsTask(void * parameter);
void gps_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 0);

#endif

