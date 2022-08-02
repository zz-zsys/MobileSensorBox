#ifndef _BME280_h
#define _BME280_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include <SparkFunBME280.h>
#include "globals.h"

#define BME_I2C_ADDRESS 0x76	// I2C address of the used bme280 sensor

extern BME280 bme;				// Make BME280 instance globally available

typedef  __PACKED_PRE struct BME_CONFIG_Ttag {
	bool		debug;
	int			sdapin;
	int			sclpin;
	uint32_t	freq;
	uint8_t		addr;
	uint32_t	sendIntervalMs;
	float		tempOffset;
} __PACKED_POST BME_CONFIG_T;

void bme_debug(bool debug = false, HardwareSerial &port = Serial);
esp_err_t bme_init(bool debug = false, HardwareSerial &debugPort = Serial);
esp_err_t bme_getData(int16_t *sTemp_x100, uint8_t *bHum, uint16_t *usPressure_hPa_x10);
esp_err_t bme_enableSleep(bool enable = true);
void bmeTask(void * parameter);
void bme_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 1);

#endif

