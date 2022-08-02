#ifndef _DISPLAY_h
#define _DISPLAY_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include <U8g2lib.h>
#include "globals.h"
#include "time.h"
#include "sys/time.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2; // Make U8G2 instance globally available

typedef  __PACKED_PRE struct DISPLAY_CONFIG_Ttag {
	bool		debug;
	uint32_t	sda_pin;
	uint32_t	scl_pin;
	uint32_t	freq;
	uint8_t		addr;
	uint32_t	refreshTimeInMs;

} __PACKED_POST DISPLAY_CONFIG_T;

esp_err_t disp_init();
esp_err_t disp_sleep();

void disp_debug(bool debug = false, HardwareSerial &port = Serial);
void disp_frame(struct tm *timeInfo, int16_t *temp_x100, uint8_t *hum, int16_t *ppb, float *lat, float *lon, float *pm2_5, float *pm10_0, uint16_t *avgParticleSize_x1000);
void displayTask(void * parameter);
void display_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 1);

#endif

