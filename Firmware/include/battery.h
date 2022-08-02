#ifndef _BATTERY_h
#define _BATTERY_h

#include <Arduino.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include <esp_adc_cal.h>
#include "globals.h"

#define BAT_ADC_GPIO_CHANNEL	ADC1_GPIO35_CHANNEL	// battery probe GPIO pin -> ADC1_CHANNEL_7
#define BAT_VOLTAGE_DIVIDER		1.97				// battery divider ratio
#define MIN_BATT_THRESHOLD		3400				// minimum battery level threshold for empty		[mV]
#define MAX_BATT_THRESHOLD		4100				// maximum battery level threshold for full charged [mV]
#define OTA_MIN_BATT			3600				// minimum battery level for OTA [mV]
#define DEFAULT_VREF			1100				// alternative: use adc2_vref_to_gpio() for better estimate
#define BAT_ADC_SAMPLING_COUNT	64					// do some multisampling to get better values

typedef __PACKED_PRE struct BATTERY_CONFIG_Ttag {
	bool				debug;
	adc1_channel_t		adc_channel;
	adc_atten_t			atten;
	adc_unit_t			unit;
	adc_bits_width_t	width;
}__PACKED_POST BATTERY_CONFIG_T;

typedef enum eBatteryStatus_tTag {					// data port enumeration for display queue
	BATT_LVL_FULL,
	BATT_LVL_MIDDLE,
	BATT_LVL_LOW
} eBatteryStatus_t;

void battery_debug(bool debug = false, HardwareSerial &port = Serial);
uint16_t battery_read_voltage(void);
void battery_init(bool debug = false, HardwareSerial &port = Serial);
eBatteryStatus_t battery_checkLevel();
void battery_taskInit(uint32_t ulStackDepth = 2048, UBaseType_t uxPriority = 1, const BaseType_t xCoreID = 0);

#endif

