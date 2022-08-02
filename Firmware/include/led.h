#ifndef _LED_h
#define _LED_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include "globals.h"

#define FULL_BATT_LED			GPIO_NUM_13
#define LOW_BATT_LED			GPIO_NUM_2
#define MEASURE_LED				GPIO_NUM_14

typedef enum eLedStatus_tTag {					// data port enumeration for display queue
	LED_OFF,
	LED_ON,
	LED_BLINK
} eLedStatus_t;

void full_batt_led_init();
void low_batt_led_init();
void batt_led_init();
void measure_led_init();
void batt_full_led_state(eLedStatus_t state);
void batt_low_led_state(eLedStatus_t state);
void measure_led_state(eLedStatus_t state);
#endif

