// 
// 
// 

#include "led.h"

bool isBattFullLedEnable = false;
bool isBattLowLedEnable = false;
bool isMeasureLedEnable = false;

void full_batt_led_init() {
	gpio_pad_select_gpio(FULL_BATT_LED);
	gpio_set_direction(FULL_BATT_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(FULL_BATT_LED, 0);
}

void low_batt_led_init() {
	gpio_pad_select_gpio(LOW_BATT_LED);
	gpio_set_direction(LOW_BATT_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(LOW_BATT_LED, 0);
}

void batt_led_init() {
	full_batt_led_init();
	low_batt_led_init();
}
void measure_led_init() {
	gpio_pad_select_gpio(MEASURE_LED);
	gpio_set_direction(MEASURE_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(MEASURE_LED, 0);

}
void batt_full_led_state(eLedStatus_t state) {
	switch (state) {
		case LED_OFF: {
			gpio_set_level(FULL_BATT_LED, 0);
			break;
		}
		case LED_ON: {
			gpio_set_level(FULL_BATT_LED, 1);
			break;
		}
		case LED_BLINK: {
			if (isBattFullLedEnable) {
				gpio_set_level(FULL_BATT_LED, 0);
				isBattFullLedEnable = false;
			}
			else {
				gpio_set_level(FULL_BATT_LED, 1);
				isBattFullLedEnable = true;
			}
			break;
		}
	}
}
void batt_low_led_state(eLedStatus_t state) {
	switch (state) {
		case LED_OFF: {
			gpio_set_level(LOW_BATT_LED, 0);
			break;
		}
		case LED_ON: {
			gpio_set_level(LOW_BATT_LED, 1);
			break;
		}
		case LED_BLINK: {
			if (isBattLowLedEnable) {
				gpio_set_level(LOW_BATT_LED, 0);
				isBattLowLedEnable = false;
			}
			else {
				gpio_set_level(LOW_BATT_LED, 1);
				isBattLowLedEnable = true;
			}
			break;
		}
	}
}
void measure_led_state(eLedStatus_t state) {
	switch (state) {
		case LED_OFF: {
			gpio_set_level(MEASURE_LED, 0);
			break;
		}
		case LED_ON: {
			gpio_set_level(MEASURE_LED, 1);
			break;
		}
		case LED_BLINK: {
			if (isMeasureLedEnable) {
				gpio_set_level(MEASURE_LED, 0);
				isMeasureLedEnable = false;
			}
			else {
				gpio_set_level(MEASURE_LED, 1);
				isMeasureLedEnable = true;
			}
			break;
		}
	}
}
