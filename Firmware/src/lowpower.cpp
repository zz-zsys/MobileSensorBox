#include "lowpower.h"
#include "gps.h"
#include "pmsensor.h"
#include "no2_sensor.h"
#include "bme280.h"
#include "display.h"
#include "led.h"

LP_CONFIG_T lpConfig = {
	.debug = false
};

static HardwareSerial* debugSerial;

TaskHandle_t xTaskLp;
EventGroupHandle_t xLowPowerEvent;
static uint32_t lp_result = 0;
bool isLowPowerActivated = false;


void lp_debug(bool debug, HardwareSerial &port) {
	lpConfig.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		print the system current time
* @param[in]	input_time		input time
* @retval		none
* @note			debugging mode must be enabled to set the current time on serial monitor
*/
/************************************************************************************************************************/
void printCurrentTime(time_t input_time) {
	struct tm localtm;
	struct tm gmtm;

	Serial.printf("setTime: %ld\n", input_time);
	
	gmtime_r(&input_time, &gmtm);
	Serial.printf("The GMT date/time is: %s", asctime(&gmtm));
	
	localtime_r(&input_time, &localtm);
	Serial.printf("The current date/time is: %s", asctime(&localtm));
}

/************************************************************************************************************************/
/*!
* @brief		setup the system time
* @param[in]	setTime		epoch time in time_t format
* @param[in]	value		timezone setenv value (see https://bit.ly/2JjeoBD for more examples)
* @retval		none
*/
/************************************************************************************************************************/
void setupTime(time_t setTime, const char* value) {

	int ret = 0;

	//// log the set time into the lastConfigTime
	//lastConfigTime = setTime;

	struct timeval now = { .tv_sec = setTime };
	
	// set the system time
	settimeofday(&now, NULL);

	setenv("TZ", value, 1);

	tzset();

	time_t nowTime = time(NULL);

	// print the current time
	printCurrentTime(nowTime);
}

/************************************************************************************************************************/
/*!
* @brief		disconnect internal circuits from an RTC IO to minimize leakage current during deep sleep
* @retval		none
*/
/************************************************************************************************************************/
void lp_isolateGPIO() {
	for (int i = 0; i < 40; i++) {
		gpio_num_t pin = gpio_num_t(i);
		if (rtc_gpio_is_valid_gpio(pin)) {
			rtc_gpio_isolate(pin);
		}
	}
}

/************************************************************************************************************************/
/*!
* @brief		disabling hold function to allow the pad to be set again
* @retval		none
*/
/************************************************************************************************************************/
void lp_disableHoldGPIO() {
	for (int i = 0; i < 40; i++) {
		gpio_num_t pin = gpio_num_t(i);
		if (rtc_gpio_is_valid_gpio(pin)) {
			rtc_gpio_hold_dis(pin);
		}
	}
}

/************************************************************************************************************************/
/*!
* @brief		turn off bluetooth
* @retval		none
*/
/************************************************************************************************************************/
void lp_turnOffBluetooth() {
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
}

/************************************************************************************************************************/
/*!
* @brief		turn off wifi
* @retval		none
*/
/************************************************************************************************************************/
void lp_turnOffWifi() {
	esp_wifi_stop();
	esp_wifi_deinit();
}

/************************************************************************************************************************/
/*!
* @brief		Print the touchpad by which ESP32 has been awaken from sleep
* @retval		none
*/
/************************************************************************************************************************/
void lp_printWakeupTouchpad() {
	touch_pad_t pin;
	pin = esp_sleep_get_touchpad_wakeup_status();
	switch (pin)
	{
	case 0: debugSerial->println("Touch detected on GPIO 4"); break;
	case 1: debugSerial->println("Touch detected on GPIO 0"); break;
	case 2: debugSerial->println("Touch detected on GPIO 2"); break;
	case 3: debugSerial->println("Touch detected on GPIO 15"); break;
	case 4: debugSerial->println("Touch detected on GPIO 13"); break;
	case 5: debugSerial->println("Touch detected on GPIO 12"); break;
	case 6: debugSerial->println("Touch detected on GPIO 14"); break;
	case 7: debugSerial->println("Touch detected on GPIO 27"); break;
	case 8: debugSerial->println("Touch detected on GPIO 33"); break;
	case 9: debugSerial->println("Touch detected on GPIO 32"); break;
	default: debugSerial->println("Wakeup not by touchpad"); break;
	}
}

/************************************************************************************************************************/
/*!
* @brief		Print the reason by which ESP32 has been awaken from sleep
* @retval		none
*/
/************************************************************************************************************************/
void lp_printWakeupReason() {
	struct tm localtm;
	esp_sleep_wakeup_cause_t wakeup_reason;
	wakeup_reason = esp_sleep_get_wakeup_cause();
	switch (wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0: {

		debugSerial->println("Wakeup caused by external signal using RTC_IO");
		
		setenv("TZ", tz_berlin, 1);

		tzset();

		time(&currentTime);

		localtime_r(&currentTime, &localtm);

		debugSerial->printf("Wakeup timestamp: %s\n", asctime(&localtm));
		break;
	}
	case ESP_SLEEP_WAKEUP_EXT1: {
		debugSerial->println("Wakeup caused by external signal using RTC_CNTL");
		break;
	};
	case ESP_SLEEP_WAKEUP_TIMER: {
		debugSerial->println("Wakeup caused by timer");
		
		setenv("TZ", tz_berlin, 1);

		tzset();

		time(&currentTime);

		localtime_r(&currentTime, &localtm);


		debugSerial->printf("Wakeup timestamp: %s\n", asctime(&localtm));
		
		break;
	}
	case ESP_SLEEP_WAKEUP_TOUCHPAD: {
		debugSerial->println("Wakeup caused by touchpad");
		lp_printWakeupTouchpad();
		break;
	}
	case ESP_SLEEP_WAKEUP_ULP: debugSerial->println("Wakeup caused by ULP program"); break;
	case ESP_SLEEP_WAKEUP_GPIO: debugSerial->println("Wakeup caused by GPIO"); break;
	case ESP_SLEEP_WAKEUP_UART: debugSerial->println("Wakeup caused by UART"); break;
	default: {
		debugSerial->println("Wakeup was not caused by deep sleep");
		break;
	}
	}
}

/************************************************************************************************************************/
/*!
* @brief		Initialise the low power function
* @param[in]	debug	debug flag (true if enable, false if disable)
* @param[in]	port	serial port for debugging purpose
* @retval		none
*/
/************************************************************************************************************************/
esp_err_t lp_init(bool debug, HardwareSerial &port) {

	// create the FreeRTOS event group
	if (xLowPowerEvent == NULL) xLowPowerEvent = xEventGroupCreate();
	
	if (xLowPowerEvent == NULL) return ESP_FAIL;
	
	lp_debug(debug, port);

	// print the wake up reason
	lp_printWakeupReason();

	// disable gpio pad hold function of all GPIO pins.
	lp_disableHoldGPIO();

	return ESP_OK;
}

/************************************************************************************************************************/
/*!
* @brief		enable the deep sleep function of the whole system
* @param[in]	debug	debug flag (true if enable, false if disable)
* @param[in]	port	serial port for debugging purpose
* @retval		none
*/
/************************************************************************************************************************/
esp_err_t lp_deep_sleep_timer(uint64_t time_in_us, gpio_num_t gpio_num, int level)
{
	esp_err_t lRet = ESP_OK;

	isLowPowerActivated = true;

	if (xTaskWatchdog != NULL) {
		debugSerial->printf("Deleting Watchdog task...");
		vTaskDelete(xTaskWatchdog);
		debugSerial->printf("Deleted!\n");
	}

	if (xTaskLora != NULL) {
		debugSerial->printf("Deleting Lora task...");
		vTaskDelete(xTaskLora);
		debugSerial->printf("Deleted!\n");
	}

	if (xTaskDgs != NULL) {
		debugSerial->printf("Deleting DGS task...");
		vTaskDelete(xTaskDgs);
		debugSerial->printf("Deleted!\n");
	}

	if (xTaskGps != NULL) {
		debugSerial->printf("Deleting GPS task...");
		vTaskDelete(xTaskGps);
		debugSerial->printf("Deleted!\n");
	}

	do {} while (xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) != pdTRUE);

	if (xTaskSps != NULL) {
		debugSerial->printf("Deleting SPS30 task...");
		vTaskDelete(xTaskSps);
		debugSerial->printf("Deleted!\n");
	}

	if (xTaskBme != NULL) {
		debugSerial->printf("Deleting BME task...");
		vTaskDelete(xTaskBme);
		debugSerial->printf("Deleted!\n");
	}

	if (xTaskDisplay != NULL) {
		debugSerial->printf("Deleting Display task...");
		vTaskDelete(xTaskDisplay);
		debugSerial->printf("Deleted!\n");
	}

	xSemaphoreGive(xSemaphoreI2C);

	debugSerial->printf("Deleting all I2C task finished!\n");


	if (xTaskGps != NULL) {
		debugSerial->printf("Enable GPS sleep mode\n");
		gps_enableSleep(0);
	}

	if (lpConfig.debug) delay(5000);

	//if (xTaskDgs != NULL) {
	//	debugSerial->printf("Enable DGS sleep mode\n");
	//	dgs_enableSleep();
	//	delay(2000);
	//}

	if (xTaskBme != NULL) {
		debugSerial->printf("Enable the BME280 sleep mode...");
		while (ESP_OK != (lRet = bme_enableSleep())) {
			// wait until the BME sensor is sleep
			debugSerial->printf(".");
			delay(1000);
		};
		debugSerial->printf("BME280 sleep mode success!\n");
	}

	if (lpConfig.debug) delay(5000);

	if (xTaskDisplay != NULL) {
		debugSerial->printf("Enable the display sleep mode...");
		while (ESP_OK != (lRet = disp_sleep())) {
			// wait until the display is sleep
			debugSerial->printf(".");
			delay(1000);
		};
		debugSerial->printf("Display sleep mode success!\n");
	}

	if (lpConfig.debug) delay(5000);


	if (xTaskSps != NULL) {
		debugSerial->printf("Enable the SPS30 sleep mode...\n");
		while (ESP_OK != (lRet = sps_enableSleep())) {
			// wait until the SPS30 sensor is sleep
			debugSerial->printf(".");
			delay(1000);
		};
		debugSerial->printf("SPS30 sleep mode success!\n");
	}

	if (lpConfig.debug) delay(5000);

	// turn off the led
	debugSerial->printf("Turn off the LEDs\n");
	batt_full_led_state(LED_OFF);
	batt_low_led_state(LED_OFF);
	measure_led_state(LED_OFF);

	if (lpConfig.debug) delay(5000);

	// turn off wifi and ble, (at the moment there is no effect as wifi and ble aren't used)
	lp_turnOffWifi();
	lp_turnOffBluetooth();

	lp_isolateGPIO();

	esp_sleep_enable_ext0_wakeup(gpio_num, level);

	if(time_in_us > 0) esp_sleep_enable_timer_wakeup(time_in_us);

	debugSerial->printf("Going really to deep sleep NOW!\n");

	struct tm localtm;

	time(&currentTime);

	localtime_r(&currentTime, &localtm);

	debugSerial->printf("Sleep timestamp: %s\n", asctime(&localtm));

	esp_deep_sleep_start();
}

void lpTask(void * parameter) {
	
	lp_debug(false);
	debugSerial->printf("Start LowPower task\n");

	esp_err_t lRet = ESP_OK;

	while (ESP_OK != (lRet = lp_init(LP_DEBUG))) {
		// wait until low power init is finished
	};

	for (;;) {

		vTaskDelay(LP_EVENT_INTERVAL_TIME);

		// read bits within an RTOS event group with set interval time to check if all event still responsive
		lp_result = xEventGroupWaitBits(xLowPowerEvent,
			allLpEventId,
			pdTRUE,
			pdTRUE,
			LP_EVENT_INTERVAL_TIME);

		if ((lp_result & allLpEventId) == allLpEventId) {
			log_i("Low power not needed yet..\n");
		}
		else {
			if (!(lp_result & gpsLpEventId)) {
				lp_deep_sleep_timer(LP_SLEEP_TIME, /*gpio_num_t */ GPIO_NUM_39, /*level*/ 0);
			}
		}
	}
}

void lp_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(lpTask, "lpTask", ulStackDepth, (void*)1, uxPriority, &xTaskLp, xCoreID);
}

