#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "globals.h"
#include "battery.h"
#include "bme280.h"
#include "gps.h"
#include "display.h"
#include "pmsensor.h"
#include "no2_sensor.h"
#include "lorawan.h"
#include "watchdog.h"
#include "lowpower.h"
#include "led.h"
#include "otaupdate.h"
#include "WifiProvisioning.h"
#include "WebServerClient.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

SemaphoreHandle_t xSemaphoreI2C;
SemaphoreHandle_t xSemaphoreBattAdc;

uint32_t printSysTime = 0;
uint32_t buttonPressedTime = 0;
uint32_t buttonReleaseTime = 0;
uint32_t lRet = ESP_OK;
bool doBlink = false;

GPIO_T button = {GPIO_NUM_39, INPUT};

typedef enum BUTTON_STATE_Ttag {
	BUTTON_CHECK_PRESSED,
	BUTTON_CHECK_RELEASE,
} BUTTON_STATE_T;

volatile BUTTON_STATE_T buttonState = BUTTON_CHECK_PRESSED;

typedef struct  {
	int sda;
	int scl;
	uint32_t frequency;
} i2c_config_t;

i2c_config_t i2c_conf = {
	.sda = SDA,
	.scl = SCL,
	.frequency = 100000
};


void IRAM_ATTR BUTTON_ISR_CHANGE() {
	switch (buttonState) {
		case BUTTON_CHECK_PRESSED: button.pressed = true; break;
		case BUTTON_CHECK_RELEASE: button.release = true; break;
	}
}

void printSystemInfo() {
	configTime(0, 0, "pool.ntp.org");

	Serial.printf("Task\t\t| Bytes Left\t| State\n");
	Serial.printf("Main\t\t| %d\t\t|	\n", ESP.getFreeHeap());
	Serial.printf("BME\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskBme), eTaskGetState(xTaskBme));
	Serial.printf("SPS30\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskSps), eTaskGetState(xTaskSps));
	Serial.printf("NO2\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskDgs), eTaskGetState(xTaskDgs));
	Serial.printf("GPS\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskGps), eTaskGetState(xTaskGps));
	Serial.printf("LoRa\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskLora), eTaskGetState(xTaskLora));
	Serial.printf("Display\t\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskDisplay), eTaskGetState(xTaskDisplay));
	Serial.printf("Watchdog\t| %d\t\t| %d\n", uxTaskGetStackHighWaterMark(xTaskWatchdog), eTaskGetState(xTaskWatchdog));
	Serial.println();
}

void set_tzenv(const char* value) {
	// set/change the environment timezone
	setenv("TZ", value, 1);	// default Berlin, Germany

	// set time zone
	tzset();
}

void setup() {

	// initialize the serial port
	Serial.begin(115200);

	// assign the semaphore for the mutex
	xSemaphoreI2C = xSemaphoreCreateMutex();
	xSemaphoreBattAdc = xSemaphoreCreateMutex();

	Wire.begin(i2c_conf.sda, i2c_conf.scl, i2c_conf.frequency);

	// initialize the battery measurement function where the ADC is configured
	battery_taskInit();
	lp_taskInit();
	lora_taskInit();
	bme_taskInit();
	sps_taskInit();
	dgs_taskInit();
	gps_taskInit();
	display_taskInit();

	vTaskDelay(10000 / portTICK_PERIOD_MS);

	watchdog_taskInit();

	pinMode(button.pin, button.mode);
	//attachInterrupt(digitalPinToInterrupt(button.pin), BUTTON_ISR_CHANGE, CHANGE);
	attachInterrupt(digitalPinToInterrupt(button.pin), BUTTON_ISR_CHANGE, RISING);


	log_d("Setup is finished");

}


void loop() {

	switch (buttonState) {
		case BUTTON_CHECK_PRESSED:
		{
			if (button.pressed) {
				buttonState = BUTTON_CHECK_RELEASE;
				buttonPressedTime = millis();
				button.pressed = false;
				log_i("Button pressed");
			}
			break;
		}
		case BUTTON_CHECK_RELEASE:
		{
			if (button.release) {
				log_i("Button released");
				buttonState = BUTTON_CHECK_PRESSED;
				buttonReleaseTime = millis();
				log_i("TotalTime = %d", buttonReleaseTime - buttonPressedTime);
				if (buttonReleaseTime - buttonPressedTime > 7000) {
					doBlink = true;
					//ota_taskInit();
					wps_taskInit();

				}
				else if (buttonReleaseTime - buttonPressedTime >= 1000 && buttonReleaseTime - buttonPressedTime <= 7000) {
					doBlink = false;
					//lRet = lp_deep_sleep_timer(LP_SLEEP_TIME, /*gpio_num_t */ button.pin, /*level*/ 0);
					Serial.printf("Cancel DGS warmup\n");
					dgs_cancelWarmUp();
				}
				else {
					// do something..
					doBlink = false;
				}

				button.release = false;
			}
			break;
		}
	}

	vTaskDelay(1);
}