// 
// 
// 

#include "watchdog.h"
#include "battery.h"
#include "bme280.h"
#include "gps.h"
#include "pmsensor.h"
#include "no2_sensor.h"
#include "lorawan.h"
#include "watchdog.h"
#include "display.h"

static HardwareSerial *debugSerial;
WD_CONFIG_T wd_config = {
	.debug = false
};

TaskHandle_t xTaskWatchdog;
EventGroupHandle_t xWatchdogEvent;								// FreeRTOS event handle for watchdog task
bool isLorawanStopResponding;
bool isBmeStopResponding;
bool isSpsStopResponding;
bool isDgsStopResponding;
bool isGpsStopResponding;
bool isDisplayStopResponding;

static uint32_t wd_result = 0;


void watchdog_debug(bool debug, HardwareSerial &port) {
	wd_config.debug = debug;
	debugSerial = &port;
}

esp_err_t watchdog_init(bool debug, HardwareSerial &debugPort) {

	watchdog_debug(debug, debugPort);

	// create the FreeRTOS event group
	if (xWatchdogEvent == NULL) xWatchdogEvent = xEventGroupCreate();
	
	if (xWatchdogEvent == NULL) return ESP_FAIL;
	else return ESP_OK;

}

void watchdogTask(void * parameter) {

	watchdog_debug(false);

	debugSerial->printf("Start watchdog task!\n");

	esp_err_t lRet = ESP_OK;

	while (ESP_OK != (lRet = watchdog_init(true))) {
		// wait until watchdog init is finished
	};

	for (;;) {

		// wait for the watchdog event interval
		vTaskDelay(WD_EVENT_INTERVAL_TIME);

		// read bits within an RTOS event group every 10 s to check if all task still responsive
		wd_result = xEventGroupWaitBits(xWatchdogEvent,
			allWdEventId,
			pdTRUE,
			pdTRUE,
			WD_EVENT_INTERVAL_TIME);

		if ((wd_result & allWdEventId) == allWdEventId) {
			debugSerial->printf("WD: System is healthy..\n");
		}
		else {
			if (!(wd_result & lorawanWdEventId)) {
				log_e("Lorawan stopped responding..restart task\n");
				isLorawanStopResponding = true;
			}
			if (!(wd_result & bmeWdEventId)) {
				log_e("BME280 stopped responding..restart task\n");
				isBmeStopResponding = true;
			}
			if (!(wd_result & spsWdEventId)) {
				log_e("SPS30 stopped responding..restart task\n");
				isSpsStopResponding = true;
			}
			if (!(wd_result & dgsWdEventId)) {
				log_e("DGS stopped responding..restart task\n");
				isDgsStopResponding = true;
			}
			if (!(wd_result & gpsWdEventId)) {
				log_e("GPS stopped responding..restart task\n");
				isGpsStopResponding = true;
			}
			if (!(wd_result & displayWdEventId)) {
				log_e("Display stopped responding..restart task\n");
				isDisplayStopResponding = true;
			}


			// if the lorawan task stop responding
			if (isLorawanStopResponding) {
				isLorawanStopResponding = false;

				// delete the lorawan task
				if(xTaskLora != NULL) vTaskDelete(xTaskLora);

				// create the new lorawan task
				lora_taskInit();
			}

			// if the lorawan task stop responding
			if (isBmeStopResponding) {
				isBmeStopResponding = false;

				// release the semaphore for the related task
				I2C_MUTEX_UNLOCK();

				// delete the lorawan task
				if (xTaskBme != NULL) vTaskDelete(xTaskBme);

				// create the new lorawan task
				bme_taskInit();
			}

			// if the sps30 task stop responding
			if (isSpsStopResponding) {
				isSpsStopResponding = false;

				// release the semaphore for the related task
				I2C_MUTEX_UNLOCK();

				// delete the sps30 task
				if (xTaskSps != NULL) vTaskDelete(xTaskSps);

				// create the new SPS30 task
				sps_taskInit();
			}

			// if the DGS task stop responding
			if (isDgsStopResponding) {
				isDgsStopResponding = false;

				// delete the DGS task
				if (xTaskDgs != NULL) vTaskDelete(xTaskDgs);

				// create the new DGS task
				dgs_taskInit();
			}

			// if the GPS task stop responding
			if (isGpsStopResponding) {
				isGpsStopResponding = false;

				// delete the gps task
				if (xTaskGps != NULL) vTaskDelete(xTaskGps);

				// create the new gps task
				gps_taskInit();
			}

			// if the display task stop responding
			if (isDisplayStopResponding) {
				isDisplayStopResponding = false;

				// release the semaphore for the related task
				I2C_MUTEX_UNLOCK();

				// delete the gps task
				if (xTaskDisplay != NULL) vTaskDelete(xTaskDisplay);

				// create the new gps task
				display_taskInit();
			}
		}
	}
}

void watchdog_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(watchdogTask, "watchdogTask", ulStackDepth, (void*)1, uxPriority, &xTaskWatchdog, xCoreID);
}

