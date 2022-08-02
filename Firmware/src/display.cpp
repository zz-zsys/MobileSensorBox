#include <Wire.h>
#include "display.h"


DISPLAY_CONFIG_T disp_config = {
	.debug				= false,
	.sda_pin			= SDA,
	.scl_pin			= SCL,
	.freq				= 100000,
	.addr				= 0x3C,
	.refreshTimeInMs	= DISPLAY_REFRESH_TIME_MS 
};

static HardwareSerial* debugSerial;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, disp_config.scl_pin, disp_config.sda_pin);	// U8G2 class instance for the given display


TaskHandle_t xTaskDisplay;								// task handle for the display task
QueueHandle_t xQueueDisplay;							// queue handle for the display task

static BME_PACKET_T tBmeRxPacket = { 0 };				// bme received packet
static SPS_PACKET_T tSpsRxPacket = { 0 };				// sps30 received packet
static DGS_PACKET_T tDgsRxPacket = { 0 };				// dgs no2 received packet
static GPS_PACKET_T tGpsRxPacket = { 0 };				// gps received packet

MESSAGE_BUFFER_T tDisplayRxPacket;						// received packet from display queue

uint32_t dispTime = 0;									// time counter for display task
uint32_t dispCheckConnectionTime = 0;					// check display connection time counter;

struct tm *tmInfo;										// pointer to store the localtime data in struct tm format

bool isDisplayConnected = false;

bool disp_isConnected() {
	do {} while (xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) != pdTRUE);
	uint8_t error = 0;
	Wire.beginTransmission(disp_config.addr);
	error = Wire.endTransmission();
	xSemaphoreGive(xSemaphoreI2C);

	if (!error) {
		isDisplayConnected = true;
		return true;
	}
	else {
		log_d("display check connection error: %d", error);
		return false;
	}
}

void disp_debug(bool debug, HardwareSerial &port) {
	disp_config.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		initialize the display
* @retval		ESP_OK if success, ESP_FAIL if failed
*/
/************************************************************************************************************************/
esp_err_t disp_init() {
	// check if the i2c semaphore can be obtained
	if (DISPLAY_MUTEX_LOCK()) {	// if success to obtained

		// create the display queue
		if (xQueueDisplay == NULL) xQueueDisplay = xQueueCreate(SEND_QUEUE_SIZE, sizeof(tDisplayRxPacket));

		if (xQueueDisplay == 0) {
			log_e("Could not create display packet received queue. Aborting..");
			return ESP_FAIL;
		}
		
		if(disp_config.debug) log_i("Display packet received queue created, size %d Bytes", SEND_QUEUE_SIZE * sizeof(tDisplayRxPacket));

		// set the i2c bus clock to 100 kHz, default was 400 kHz that could lead the communication problem with SPS30
		u8g2.setBusClock(disp_config.freq);

		// initialize the display
		if (!u8g2.begin()) {
			return ESP_FAIL;
		}

		if (disp_config.debug) log_i("display i2c bus freq: %d", u8g2.getBusClock());

		// enable the UTF8Print for special case printing
		u8g2.enableUTF8Print();

		// release the semaphore
		DISPLAY_MUTEX_UNLOCK();

		return ESP_OK;
	}
	else return ESP_FAIL;
};

esp_err_t disp_sleep() {
	if (DISPLAY_MUTEX_LOCK()) {
		if(isDisplayConnected) u8g2.noDisplay();
		DISPLAY_MUTEX_UNLOCK();
		return ESP_OK;
	}
	else {
		log_e("Failed to take i2c mutex semaphore");
		return ESP_FAIL;
	}
}

/************************************************************************************************************************/
/*!
* @brief		frame template for the display
* @param[in]	tmInfo					localtime data in struct tm format			
* @param[in]	temp_x100				temperature in °C (scale factor 0.01)
* @param[in]	hum						humidity data
* @param[in]	ppb						no2 ppb value
* @param[in]	lat						latitude
* @param[in]	lon						longitude
* @param[in]	pm2_5					mass concentration of PM2.5
* @param[in]	pm10_0					mass concentration of PM10
* @param[in]	avgParticleSize_x1000	average particle size in µm (scale factor 0.001)
* @retval		none
*/
/************************************************************************************************************************/
void disp_frame(struct tm *timeInfo, int16_t *temp_x100, uint8_t *hum, int16_t *ppb, float *lat, float *lon, 
				float *pm2_5, float *pm10_0, uint16_t *avgParticleSize_x1000) {

	u8g2.clearBuffer();											// clear the internal memory
	u8g2.setFont(u8g2_font_timR08_tf);							// choose a suitable font
	u8g2.setCursor(0, 8);
	u8g2.printf("%02d-%02d-%02d", timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday);	/** date */
	u8g2.setCursor(82, 8);
	u8g2.printf("%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);				/** time */

	u8g2.drawHLine(0, 10, u8g2.getDisplayWidth());

	u8g2.setCursor(0, 20);
	u8g2.printf("T:%02.1f\xb0\fC", *temp_x100 / 100.0);
	u8g2.setCursor(48, 20);
	u8g2.printf("H:%02d%%", *hum);
	u8g2.setCursor(82, 20);
	u8g2.printf("ppb:%d", *ppb);

	u8g2.setCursor(0, 32);
	u8g2.printf("lat: %.5f", *lat);
	u8g2.setCursor(62, 32);
	u8g2.printf("lon: %.5f", *lon);

	u8g2.setCursor(0, 44);
	u8g2.printf("avgSize:");
	u8g2.setCursor(62, 44);
	u8g2.printf("PM2.5 :%.5f", *pm2_5);

	u8g2.setCursor(0, 56);
	u8g2.printf("%.3f", *avgParticleSize_x1000 / 1000.0);
	u8g2.setCursor(62, 56);
	u8g2.printf("PM10  :%.5f", *pm10_0);

	u8g2.sendBuffer();											// transfer internal memory to the display

}



/************************************************************************************************************************/
/*!
* @brief		Display task entry
* @param[in]	parameter	parameter passed into the task
* @retval		none
*/
/************************************************************************************************************************/
void displayTask(void * parameter) {

	disp_debug(false);

	debugSerial->printf("Start display task!\n");

	esp_err_t lRet = ESP_OK;

	disp_debug(DISPLAY_DEBUG);

	do { } while (xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) != pdTRUE);
	Wire.begin(disp_config.sda_pin, disp_config.scl_pin, disp_config.freq);
	xSemaphoreGive(xSemaphoreI2C);

	// check if display is connected on this address
	while (!disp_isConnected()) {	

		debugSerial->printf("No display connected...\n\n");

		// set active bit to the watchdog event group
		if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, displayWdEventId);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	// wait until the display finished to initiliaze
	while (ESP_OK != (lRet = disp_init())) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	};
	
	// start the display timer counter
	dispTime = millis();
	dispCheckConnectionTime = millis();

	for (;;) {

		// check if there is incoming packet in queue
		if (xQueueDisplay != NULL && xQueueReceive(xQueueDisplay, &tDisplayRxPacket, 100 / portTICK_RATE_MS) == pdTRUE) {

			// check the port in the received packet, copy the data to the responsible packet
			switch (tDisplayRxPacket.bPort) {
				case BME_DATAPORT: {
					memcpy(tBmeRxPacket.abPacket, tDisplayRxPacket.abPacket, tDisplayRxPacket.bMessageSize);
					break;
				}
				case SPS_DATAPORT: {
					memcpy(tSpsRxPacket.abPacket, tDisplayRxPacket.abPacket, tDisplayRxPacket.bMessageSize);
					break;
				}
				case DGS_DATAPORT: {
					memcpy(tDgsRxPacket.abPacket, tDisplayRxPacket.abPacket, tDisplayRxPacket.bMessageSize);
					break;
				}
				case GPS_DATAPORT: {
					memcpy(tGpsRxPacket.abPacket, tDisplayRxPacket.abPacket, tDisplayRxPacket.bMessageSize);
					break;
				}
				default: {
					break;
				}
			}
		}

		// check if the task timer counter equal or greater than the configured refresh time
		if (millis() - dispTime > disp_config.refreshTimeInMs) {

			// check if the i2c semaphore can be obtained
			if (DISPLAY_MUTEX_LOCK()) { // if success to obtain..

				// is system time already set
				if (isTimeSet) { // if yes,
					time(&currentTime);					// obtain the current epoch time
					tmInfo = localtime(&currentTime);	// convert the current time to struct tm format 
				}
				else {
					static time_t currentTime = 0;		// set the current time to zero
					tmInfo = localtime(&currentTime);	// convert the current time to struct tm format 
				}

				// send data packet to display buffer.
				disp_frame(	
					tmInfo,
					&tBmeRxPacket.tPacket.sTemperature_x100,
					&tBmeRxPacket.tPacket.bHumidity,
					&tDgsRxPacket.tPacket.sPpb,
					&tGpsRxPacket.tPacket.fLatitude,
					&tGpsRxPacket.tPacket.fLongitude,
					&tSpsRxPacket.tPacket.MassPM2,
					&tSpsRxPacket.tPacket.MassPM10,
					&tSpsRxPacket.tPacket.usAvgParticleSize_x1000
				);

				// reset the task timer counter
				dispTime = millis();

				// release the semaphore
				DISPLAY_MUTEX_UNLOCK();
			}
		}

		if (millis() - dispCheckConnectionTime >= 3000) {
			if (disp_isConnected()) {
				// set active bit to the watchdog event group
				if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, displayWdEventId);
			}
			else {
				debugSerial->printf("No display connected...\n\n");
			}

			dispCheckConnectionTime = millis();
		}

		// delay the task for a given time/ticks
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/************************************************************************************************************************/
/*!
* @brief		create and initialize the display task
* @param[in]	ulStackDepth	size of the task stack in bytes
* @param[in]	uxPriority		task priority
* @param[in]	xCoreID			pinned core where the task will be run
* @retval		none
*/
/************************************************************************************************************************/
void display_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(displayTask, "displayTask", ulStackDepth, (void*)1, uxPriority, &xTaskDisplay, xCoreID);
}