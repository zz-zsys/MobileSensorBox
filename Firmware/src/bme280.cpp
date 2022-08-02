#include "bme280.h"

BME280 bme;										// BME class instance 

BME_CONFIG_T bme_config = {						// BME config struct initialization
	.debug			= false,					// debug flag
	// .sdapin			= SDA,						// SDA pin
	// .sclpin			= SCL,						// SCL pin
	// .freq			= 100000,					// I2C frequency 
	.addr			= BME_I2C_ADDRESS,			// I2C address of the sensor
	.sendIntervalMs = BME_SEND_INTERVAL_MS,		// send packet frequency in ms
	.tempOffset		= 4.0						// temperature offset
};

static HardwareSerial *debugSerial;				// pointer to the debug serial port

BME_PACKET_T tBmeTxPacket;						// BME send lora packet instance
MESSAGE_BUFFER_T tBmeTxDisplayPacket;			// BME send display packet instance
struct {										// local packet for BME data acquisition 
	float temp;									// temperature 
	float hum;									// humidity
	float pressure;								// pressure
} bmePacket;

TaskHandle_t xTaskBme;							// task handle for bme task
QueueHandle_t xQueueBme;						// queue handle for bme task

static bool header = false;						// header flag for debugging purpose
uint32_t bmeTaskTime = 0;						// time counter for bme task



/************************************************************************************************************************/
/*!
* @brief		configure the debugging mode for the bme task
* @param[in]	debug	debug flag (true if enable, false if disable)
* @param[in]	port	serial port for debugging purpose
* @retval		none
*/
/************************************************************************************************************************/
void bme_debug(bool debug, HardwareSerial &port) {
	bme_config.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		initialize the BME sensor
* @param[in]	debug		debug flag (true to enable, false (default) to disable)
* @param[in]	debugPort	pointer to the debugger serial port
* @retval		ESP_OK if success, ESP_FAIL if failed
*/
/************************************************************************************************************************/
esp_err_t bme_init(bool debug, HardwareSerial &debugPort) {

	// set the debugging mode
	bme_debug(debug, debugPort);

	// check if the i2c semaphore can be obtained 
	if (BME_MUTEX_LOCK()) { // if success to obtain..

		// create the bme queue instance
		if (xQueueBme == NULL) xQueueBme = xQueueCreate(SEND_QUEUE_SIZE, sizeof(tBmeTxPacket.tPacket));

		if (xQueueBme == 0) {
			log_e("Could not create bme packet send queue. Aborting..");
			return ESP_FAIL;
		}

		if(bme_config.debug) log_i("BME send queue created, size %d Bytes", SEND_QUEUE_SIZE * sizeof(tBmeTxPacket.tPacket));

		// initialize the i2c bus
		// Wire.begin(bme_config.sdapin, bme_config.sclpin, bme_config.freq);
		
		// set the i2c address of the bme sensor
		bme.setI2CAddress(bme_config.addr);

		// begin communication over i2c
		if (bme.beginI2C(Wire) == false)
		{
			log_e("The sensor did not respond. Please check wiring.\n");
			while (1); //Freeze
		}
		else {
			if (bme_config.debug) log_i("BME280 is connected\n");
		}

		// set filter coeff to 16
		bme.setFilter(4);

		// set temperature reading oversampling
		bme.setTempOverSample(16);

		// set the humidity reading oversampling
		bme.setHumidityOverSample(4);

		// Release the i2c semaphore
		BME_MUTEX_UNLOCK();

		return ESP_OK;
	}

	return ESP_FAIL;
}

/************************************************************************************************************************/
/*!
* @brief		read the temperature, humidity and pressure data from the sensor
* @param[in]	sTemp_x100			pointer to store the temperature value in °C with scaling factor 0.01
* @param[in]	bHum				pointer to store the humidity value in %RH
* @param[in]	usPressure_hPa_x10	pointer to store the pressure value in hPa with scaling factor 0.1
* @retval		ESP_OK if success, ESP_FAIL if failed
*/
/************************************************************************************************************************/
esp_err_t bme_getData(int16_t *sTemp_x100, uint8_t *bHum, uint16_t *usPressure_hPa_x10) {
	
	// check if the i2c semaphore can be obtained
	if (BME_MUTEX_LOCK()) { // if success to obtain..

		// check if device is taking measurement
		if (!bme.isMeasuring()) {
			// if not measuring, release the semaphore
			BME_MUTEX_UNLOCK();
			return ESP_FAIL;
		}

		else {
			bmePacket.temp = bme.readTempC() - bme_config.tempOffset;				// read the temperature in °C
			bmePacket.hum = bme.readFloatHumidity();		// read the humidity in %
			bmePacket.pressure = bme.readFloatPressure();	// read the pressure in hPa

			*sTemp_x100 = (int16_t)(bmePacket.temp * 100);
			*bHum = (uint8_t)bmePacket.hum;
			*usPressure_hPa_x10 = (uint16_t)(bmePacket.pressure / 10);

			if (!header) {
				if (bme_config.debug) debugSerial->printf("Temp[\xC2\xB0\fC]\tHum[%%]\tPressure[Pa]\n");
				header = true;
			}

			if (bme_config.debug) debugSerial->printf("%.2f\t%.2f\t%.2f\n", bmePacket.temp, bmePacket.hum, bmePacket.pressure);

			// release the semaphore
			BME_MUTEX_UNLOCK();

			return ESP_OK;
		}
	}

	// if semaphore can't be obtained in the given time
	return ESP_FAIL;
}

/************************************************************************************************************************/
/*!
* @brief		read the temperature, humidity and pressure data from the sensor
* @param[in]	sTemp_x100			pointer to store the temperature value in °C with scaling factor 0.01
* @param[in]	bHum				pointer to store the humidity value in %RH
* @param[in]	usPressure_hPa_x10	pointer to store the pressure value in hPa with scaling factor 0.1
* @retval		ESP_OK if success, ESP_FAIL if failed
*/
/************************************************************************************************************************/
esp_err_t bme_enableSleep(bool enable) {
	if (xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) == pdTRUE) {			// if success to obtain..

		if(enable)bme.setMode(MODE_SLEEP);	// set sensor mode to sleep mode
		else bme.setMode(MODE_NORMAL);		// set sensor mode to normal mode

		xSemaphoreGive(xSemaphoreI2C);		// release the semaphore

		return ESP_OK;

	}

	else {
		log_e("Failed to take i2c mutex semaphore");
		return ESP_FAIL;					// else return fail..
	}
}

/************************************************************************************************************************/
/*!
* @brief		BME task entry
* @param[in]	parameter	parameter passed into the task
* @retval		none
*/
/************************************************************************************************************************/
void bmeTask(void * parameter) {
	
	bme_debug(false);

	debugSerial->printf("Start BME task!\n");

	esp_err_t lRet = ESP_OK;

	// initialize the bme sensor
	while (ESP_OK != (lRet = bme_init(BME_DEBUG))) {
		// wait until bme init is finished
	};
	
	// start the task timer counter
	bmeTaskTime = millis();

	for (;;) {

		// check if the temperature, humidity and pressure data can be read from the sensor
		if (ESP_OK != (lRet = bme_getData(&tBmeTxPacket.tPacket.sTemperature_x100, &tBmeTxPacket.tPacket.bHumidity, &tBmeTxPacket.tPacket.usPressure_hPa_x10))) {
			// if failed to read
		}

		else {	// if success to read the data, parse and send the packet

			// check if the task timer counter equal or greater than the configured send packet interval time
			if (millis() - bmeTaskTime >= bme_config.sendIntervalMs) {	// if yes

				// prepare the send packet for the display and lora
				tBmeTxDisplayPacket.bPort = BME_DATAPORT;
				tBmeTxDisplayPacket.bMessageSize = sizeof(tBmeTxPacket.tPacket);
				memcpy(tBmeTxDisplayPacket.abPacket, tBmeTxPacket.abPacket, tBmeTxDisplayPacket.bMessageSize);

				// enqueue the data in the display queue for display purpose
				if (xQueueDisplay != NULL && xQueueSendToBack(xQueueDisplay, &tBmeTxDisplayPacket, 100 / portTICK_RATE_MS) == pdTRUE) {

				}

				// enqueue the data in the bme queue for lora packet
				if (xQueueSendToBack(xQueueBme, &tBmeTxPacket.tPacket, 500 / portTICK_RATE_MS) != pdTRUE) {
					log_e("Fail to send bme data in queue");
				}

				else {
					if (DEBUG_SEND_PACKET) debugSerial->printf("BME TX Packet\t->\t%02.2f\t%d\t%02.1f\n\n", tBmeTxPacket.tPacket.sTemperature_x100 / 100.0, tBmeTxPacket.tPacket.bHumidity, tBmeTxPacket.tPacket.usPressure_hPa_x10 / 10.0);

					// reset all value in the packet to zero
					memset(&tBmeTxPacket, 0x00, sizeof(tBmeTxPacket.tPacket));

					// reset the counter
					bmeTaskTime = millis();
				}
			}

			// set active bit to the watchdog event group
			if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, bmeWdEventId);

		}

		// delay the task for a given time/ticks
		vTaskDelay(500 / portTICK_RATE_MS);

	}
}

/************************************************************************************************************************/
/*!
* @brief		create and initialize the bme task
* @param[in]	ulStackDepth	size of the task stack in bytes	
* @param[in]	uxPriority		task priority
* @param[in]	xCoreID			pinned core where the task will be run
* @retval		none
*/
/************************************************************************************************************************/
void bme_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(bmeTask, "bmeTask", ulStackDepth, (void*)1, uxPriority, &xTaskBme, xCoreID);
}

