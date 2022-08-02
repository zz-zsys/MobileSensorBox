// 
// 
// 

#include "pmsensor.h"

//typedef  __PACKED_PRE struct SPS_READ_PACKET_Ttag {
//	float   MassPM1;        // Mass Concentration PM1.0 [μg/m3]
//	float   MassPM2;        // Mass Concentration PM2.5 [μg/m3]
//	float   MassPM4;        // Mass Concentration PM4.0 [μg/m3]
//	float   MassPM10;       // Mass Concentration PM10 [μg/m3]
//	float   NumPM0;         // Number Concentration PM0.5 [#/cm3]
//	float   NumPM1;         // Number Concentration PM1.0 [#/cm3]
//	float   NumPM2;         // Number Concentration PM2.5 [#/cm3]
//	float   NumPM4;         // Number Concentration PM4.0 [#/cm3]
//	float   NumPM10;        // Number Concentration PM4.0 [#/cm3]
//	float   PartSize;       // Typical Particle Size [μm]
//} __PACKED_POST SPS_READ_PACKET_T;

SPS30_CONFIG_T sps_config = {
	.debug			= 0,
	.ser_com		= I2C_COMMS,
	.sdapin			= SDA,
	.sclpin			= SCL,
	.freq			= 100000,
	.sendIntervalMs = SPS30_SEND_INTERVAL_MS 
};

SPS30 sps30;


SPS_PACKET_T tSpsTxPacket;
MESSAGE_BUFFER_T tSpsTxDisplayPacket;

static HardwareSerial *debugSerial;

TaskHandle_t xTaskSps;
QueueHandle_t xQueueSps;

uint32_t spsTime = 0;
uint32_t ulGetCleanInterval = 0;
uint32_t ulSetCleanInterval = 604800;

// add this error def for backward compatibility
#define SPS30_ERR_NOT_AVAIL	0x52


void sps_debug(uint8_t debug, uint8_t libDebug, HardwareSerial &port) {
	sps_config.debug = debug;
	sps30.EnableDebugging(libDebug);
	debugSerial = &port;
}



void sps_errToMess(const char *mess, uint8_t r) {
	char buf[80];

	if (r) {
		sps30.GetErrDescription(r, buf, 80);
		log_e("%s: %s", mess, buf);
	}

	else {
		log_e("%s", mess);
	}
}

void sps_errorLoop(const char *mess, uint8_t r)
{
	if (r) sps_errToMess(mess, r);
	else log_e("%s", mess);

	log_e("%s", "Program on hold");
	
	SPS30_MUTEX_UNLOCK();

	for (;;) {
		delay(100000);
	}
}

esp_err_t sps_init(uint8_t debug, uint8_t libDebug, HardwareSerial &debugPort)
{
	uint8_t bRet = SPS30_ERR_OK;

	sps_debug(debug, libDebug, debugPort);

	if (SPS30_MUTEX_LOCK()) {

		if (xQueueSps == NULL) xQueueSps = xQueueCreate(SEND_QUEUE_SIZE, sizeof(tSpsTxPacket.tPacket));

		if (xQueueSps == 0) {
			log_e("Could not create sps packet send queue. Aborting..");
			return ESP_FAIL;
		}

		if(sps_config.debug) log_i("SPS send queue created, size %d Bytes", SEND_QUEUE_SIZE * sizeof(tSpsTxPacket.tPacket));

		//! Begin communication channel
		if (sps30.begin(&Wire) == false) {
			sps_errorLoop("could not initialize communication channel.", 0);
		}
		else {
			if (sps_config.debug) log_i("SPS30 communication channel initialised");
		}

		//! Check for SPS30 connection
		if (sps30.probe() == false) {
			sps_errorLoop("could not probe / connect with SPS30.", 0);
		}
		else
			debugSerial->printf("Detected SPS30\n");

		while (SPS30_ERR_OK != (bRet = sps30.GetAutoCleanInt(&ulGetCleanInterval))) {
			log_e("Failed to get clean interval");
			delay(1000);
		}

		debugSerial->printf("Get clean interval: %d s\n", ulGetCleanInterval);

		// start measurement
		if (sps30.start() == false) {
			log_e("Failed to start measurement");
			sps_errorLoop("Could NOT start measurement", 0);
		}
		else {
			debugSerial->printf("Start SPS30 measurement\n");
		}

#if AUTOSTART_FAN_CLEANING > 0
		if (!isTimeSet) {
			// start fan cleaning
			if (sps30.clean() == false) {
				log_e("Failed to start fan-cleaning");
				sps_errorLoop("Could not start fan cleaning", 0);
			}
			else {
				debugSerial->printf("Start SPS30 fan cleaning\n");
				vTaskDelay(2000 / portTICK_PERIOD_MS);
			}
		}
#endif

		if (sps_config.ser_com == I2C_COMMS) {
			if (sps30.I2C_expect() == 4) log_w("I2C buffersize limitation, only the MASS concentration are available");
		}

		SPS30_MUTEX_UNLOCK();

		return SPS30_ERR_OK;
	}

	else return ESP_FAIL;

}

void sps_getDeviceInfo() {
	char buf[32];
	uint8_t ret;

	//try to read serial number
	if (SPS30_ERR_OK != (ret = sps30.GetSerialNumber(buf, 32))) {
		sps_errToMess("could not get serial number", ret);
	}
	else {
		if (strlen(buf) > 0) {
			log_i("Serial number: %s", buf);
		}
		else log_i("Serial number is not available");
	}

	// try to get product name
	if (SPS30_ERR_OK != (ret = sps30.GetProductName(buf, 32))) {
		sps_errToMess("could not get product name", ret);
	}
	else {
		if (strlen(buf) > 0) {
			log_i("Product name: %s", buf);
		}
		else log_i("Product name is not available");
	}

	// try to get article code
	if (SPS30_ERR_OK != (ret = sps30.GetArticleCode(buf, 32))) {
		sps_errToMess("could not get article code", ret);
	}
	else {
		if (strlen(buf) > 0) {
			log_i("Article code: %s", buf);
		}
		else log_i("Article code is not available");
	}
}

uint8_t sps_readAll() {

	if (SPS30_MUTEX_LOCK()) {
		static bool header = true;
		uint8_t ret = SPS30_ERR_OK;
		struct sps_values spsVal = { 0 };

		if (SPS30_ERR_OK != (ret = sps30.GetValues(&spsVal))) {
			// data might not have been ready
			if (ret == SPS30_ERR_DATALENGTH) {
				sps_errToMess("Error during reading values: ", ret);
			}
		}

		else {
			//memcpy(&tSpsTxPacket, &spsVal, sizeof(spsVal));

			// copy all the needed data
			tSpsTxPacket.tPacket.MassPM2 = spsVal.MassPM2;
			tSpsTxPacket.tPacket.MassPM10 = spsVal.MassPM10;
			tSpsTxPacket.tPacket.usAvgParticleSize_x1000 = (uint16_t)(spsVal.PartSize*1000);

			if (sps_config.debug) {
				//if (header) {
				//	debugSerial->println(F("-------------Mass -----------\t\t------------- Number --------------\t\t\t-Average-"));
				//	debugSerial->println(F("     Concentration [μg/m3]\t\tConcentration [#/cm3]\t\t\t[μm]"));
				//	debugSerial->println(F("P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
				//	header = false;
				//}

				debugSerial->printf("%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\t%-2.4f\n",
					spsVal.MassPM1,
					spsVal.MassPM2,
					spsVal.MassPM4,
					spsVal.MassPM10,
					spsVal.NumPM0,
					spsVal.NumPM1,
					spsVal.NumPM2,
					spsVal.NumPM4,
					spsVal.NumPM10,
					spsVal.PartSize
				);
			}
		}

		SPS30_MUTEX_UNLOCK();

		return ret;
	}

	else return SPS30_ERR_NOT_AVAIL;

}

uint8_t sps_stopMeasurement() {
	uint8_t ret = 0;
	uint8_t stopCmd[2] = { 0x01, 0x04 };
	Wire.begin(sps_config.sdapin, sps_config.sclpin, sps_config.freq);
	debugSerial->printf("Init wire..");
	Wire.beginTransmission(SPS30_ADDRESS);
	debugSerial->printf("Begin transmission..");
	Wire.write(stopCmd, 2);
	debugSerial->printf("Write stop cmd..");
	ret = Wire.endTransmission();
	debugSerial->printf("End transmission\n");
	debugSerial->printf("SPS30 stop measurement ret: %d\n", ret);
	return ret;
}

esp_err_t sps_enableSleep(bool enable) {
	
	if (xSemaphoreTake(xSemaphoreI2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
		if (enable) {
			debugSerial->printf("Stopping the SPS30 measurement\n");

			if (sps30.stop()) {
				debugSerial->printf("SPS30 measurement is stopped\n");
				SPS30_MUTEX_UNLOCK();
				return SPS30_ERR_OK;
			}
			else {
				debugSerial->printf("SPS30 measurement is failed to stop!\n");
				SPS30_MUTEX_UNLOCK();
				return ESP_FAIL;
			}
		}

		else {
			if (sps30.start()) {
				debugSerial->printf("SPS30 measurement is started!\n");
				SPS30_MUTEX_UNLOCK();
				return SPS30_ERR_OK;
			}
			else {
				//log_e("SPS30 measurement is failed to start!");
				SPS30_MUTEX_UNLOCK();
				return ESP_FAIL;
			}
		}
	}

	else {
		log_e("Failed to take i2c mutex semaphore");
		return ESP_FAIL;
	}
}

void spsTask(void * parameter) {

	sps_debug(false);

	debugSerial->printf("Start SPS task!\n");

	esp_err_t lRet = SPS30_ERR_OK;

	while (SPS30_ERR_OK != (lRet = sps_init(SPS30_DEBUG, SPS30_LIBDEBUG))) {
		// wait until sps30 init is finished
	}

	spsTime = millis();

	for (;;) {

		if (millis() - spsTime > sps_config.sendIntervalMs) {

			memset(&tSpsTxPacket, 0x00, sizeof(tSpsTxPacket.tPacket));

			//! read the SPS30 incoming data
			if (SPS30_ERR_OK != (lRet = sps_readAll())) {
				// error when read the data
				log_e("Error SPS read: 0x%.2X", lRet);

				if (lRet == 0x52) {
					do {} while (xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) != pdTRUE);
					sps30.begin(&Wire);
					xSemaphoreGive(xSemaphoreI2C);
				}
			}
			else {

				tSpsTxDisplayPacket.bPort = SPS_DATAPORT;
				tSpsTxDisplayPacket.bMessageSize = sizeof(tSpsTxPacket.tPacket);
				memcpy(tSpsTxDisplayPacket.abPacket, tSpsTxPacket.abPacket, tSpsTxDisplayPacket.bMessageSize);

				if (xQueueDisplay != NULL && xQueueSendToBack(xQueueDisplay, &tSpsTxDisplayPacket, 100 / portTICK_RATE_MS) == pdTRUE) {

				}

				if (xQueueSendToBack(xQueueSps, &tSpsTxPacket.abPacket, 500 / portTICK_RATE_MS) != pdTRUE) {
					log_e("Failed to send sps data in queue");
				}

				else {
					if (DEBUG_SEND_PACKET) {
						debugSerial->printf("SPS30 TX Packet\t->\t%-2.5f\t%-2.5f\t%-2.5f\t\n\n",
							tSpsTxPacket.tPacket.MassPM2,
							tSpsTxPacket.tPacket.MassPM10,
							tSpsTxPacket.tPacket.usAvgParticleSize_x1000 / 1000.0
						);
					}

					spsTime = millis();
				}

				if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, spsWdEventId);

			}
		}

		vTaskDelay(100 / portTICK_RATE_MS);

	}
}

void sps_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(spsTask, "spsTask", ulStackDepth, (void*)1, uxPriority, &xTaskSps, xCoreID);
}