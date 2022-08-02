// 
// 
// 

#include "no2_sensor.h"

typedef __PACKED_PRE struct DGS_READ_PACKET_Ttag {
	int32_t lPpb;
	int32_t lTempC;
	int32_t lRh;
	int32_t lPpbRaw;
	int32_t lTempRaw;
	int32_t lRhRaw;
} __PACKED_POST DGS_READ_PACKET_T;

DGS_CONFIG_T dgs_config = { 
	.debug			= false,
	.ser_port_nr	= DGS_SERIAL_PORT_NR,
	.baud			= 9600,
	.ser_config		= SERIAL_8N1,
	.rxPin			= 34,
	.txPin			= 25,
	.sendIntervalMs = DGS_SEND_INTERVAL_MS
};

HardwareSerial dgsSerial(dgs_config.ser_port_nr);
DGS dgs(&dgsSerial);
static HardwareSerial *debugSerial;

DGS_READ_PACKET_T tDgsReadPacket;
DGS_PACKET_T tDgsTxPacket;
MESSAGE_BUFFER_T tDgsTxDisplayPacket;

uint32_t dgsTime = 0;
uint32_t dgsWarmUpCounter = 0;
bool isDgsWarmUpFinished = false;


TaskHandle_t xTaskDgs;
QueueHandle_t xQueueDgs;

void dgs_debug(bool debug, bool libDebug, HardwareSerial &port) {
	dgs_config.debug = debug;
	dgs.DEBUG = libDebug;
	debugSerial = &port;
}

esp_err_t dgs_init(bool debug, bool libDebug) {

	dgs_debug(debug, libDebug);

	if (xQueueDgs == NULL) xQueueDgs = xQueueCreate(SEND_QUEUE_SIZE, sizeof(tDgsTxPacket.tPacket));

	if (xQueueDgs == 0) {
		log_e("Could not create DGS packet send queue. Aborting..");
		return ESP_FAIL;
	}

	if(dgs_config.debug) log_i("DGS send queue created, size %d Bytes", SEND_QUEUE_SIZE * sizeof(tDgsTxPacket.tPacket));

	dgsSerial.begin(dgs_config.baud, dgs_config.ser_config, dgs_config.rxPin, dgs_config.txPin);

	if (dgs_config.debug) {
		debugSerial->printf("Read sensor information\n");
		dgs.getFW();
		//delay(100);
		dgs.getEEPROM();
		//delay(100);
		debugSerial->println();
	}

	return ESP_OK;
}

esp_err_t dgs_readAll() {
	
	if (dgs.getData('\r')) {
		tDgsReadPacket.lPpb = dgs.getConc('p');		//default is 'p' for temperature compensated ppb, any other character for raw counts
		tDgsReadPacket.lTempC = dgs.getTemp('C');		//'F' or 'C' for units of temperature, default is 'C', any other character for raw counts
		tDgsReadPacket.lRh = dgs.getRh('r');			//default is 'r' for %Rh, any other character for raw counts
		tDgsReadPacket.lPpbRaw = dgs.getConc(0);		//default is 'p' for temperature compensated ppb, any other character for raw counts
		tDgsReadPacket.lTempRaw = dgs.getTemp(0);		//'F' or 'C' for units of temperature, default is 'C', any other character for raw counts
		tDgsReadPacket.lRhRaw = dgs.getRh(0);			//default is 'r' for %Rh, any other character for raw counts

		if (dgs_config.debug) {
			debugSerial->printf("Conc: %d, Tavg: %d, Havg: %d, Craw: %d, Traw: %d, Hraw: %d \n",
				tDgsReadPacket.lPpb,
				tDgsReadPacket.lTempC,
				tDgsReadPacket.lRh,
				tDgsReadPacket.lPpbRaw,
				tDgsReadPacket.lTempRaw,
				tDgsReadPacket.lRhRaw);
		}

		// copy the needed data to be sent
		tDgsTxPacket.tPacket.sPpb = (int16_t)tDgsReadPacket.lPpb;
		tDgsTxPacket.tPacket.usPpbRaw = (uint16_t)tDgsReadPacket.lPpbRaw;

		return ESP_OK;
	}

	else return ESP_FAIL;

}

void dgs_resetModule() {
	dgs.resetModule();
}

void dgs_enableSleep(bool enable) {

	if (enable) {
		dgsSerial.write('s');	// put the sensor in standby mode
	}

	else {
		dgsSerial.write('\r');	// wake the sensor from the standby mode
	}

	// flush the RX serial buffer
	while (dgsSerial.available()) dgsSerial.read();

}

void dgs_cancelWarmUp() {
	isDgsWarmUpFinished = true;
}

void dgsTask(void *parameter) {
	
	dgs_debug(false);

	debugSerial->printf("Start DGS task!\n");
	
	esp_err_t lRet = ESP_OK;

	while (ESP_OK != (lRet = dgs_init(DGS_DEBUG, DGS_LIBDEBUG))) {
		// wait until dgs init is finished
	}

	dgsTime = millis();
	dgsWarmUpCounter = millis();

	for (;;) {

		if (millis() - dgsTime >= dgs_config.sendIntervalMs) {

			if (ESP_OK != (lRet = dgs_readAll())) {
				// do when failed to read dgs data
			}
			else {
				
				tDgsTxDisplayPacket.bPort = DGS_DATAPORT;
				tDgsTxDisplayPacket.bMessageSize = sizeof(tDgsTxPacket.tPacket);
				memcpy(tDgsTxDisplayPacket.abPacket, tDgsTxPacket.abPacket, tDgsTxDisplayPacket.bMessageSize);

				if (xQueueDisplay != NULL && xQueueSendToBack(xQueueDisplay, &tDgsTxDisplayPacket, 100 / portTICK_RATE_MS) == pdTRUE) {

				}

				if (millis() - dgsWarmUpCounter < 3600000 && !isDgsWarmUpFinished) {
					tDgsTxPacket.tPacket.sPpb = -1;
					tDgsTxPacket.tPacket.usPpbRaw = 0;
				}

				else {
					isDgsWarmUpFinished = true;
				}

				if (xQueueSendToBack(xQueueDgs, &tDgsTxPacket.tPacket, 500 / portTICK_RATE_MS) != pdTRUE) {
					log_e("Fail to send bme data in queue");
				}

				else {
					if (DEBUG_SEND_PACKET) {
						debugSerial->printf("DGS TX Packet\t->\t%-d\t%-d\n\n",
							tDgsTxPacket.tPacket.sPpb,
							tDgsTxPacket.tPacket.usPpbRaw
						);
					}
					dgsTime = millis();
				}

				if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, dgsWdEventId);

			}
		}

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}
void dgs_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(dgsTask, "dgsTask", ulStackDepth, (void*)1, uxPriority, &xTaskDgs, xCoreID);
}

