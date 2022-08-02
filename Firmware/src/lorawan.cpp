#include "lorawan.h"

#define d2r (M_PI / 180.0)								// degree to radian converter

LORA_CONFIG_T lora_config = {							// Lora config struct initialization
	.debug = false,										// debug flag
	.adrMode = 0,										// data rate adaptation flag
	.checkLinkMode = 0,									// check link mode flag
	.spreadFactor = LORA_SPREAD_FACTOR,					// spreadfactor 6-12
	.txPower = 14,										// tx power in dBm (max 17)
	.ackConfirmed = 0,									// acknownledge flag 
	.sendIntervalMs = LORA_SEND_INTERVAL_MS,			// set sent packet interval in ms
	.minSendIntervalMs = LORA_MIN_SEND_INTERVAL_MS,		// minimum sent packet interval in ms
	.gpsDistanceThreshold = LORA_GPS_DISTANCE_THRESHOLD	// gps distance threshold
};

//LORA_SESSION_T loraSavedData;

RTC_DATA_ATTR uint32_t netid;
RTC_DATA_ATTR uint32_t devaddr;
RTC_DATA_ATTR uint8_t nwkKey[16];
RTC_DATA_ATTR uint8_t artKey[16];

static HardwareSerial *debugSerial;						// pointer to the debug serial port

LORA_PACKET_T tLoraTxPacket;								// LoRaWAN sent packet instance
LORA_PACKET_T tLoraLastTxPacket;							// LoRaWAN last sent packet instance
BME_PACKET_T tBmeRxPacket;								// BME received packet instance
SPS_PACKET_T tSpsRxPacket;								// SPS30 received packet instance
DGS_PACKET_T tDgsRxPacket;								// DGS NO2 received packet instance
GPS_PACKET_T tGpsRxPacket;								// GPS received packet instance


TaskHandle_t xTaskLora;									// task handle for lora task
QueueHandle_t xQueueLora;								// queue handle for lora task

uint32_t loraTxTime = 0;								// time counter lora task

static bool isBmePacketReceived = false;				// bme packet receiving flag
static bool isSpsPacketReceived = false;				// sps packet receiving flag
static bool isDgsPacketReceived = false;				// dgs packet receiving flag
static bool isGpsPacketReceived = false;				// gps packet receiving flag
static bool isFirstLoraPacketSent = false;				// first lora packet sent flag

// define the new HAL configuration based on the existing HAL class configuration
// due to the non-exisiting esp32 hal configuration
class MyHalConfig_t : public Arduino_LMIC::HalConfiguration_t {

public:
	MyHalConfig_t() {};									// class instance 

	// override the exisiting begin function, as ESP32 use other SPI configuration call function
	virtual void begin(void) override {			
		SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
	}
};

MyHalConfig_t myHalConfig;								// new HAL class instance

// define the lmic_pin for the lora module
const lmic_pinmap lmic_pins = {
	.nss = LORA_CS,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = LORA_RST == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_RST,
	.dio = {LORA_IRQ, LORA_IO1,
			LORA_IO2 == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_IO2},
			// optional: set polarity of rxtx pin.
			.rxtx_rx_active = 0,
			// optional: set RSSI cal for listen-before-talk
			// this value is in dB, and is added to RSSI
			// measured prior to decision.
			// Must include noise guardband! Ignored in US,
			// EU, IN, other markets where LBT is not required.
			.rssi_cal = 0,
			// optional: override LMIC_SPI_FREQ if non-zero
			.spi_freq = 0,
			.pConfig = &myHalConfig
};

/************************************************************************************************************************/
/*!
* @brief		configure the debugging mode for the lora task
* @param[in]	debug	debug flag (true if enable, false if disable)
* @param[in]	port	serial port for debugging purpose
* @retval		none
*/
/************************************************************************************************************************/
void lora_debug(bool debug, HardwareSerial &port) {
	lora_config.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		initialize the Lora module
* @param[in]	debug		debug flag (true to enable, false (default) to disable)
* @param[in]	debugPort	pointer to the debugger serial port
* @retval		ESP_OK if success (at the moment always ESP_OK)
*/
/************************************************************************************************************************/
esp_err_t lora_init(bool debug, HardwareSerial &debugPort) {

	// set the debugging mode (default : debug is disable)
	lora_debug(debug, debugPort);

	// LMIC init
	os_init();

	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Set/ Reset the check link mode
	LMIC_setLinkCheckMode(lora_config.checkLinkMode);

	// This tells LMIC to make the receive windows bigger, in case your clock is
	// faster or slower. This causes the transceiver to be earlier switched on,
	// so consuming more power. You may sharpen (reduce) CLOCK_ERROR_PERCENTAGE
	// in globals.h if you are limited on battery.
	LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR_PROCENTAGE / 100);

	// Set the data rate to Spreading Factor 7.  This is the fastest supported
	// rate for 125 kHz channels, and it minimizes air time and battery power.
	// Set the transmission power to 14 dBi (25 mW).
	lora_switch_SFTX(lora_config.spreadFactor, lora_config.txPower);
	//LMIC_setDrTxpow(DR_SF7, lora_config.txPower);

	

	// Start joining to the lora server via OTAA
	if (!LMIC_startJoining()) {
		log_i("Already joined");
	}

	return ESP_OK; // continue main program
}

/************************************************************************************************************************/
/*!
* @brief		convert the big_endian to low-endian and vice versa
* @param[in]	b	packet/data	array	
* @param[in]	c	size of the packet/data array
* @retval		none
*/
/************************************************************************************************************************/
void lora_reverseBytes(uint8_t *b, size_t c) {
	u1_t i;
	for (i = 0; i < c / 2; i++) {
		unsigned char t = b[i];
		b[i] = b[c - 1 - i];
		b[c - 1 - i] = t;
	}
}

/************************************************************************************************************************/
/*!
* @brief		lmic os callback to obtain the app EUI
* @param[in]	b	packet/data	array
* @param[in]	c	size of the packet/data array
* @retval		none
*/
/************************************************************************************************************************/
void os_getArtEui(u1_t* buf) {
	memcpy_P(buf, APPEUI, 8);
	lora_reverseBytes(buf, 8);
}

/************************************************************************************************************************/
/*!
* @brief		lmic os callback to obtain the dev EUI
* @param[in]	b	packet/data	array
* @param[in]	c	size of the packet/data array
* @retval		none
*/
/************************************************************************************************************************/
void os_getDevEui(u1_t* buf) {
	memcpy_P(buf, DEVEUI, 8);
	lora_reverseBytes(buf, 8);
}

/************************************************************************************************************************/
/*!
* @brief		lmic os callback to obtain the app key
* @param[in]	b	packet/data	array
* @param[in]	c	size of the packet/data array
* @retval		none
*/
/************************************************************************************************************************/
void os_getDevKey(u1_t* buf) {
	memcpy_P(buf, APPKEY, 16);
}


/************************************************************************************************************************/
/*!
* @brief		append key from the array in alphanumerical form
* @param[in]	buf		pointer to buffer that store the key
* @param[in]	name	key name	
* @param[in]	key		key in array
* @param[in]	len		size of the key array
* @param[in]	lsb		true if read from lsb 1st, false if read from msb 1st
* @retval		pointer to the stored key buffer address
*/
/************************************************************************************************************************/

char* appendKey(char *buf, const char *name, const uint8_t *key, size_t len, bool lsb) {
	const uint8_t *p;
	static char keystring[50];
	static char keybyte[4];

	memset(keystring, 0x00, sizeof(keystring) / sizeof(keystring[0]));

	if (len > 25) return NULL;

	for (uint8_t i = 0; i < len; i++) {
		p = lsb ? key + len - i - 1 : key + i;
		if (i == len - 1) sprintf(keybyte, "%02X", *p);
		else sprintf(keybyte, "%02X:", *p);
		strncat(keystring, keybyte, 4);
	}

	log_d("%s: %s", name, keystring);

	memcpy(buf, keystring, sizeof(keystring) / sizeof(keystring[0]));

	return buf;
}

/************************************************************************************************************************/
/*!
* @brief		append key from the array in alphanumerical form
* @param[in]	buf		pointer to buffer that store the key
* @param[in]	name	key name
* @param[in]	key		key in array
* @param[in]	len		size of the key array
* @param[in]	lsb		true if read from lsb 1st, false if read from msb 1st
* @retval		pointer to the stored key buffer address
*/
/************************************************************************************************************************/

char* appendDataInHex(char *buf, const char *name, uint8_t *key, uint8_t begin, size_t len) {
	const uint8_t *p;
	static char keystring[120];
	static char keybyte[4];

	memset(keystring, 0x00, sizeof(keystring) / sizeof(keystring[0]));

	if (len > (sizeof(keystring)/2)) return NULL;

	for (uint8_t i = 0; i < len; i++) {
		p = key + begin + i;
		if (i == len - 1) sprintf(keybyte, "%02X", *p);
		else sprintf(keybyte, "%02X ", *p);
		strncat(keystring, keybyte, 4);
	}

	log_d("%s: %s", name, keystring);

	memcpy(buf, keystring, sizeof(keystring) / sizeof(keystring[0]));

	return buf;
}

void lora_showSessionKeys() {
	char nwkKeyBuff[50];
	char artKeyBuff[50];

	netid = LMIC.netid;
	devaddr = LMIC.devaddr;
	memcpy(nwkKey, LMIC.nwkKey, 16);
	memcpy(artKey, LMIC.artKey, 16);

	log_i("\n%s: %d\n%s: %d\n%s: %s\n%s: %s\n", "netid", netid, "devaddr", devaddr, "nwkKey", appendKey(nwkKeyBuff, "nwkKey", nwkKey, 16, false), "artKey", appendKey(artKeyBuff, "artKey", artKey, 16, false));

}

void lora_switch_SFTX(uint8_t sf, uint8_t tx) {
	if (tx > 17) lora_config.txPower = 17;
	else
		lora_config.txPower = tx;
	switch (sf) {
#if defined(CFG_eu868)
	case 6:
		LMIC_setDrTxpow(DR_SF7B, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
#endif
	case 7:
		LMIC_setDrTxpow(DR_SF7, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
	case 8:
		LMIC_setDrTxpow(DR_SF8, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
	case 9:
		LMIC_setDrTxpow(DR_SF9, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
	case 10:
		LMIC_setDrTxpow(DR_SF10, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
	case 11:
#if defined(CFG_us915)
		LMIC_setDrTxpow(DR_SF11CR, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
#else
		LMIC_setDrTxpow(DR_SF11, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
#endif
	case 12:
#if defined(CFG_us915)
		LMIC_setDrTxpow(DR_SF12CR, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
#else
		LMIC_setDrTxpow(DR_SF12, lora_config.txPower);
		lora_config.spreadFactor = sf;
		break;
#endif
	default:
		break;
	}

	log_i("LoRa SF: %d, TX: %d\n", lora_config.spreadFactor, lora_config.txPower);
}

size_t injectDataToLoraPacket(uint8_t* buf, uint8_t *input, size_t inputSize, bool first) {
	
	static size_t packetSize;

	// accumulated the packet size if not the first packet
	if (first) {
		packetSize = inputSize;
		memcpy(buf, input, inputSize);
	}
	else {
		memcpy(buf + packetSize, input, inputSize);
		packetSize += inputSize;
	}

	return packetSize;
}

/************************************************************************************************************************/
/*!
* @brief		calculate haversine distance for linear distance
* @param[in]	lat1		latitude of the first position
* @param[in]	long1		longitude of the first position
* @param[in]	lat2		latitude of the second position
* @param[in]	long2		longitude of the second position
* @retval		distance in meter
*/
/************************************************************************************************************************/
float gps_calcHaversineInMeter(float lat1, float long1, float lat2, float long2)
{
	float dlong = (long2 - long1) * d2r;
	float dlat = (lat2 - lat1) * d2r;
	float a = pow(sin(dlat / 2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong / 2.0), 2);
	float c = 2 * atan2(sqrt(a), sqrt(1 - a));
	float d = 6367 * c * 1000;

	return d;
}

esp_err_t lora_checkIncomingPacket() {
	if (xQueueBme != NULL && xQueueReceive(xQueueBme, &tBmeRxPacket, 1 / portTICK_RATE_MS) == pdTRUE) {
		//log_i("Incoming BME packet\n");
		isBmePacketReceived = true;
	}

	if (xQueueSps != NULL && xQueueReceive(xQueueSps, &tSpsRxPacket, 1 / portTICK_RATE_MS) == pdTRUE) {
		//log_i("Incoming SPS30 packet\n");
		isSpsPacketReceived = true;
	}

	if (xQueueDgs != NULL && xQueueReceive(xQueueDgs, &tDgsRxPacket, 1 / portTICK_RATE_MS) == pdTRUE) {
		//log_i("Incoming DGS packet\n");
		isDgsPacketReceived = true;
	}

	if (xQueueGps != NULL && xQueueReceive(xQueueGps, &tGpsRxPacket, 1 / portTICK_RATE_MS) == pdTRUE) {
		//log_i("Incoming GPS packet\n");
		isGpsPacketReceived = true;
	}

	if (isBmePacketReceived && isSpsPacketReceived && isDgsPacketReceived && isGpsPacketReceived) {
		
		static size_t packetSize;

		memset(tLoraTxPacket.abPacket, 0x00, sizeof(tLoraTxPacket.abPacket));
		memcpy(tLoraTxPacket.tPacket.tBmePacket.abPacket, tBmeRxPacket.abPacket, sizeof(tBmeRxPacket));
		memcpy(tLoraTxPacket.tPacket.tDgsPacket.abPacket, tDgsRxPacket.abPacket, sizeof(tDgsRxPacket));
		memcpy(tLoraTxPacket.tPacket.tGpsPacket.abPacket, tGpsRxPacket.abPacket, sizeof(tGpsRxPacket));
		memcpy(tLoraTxPacket.tPacket.tSpsPacket.abPacket, tSpsRxPacket.abPacket, sizeof(tSpsRxPacket));
		tLoraTxPacket.tPacket.usBatteryStatus = battery_read_voltage();

		log_d("Packet: %s\n", appendDataInHex(loraData, "loraData", tLoraTxPacket.abPacket, 0, sizeof(tLoraTxPacket.tPacket)));

		isBmePacketReceived = false;
		isSpsPacketReceived = false;
		isDgsPacketReceived = false;
		isGpsPacketReceived = false;

		return ESP_OK;
	}
	else return ESP_FAIL;
}

/************************************************************************************************************************/
/*!
* @brief		enqueue LoRaWAN data packet
* @retval		ESP_OK if finished enqueue data, ESP_FAIL if unfinished
*/
/************************************************************************************************************************/
esp_err_t lora_enqueuedata() {

	static char loraData[120] = "";
	esp_err_t lRet = ESP_OK;

	if (ESP_OK != (lRet = lora_checkIncomingPacket())) {
		return ESP_FAIL;
	}

	else {
		//appendDataInHex(loraData, "loraData", tLoraTxPacket.abPacket, 0, sizeof(tLoraTxPacket.tPacket));
		appendDataInHex(loraData, "loraData", tLoraTxPacket.abPacket, 0, sizeof(tLoraTxPacket.tPacket));

		log_d("loraTxData: %s", loraData);

		return ESP_OK;
	}
}

/************************************************************************************************************************/
/*!
* @brief		send LoRaWAN data packet
* @retval		none
*/
/************************************************************************************************************************/
void lora_send() {

	MESSAGE_BUFFER_T loraTxBuffer;
	char txData[120];
	char *pTxData;

	// check if there is a pending TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		// waiting for LoRa getting ready
		log_e("There is a pending TX/RX job running");
	}
	else {

		//loraTxBuffer.bPort = 1;
		//loraTxBuffer.bMessageSize = sizeof(tLoraTxPacket.abPacket);
		//memcpy(loraTxBuffer.abPacket, tLoraTxPacket.abPacket, sizeof(tLoraTxPacket));
		//pTxData = appendDataInHex(txData, "loraTx", loraTxBuffer.abPacket, 0x00, loraTxBuffer.bMessageSize);

		loraTxBuffer.bPort = 1;
		loraTxBuffer.bMessageSize = sizeof(tLoraTxPacket.abPacket);
		memcpy(loraTxBuffer.abPacket, tLoraTxPacket.abPacket, sizeof(tLoraTxPacket));
		pTxData = appendDataInHex(txData, "loraTx", loraTxBuffer.abPacket, 0x00, loraTxBuffer.bMessageSize);

		if (!LMIC_setTxData2(loraTxBuffer.bPort, loraTxBuffer.abPacket, loraTxBuffer.bMessageSize, lora_config.ackConfirmed)) {
			log_i("%d bytes sent to LoRa", loraTxBuffer.bMessageSize);
			log_i("Send: %s", pTxData);

			// save the last sent data to another packet for further calculation
			//memcpy(tLoraLastTxPacket.abPacket, tLoraTxPacket.abPacket, sizeof(tLoraTxPacket));
			memcpy(tLoraLastTxPacket.abPacket, tLoraTxPacket.abPacket, sizeof(tLoraTxPacket));

			if (!isFirstLoraPacketSent) {
				isFirstLoraPacketSent = true;
			}
		}
		else {
			log_e("Could not sent %d bytes to LoRa", loraTxBuffer.bMessageSize);
		}
	}
}

void lora_enableSleep(bool enable) {
	if(enable) LMIC_shutdown();
	else {
		LMIC_reset();
		LMIC_setSession(netid, devaddr, nwkKey, artKey);
	}
}

void onEvent(ev_t ev) {
	char buff[50];
	char data[120];
	char *pData = NULL;
	os_getTime();
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		strcpy_P(buff, PSTR("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		strcpy_P(buff, PSTR("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		strcpy_P(buff, PSTR("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		strcpy_P(buff, PSTR("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		strcpy_P(buff, PSTR("EV_JOINING"));
		break;
	case EV_JOINED: {
		strcpy_P(buff, PSTR("EV_JOINED"));

		// disable data rate adaptation
		LMIC_setAdrMode(lora_config.adrMode);

		// set cyclic lmic link check to off if no ADR because is not supported by ttn (but enabled by lmic after join)
		LMIC_setLinkCheckMode(lora_config.checkLinkMode);

		lora_switch_SFTX(lora_config.spreadFactor, lora_config.txPower);

		lora_showSessionKeys();

		loraTxTime = millis();

		break;
	}
	case EV_JOIN_FAILED:
		strcpy_P(buff, PSTR("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		strcpy_P(buff, PSTR("EV_REJOIN_FAILED"));
		break;
	case EV_TXCOMPLETE:
	{
		strcpy_P(buff, (LMIC.txrxFlags & TXRX_ACK) ? PSTR("RECEIVED ACK") : PSTR("TX COMPLETE"));

		if (LMIC.dataLen) {
			log_i("Downlink data is available!");
			pData = appendDataInHex(data, "loraRxData", LMIC.frame, LMIC.dataBeg, LMIC.dataLen);
		}
		break;
	}
	case EV_LOST_TSYNC:
		strcpy_P(buff, PSTR("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		strcpy_P(buff, PSTR("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		strcpy_P(buff, PSTR("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		strcpy_P(buff, PSTR("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		strcpy_P(buff, PSTR("EV_LINK_ALIVE"));
		break;
	case EV_TXSTART:
		strcpy_P(buff, PSTR("EV_TXSTART"));
		loraTxTime = millis();
		break;
	default:
		sprintf_P(buff, PSTR("LMIC EV %d"), ev);
		break;
	}

	log_i("%s\n", buff);
	
	if (pData != NULL) {
		log_i("Received %d bytes of payload from port %d : %s\n", LMIC.dataLen, LMIC.frame[LMIC.dataBeg - 1], data);
	}

	

}

void loraTask(void * parameter) {


	lora_debug(false);

	debugSerial->printf("Start LoRaWAN task!\n");

	int32_t lRet = ESP_OK;

	while (ESP_OK != (lRet = lora_init(LORAWAN_DEBUG))) {
		// wait until lora init is finished
	};

	loraTxTime = millis();

	for (;;) {

		os_runloop_once();


		if (LMIC.devaddr != 0) {

			if (ESP_OK != (lRet = lora_enqueuedata())) {
				// do when enqueue is failed.
			}

			else {

				// condition to make sure there is no violation happen. With 44 bytes (inc. header), at duty cycle 1%,
				// the time between packet subsequent starts is around 9.2416 s 
				if (millis() - loraTxTime >= lora_config.minSendIntervalMs) {

					if (millis() - loraTxTime >= lora_config.sendIntervalMs) {
						lora_send();
						loraTxTime = millis();
					}
					else {
						if (isFirstLoraPacketSent) {
							//float fDistance = gps_calcHaversineInMeter(
							//	tLoraLastTxPacket.tPacket.tLatitude.fValue,
							//	tLoraLastTxPacket.tPacket.tLongitude.fValue,
							//	tLoraTxPacket.tPacket.tLatitude.fValue,
							//	tLoraTxPacket.tPacket.tLongitude.fValue
							//);

							float fDistance = gps_calcHaversineInMeter(
								tLoraLastTxPacket.tPacket.tGpsPacket.tPacket.fLatitude,
								tLoraLastTxPacket.tPacket.tGpsPacket.tPacket.fLongitude,
								tLoraTxPacket.tPacket.tGpsPacket.tPacket.fLatitude,
								tLoraTxPacket.tPacket.tGpsPacket.tPacket.fLongitude
							);

							log_i("GPS diff distance: %f m", fDistance);

							if (fDistance >= lora_config.gpsDistanceThreshold) {
								lora_send();
								loraTxTime = millis();
							}
						}
					}
				}
			}
		}

		else {
			if(xQueueBme != NULL) xQueueReset(xQueueBme);
			if(xQueueDgs != NULL) xQueueReset(xQueueDgs);
			if(xQueueGps != NULL) xQueueReset(xQueueGps);
			if(xQueueSps != NULL) xQueueReset(xQueueSps);
			loraTxTime = millis();
		}

		if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, lorawanWdEventId);

		vTaskDelay(1);
	}
}

void lora_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(loraTask, "loraTask", ulStackDepth, (void*)1, uxPriority, &xTaskLora, xCoreID);
}