#include "gps.h"
#include "led.h"


GPS_CONFIG_T gps_config = {							// GPS config struct initialization
	.debug			= false,						// debug flag
	.ser_port_nr	= GPS_SERIAL_PORT_NR,			// serial port number
	.baud			= 9600,							// baud rate
	.ser_config		= SERIAL_8N1,					// serial config
	.rxPin			= 12,							// RX pin
	.txPin			= 15,							// TX pin
	.sendIntervalMs = GPS_SEND_INTERVAL_MS			// send packet frequency in ms
};

HardwareSerial gpsSerial(gps_config.ser_port_nr);	// GPS serial port instance 
UbxPacketHandler gps(gpsSerial);					// UBX GPS packet handler instance
static HardwareSerial *debugSerial;					// pointer to the debug serial port

GPS_PACKET_T tGpsTxPacket;							// GPS send lora packet instance
MESSAGE_BUFFER_T tGpsTxDisplayPacket;				// GPs send display packet instance

TaskHandle_t xTaskGps;								// task handle for gps task
QueueHandle_t xQueueGps;							// queue handle for gps task

//! NAV-TIMEUTC variables
uint16_t year;								
uint8_t	month, day, hour, minute, seconds;			

//! NAV-STATUS variables;
bool timeValid;										// time valid flag
bool gpsValid;										// gps fix flag
uint32_t gpsFixCounter = 0;

//! NAV-POSLLH variables
float lat, lon, altitude;							// latitude,longitude,and altitude

//! Flag for UBX message read
bool flagTimeUTC, flagPosLLH, flagStatus;

time_t lastConfigTime;								// last config time
time_t currentTime;									// current system time
uint32_t gpsTaskTime = 0;							// time counter for gps task

RTC_DATA_ATTR bool isTimeSet = false;				// time set flag
bool isGpsSentDataAvailable = false;				// gps sent data availability flag
bool gps_header = false;							// gps header flag for debugging purpose



/************************************************************************************************************************/
/*!
* @brief		configure the debugging mode for the gps task
* @param[in]	debug	debug flag (true if enable, false if disable)
* @param[in]	port	serial port for debugging purpose
* @retval		none
*/
/************************************************************************************************************************/
void gps_debug(bool debug, HardwareSerial &port) {
	gps_config.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		initialize the GPS
* @param[in]	debug		debug flag (true to enable, false (default) to disable)
* @param[in]	debugPort	pointer to the debugger serial port
* @retval		ESP_OK if success, ESP_FAIL if failed
*/
/************************************************************************************************************************/

esp_err_t gps_init(bool debug, HardwareSerial &debugPort) {

	// enable/disable the debugging mode
	gps_debug(debug, debugPort);

	// create the gps queue instance
	if (xQueueGps == NULL) xQueueGps = xQueueCreate(SEND_QUEUE_SIZE, sizeof(tGpsTxPacket.tPacket));

	if (xQueueGps == 0) {
		log_e("Could not create GPS packet send queue. Aborting..");
		return ESP_FAIL;
	}

	if(gps_config.debug) log_i("GPS send queue created, size %d Bytes", SEND_QUEUE_SIZE * sizeof(tGpsTxPacket.tPacket));

	// initialize the gps serial port
	gpsSerial.begin(gps_config.baud, gps_config.ser_config, gps_config.rxPin, gps_config.txPin);

	// wake the gps if sleep
	gpsSerial.write(0xFF);

	debugSerial->printf("Wake up the gps from sleep!\n");

	//// wake the gps if sleep
	//do {
	//	gpsSerial.write(0xFF);
	//} while (!gpsSerial.availableForWrite());


	return ESP_OK;
}

/************************************************************************************************************************/
/*!
* @brief		print the system current time
* @param[in]	input_time		input time
* @retval		none
* @note			debugging mode must be enabled to set the current time on serial monitor
*/
/************************************************************************************************************************/
void sys_printCurrentTime(time_t input_time) {
	struct tm localtm = { 0 };
	struct tm gmtm = { 0 };
	
	if (gps_config.debug)	debugSerial->printf("setTime: %ld\n", input_time);
	
	gmtime_r(&input_time, &gmtm);
	if (gps_config.debug)	debugSerial->printf("The GMT date/time is: %s", asctime(&gmtm));
	

	localtime_r(&input_time, &localtm);
	if (gps_config.debug)	debugSerial->printf("The current date/time is: %s\n", asctime(&localtm));

}

/************************************************************************************************************************/
/*!
* @brief		setup the system time
* @param[in]	setTime		epoch time in time_t format
* @param[in]	value		timezone setenv value (see https://bit.ly/2JjeoBD for more examples)		
* @retval		none
*/
/************************************************************************************************************************/
void sys_setupTime(time_t setTime, const char* value) {

	// log the set time into the lastConfigTime
	lastConfigTime = setTime;

	struct timeval now = { setTime, 0 };
	
	// set the system time
	settimeofday((const timeval*)&now, NULL);

	// set/change the environment timezone
	setenv("TZ", value, 1);	// default Berlin, Germany

	// set time zone
	tzset();

	time_t nowTime = time(NULL);

	// print the current time
	sys_printCurrentTime(nowTime);

}

/************************************************************************************************************************/
/*!
* @brief		print the time (convert gps struct tm data to epoch time)
* @param[in]	source		source name
* @param[in]	year		year 		
* @param[in]	month		month
* @param[in]	day			day
* @param[in]	hour		hour
* @param[in]	minute		minute
* @param[in]	seconds		seconds
* @retval		epoch time in time_t format
*/
/************************************************************************************************************************/
time_t printTime(const char* source, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds) {
	int ret;
	struct tm mytime;
	char buffer[80];

	mytime.tm_year = year - 1900;
	mytime.tm_mon = month - 1;
	mytime.tm_mday = day;
	mytime.tm_hour = hour;
	mytime.tm_min = minute;
	mytime.tm_sec = seconds;
	mytime.tm_isdst = 1;

	ret = mktime(&mytime);

	if (ret != -1) {
		strftime(buffer, sizeof(buffer), "%c", &mytime);
		if (gps_config.debug) debugSerial->printf("\n%s time: %d\n", source, ret);
		if (gps_config.debug) debugSerial->printf("%s time: %s", source, asctime(&mytime));
		if (gps_config.debug) debugSerial->printf("%s time: %02d:%02d:%02d\n", source, hour, minute, seconds);
		sys_printCurrentTime(ret);
	}

	return ret;
}




/***********************************************************************************************************************/
/*!
* @brief		Process GPS incoming data
* @retval		true if all the needed message has been parsed, false if not
* @note			none
*/
/***********************************************************************************************************************/
bool gps_trackDevice() {

	// read the gps data and its message type
	int16_t msgType = gps.processGPS();

	// if the data from UBX_NAV_TIMEUTC data packet, assign the processed data to local variables
	if ((msgType == MT_NAV_TIMEUTC) && !flagTimeUTC) {
		day = gps.ubxRxMessage.navTimeutc.day;
		month = gps.ubxRxMessage.navTimeutc.month;
		year = gps.ubxRxMessage.navTimeutc.year;
		hour = gps.ubxRxMessage.navTimeutc.hour;
		minute = gps.ubxRxMessage.navTimeutc.minute;
		seconds = gps.ubxRxMessage.navTimeutc.seconds;
		timeValid = gps.ubxRxMessage.navTimeutc.tFlags.validUTC;
		
		flagTimeUTC = true;

		// if time is valid
		if (timeValid) {
			if (gps_config.debug) debugSerial->printf("GPS time is valid\n");
			if (!isTimeSet) {

				// convert the tm struct to epoch time
				time_t gpsTime = printTime("UTC", year, month, day, hour, minute, seconds);

				// setup the system time using the gps time
				sys_setupTime(gpsTime);

				isTimeSet = true;
			}
		}

		else {
			if (gps_config.debug) debugSerial->printf("GPS time is not yet valid\n");
		}

	}

	// if the data from UBX_NAV_POSLLH data packet, assign the processed data to local variables
	else if ((msgType == MT_NAV_POSLLH) && !flagPosLLH) {

		lat = gps.ubxRxMessage.navPosllh.lat	* (double)1e-7;				//!< convert the latitude into degree unit
		lon = gps.ubxRxMessage.navPosllh.lon	* (double)1e-7;				//!< convert the longitude into degree unit
		altitude = gps.ubxRxMessage.navPosllh.hMSL	* (double)1e-3;		//!< convert the altitude (MSL) into meter unit
		flagPosLLH = true;
	}

	// if the data from UBX_NAV_STATUS data packet, gpsFixOk flag is checked to determine if gps position valid or not
	else if ((msgType == MT_NAV_STATUS) && !flagStatus) {
		if (gps.ubxRxMessage.navStatus.tFlags.gpsFixOk) {
			if (gps_config.debug) debugSerial->printf("gpsFixOk is valid\n");
			gpsValid = true;
			gpsFixCounter++;
			if (gps_config.debug) debugSerial->printf("gpsFixCounter: %d\n", gpsFixCounter);

			measure_led_state(LED_BLINK);
		}
		else {
			if (gps_config.debug) debugSerial->printf("gpsFixOk is not valid\n");
			gpsValid = false;
			measure_led_state(LED_ON);
		}

		flagStatus = true;
	}

	if (flagTimeUTC && flagPosLLH && flagStatus) {
		flagTimeUTC = false;
		flagPosLLH = false;
		flagStatus = false;
		return true;
	}

	else return false;

}

/************************************************************************************************************************/
/*!
* @brief		Put the gps on sleep mode
* @param[in]	duration	duration of the power save mode in ms (default: 0 = infinite sleep)
* @retval		none
* @note			The receiver goes into backup mode for a time period defined by duration.
*/
/************************************************************************************************************************/
void gps_enableSleep(uint32_t duration) {
	memset((uint8_t*)&gps.ubxTxMessage, 0x00, sizeof(gps.ubxTxMessage));

	gps.ubxTxMessage.rxmPmreq.duration = duration;
	gps.ubxTxMessage.rxmPmreq.tFlags.backup = 1;


	gpsSerial.flush();
	
	gps.sendUbxPacket(&gps.ubxTxMessage, RXM_PMREQ_HEADER, sizeof(gps.ubxTxMessage.rxmPmreq));

	delay(100);

	if(gps_config.debug) log_i("gps sleep mode activated..");
}

/************************************************************************************************************************/
/*!
* @brief		Wake the gps from power save mode (sleep)
* @retval		none
*/
/************************************************************************************************************************/
void gps_wakeUp() {
	gpsSerial.write(0xFF);
}

/************************************************************************************************************************/
/*!
* @brief		GPS task entry
* @param[in]	parameter	parameter passed into the task
* @retval		none
*/
/************************************************************************************************************************/
void gpsTask(void * parameter) {

	gps_debug(false);

	debugSerial->printf("Start GPS task!\n");

	esp_err_t lRet = ESP_OK;

	measure_led_init();

	// initialize the gps and wait until it is finished
	while (ESP_OK != (lRet = gps_init(GPS_DEBUG))) {
	};

	// start the task timer counter
	gpsTaskTime = millis();

	for (;;) {

		// check if all the gps message has been parsed/decoded
		if (gps_trackDevice()) { // if yes,...

			// clear the gps send packet memory to zero
			memset(&tGpsTxPacket, 0x00, sizeof(tGpsTxPacket.tPacket));

			// if system time already configured
			if (isTimeSet) {
				// get the current epoch time
				time(&currentTime);

				// print the current time 
				sys_printCurrentTime(currentTime);
			}

			// copy the gps decoded data to gps send packet
			tGpsTxPacket.tPacket.fLatitude = lat;
			tGpsTxPacket.tPacket.fLongitude = lon;
			tGpsTxPacket.tPacket.sAltitude_x100 = (int16_t)(altitude*100);

			if (gps_config.debug) {
				if (!gps_header) {
					debugSerial->printf("%-s\t\t%-s\t\t%-s\n", "Lat", "Lon", "Alt");
					gps_header = true;
				}

				debugSerial->printf("%-2.6f\t%-2.6f\t%-2.3f\n", lat, lon, altitude);
			}

			// set active bit to the low power event group when the gps fix is valid
			if (xLowPowerEvent != NULL && gpsFixCounter > 10) {
				xEventGroupSetBits(xLowPowerEvent, gpsLpEventId);
				gpsFixCounter = 0;
			}
			
			// set gps send availability flag to true
			isGpsSentDataAvailable = true;

		}

		// check if the task timer counter equal or greater than the configured send packet interval time
		// and if the gps send data is available
		if ((millis() - gpsTaskTime >= gps_config.sendIntervalMs) && isGpsSentDataAvailable) {

			// copy the gps send data packet to the gps display data packet
			tGpsTxDisplayPacket.bPort = GPS_DATAPORT;
			tGpsTxDisplayPacket.bMessageSize = sizeof(tGpsTxPacket.tPacket);
			memcpy(tGpsTxDisplayPacket.abPacket, tGpsTxPacket.abPacket, tGpsTxDisplayPacket.bMessageSize);

			// enqueue the gps display data packet
			if (xQueueDisplay != NULL && xQueueSendToBack(xQueueDisplay, &tGpsTxDisplayPacket, 100 / portTICK_RATE_MS) == pdTRUE) {

			}

			
#if SEND_GPS_PACKET_WHEN_FIX_ONLY > 0			// in case when the gps packet is sent only when gpsFixOk is valid
			// if gps data is valid
			if (gpsValid) {

				// enqueue the gps data packet in gps queue for lora packet
				if (xQueueSendToBack(xQueueGps, &tGpsTxPacket.tPacket, 500 / portTICK_RATE_MS) != pdTRUE) {
					log_e("Fail to send gps data in queue");
				}
				else {
					if (DEBUG_SEND_PACKET) debugSerial->printf("GPS TX Packet\t->\t%-2.4f\t%-2.4f\t%-2.4f\n\n", tGpsTxPacket.tPacket.fLatitude, tGpsTxPacket.tPacket.fLongitude, tGpsTxPacket.tPacket.sAltitude_x100 / 100.0);
					// reset gps send availability flag to false
					isGpsSentDataAvailable = false;
					// reset the task timer counter
					gpsTaskTime = millis();
				}
			}
#else	// in case when the gps packet is sent no matter the gpsFixOk valid or not
			// enqueue the gps data packet in gps queue for lora packet
			if (xQueueSendToBack(xQueueGps, &tGpsTxPacket.tPacket, 500 / portTICK_RATE_MS) != pdTRUE) {
				log_e("Fail to send gps data in queue");
			}

			else {
				if (DEBUG_SEND_PACKET) debugSerial->printf("GPS TX Packet\t->\t%-2.4f\t%-2.4f\t%-2.4f\n\n", tGpsTxPacket.tPacket.fLatitude, tGpsTxPacket.tPacket.fLongitude, tGpsTxPacket.tPacket.sAltitude_x100 / 100.0);
				// reset gps send availability flag to false
				isGpsSentDataAvailable = false;
				// reset the task timer counter
				gpsTaskTime = millis();
			}
#endif
			// set active bit to the watchdog event group
			if (xWatchdogEvent != NULL) xEventGroupSetBits(xWatchdogEvent, gpsWdEventId);

		}

		// delay the task for a given time/ticks
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/************************************************************************************************************************/
/*!
* @brief		create and initialize the gps task
* @param[in]	ulStackDepth	size of the task stack in bytes
* @param[in]	uxPriority		task priority
* @param[in]	xCoreID			pinned core where the task will be run
* @retval		none
*/
/************************************************************************************************************************/
void gps_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(gpsTask, "gpsTask", ulStackDepth, (void*)1, uxPriority, &xTaskGps, xCoreID);
}




