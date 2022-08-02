// 
// 
// 

#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "globals.h"
#include <base64.h>
extern "C" {
#include "mbedtls/base64.h"
}
#include "WebServerClient.h"
#include "secrets.h"

base64 b;
static HardwareSerial *debugSerial;
WSC_CONFIG_T wsc_config = {
	.debug				= false,
	.sendIntervalInMs	= 60000
};

//const char* host = "zesys";
const char* ntpServer = "pool.ntp.org";
const long  timezone = 1;
const int   daysavetime = 1;

//variabls for blinking an LED with Millis
//const int led = LED_BUILTIN; // ESP32 Pin to which onboard LED is connected
//unsigned long previousMillis = 0;  // will store last time LED was updated
//const long interval = 1000;  // interval at which to blink (milliseconds)
//int ledState = LOW;  // ledState used to set the LED

//const char* tz_berlin = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";
String nowTime;
char abTimePacket[25] = { 0 };
char *timePtr;

String sendOutput;

BME_PACKET_T	tBmePacket;							// BME280 packet
DGS_PACKET_T	tDgsPacket;							// DGS packet
GPS_PACKET_T	tGpsPacket;							// GPS packet
SPS_PACKET_T	tSpsPacket;							// SPS30 packet
uint16_t		usBatteryStatus;					// battery voltage	

TaskHandle_t xWscTask;

HTTPClient http;
char base[1024];
String test_server_path = TEST_SERVER_PATH;
String backend_server_path = BACKEND_SERVER_PATH;

String auth = BACKEND_SERVER_AUTH;
String device_name = WIFI_DEV_NAME;

uint32_t h0, h1, h2, h3, h4;
uint8_t temp[] = { 0xae, 0x0c, 0x22, 0xa1, 0x27, 0xff, 0xff, 0x00, 0x00, 0xfb, 0xbd, 0x51, 0x42, 0xd9, 0x8f, 0x58, 0x41, 0x23, 0x1e, 0x9f, 0xae, 0xf6, 0x40, 0x9f, 0xae, 0xf6, 0x40, 0x0c, 0x02, 0xbe, 0x0f };

void wsc_debug(bool debug, HardwareSerial &port) {
	wsc_config.debug = debug;
	debugSerial = &port;
}

bool getUTCTime(struct tm * info, uint32_t ms)
{
	uint32_t start = millis();
	time_t now;
	while ((millis() - start) <= ms) {
		time(&now);
		gmtime_r(&now, info);
		if (info->tm_year > (2016 - 1900)) {
			return true;
		}

		debugSerial->printf("Wait for system time initialization...\n");
		delay(10);
	}
	return false;
}

int getSystemUTCTime(char *timeBuffer, int timeBufferLength) {
	struct tm timeinfo;
	time_t now;

	while (!getUTCTime(&timeinfo, 5000)) {
		log_e("Failed to obtain time");
		delay(1000);
	}

	time(&now);

	struct timeval tval;

	char tempTime[25] = { 0 };

	strftime(tempTime, sizeof(tempTime), "%Y-%m-%dT%H:%M:%S", &timeinfo);

	gettimeofday(&tval, NULL);

	int tLen = sprintf(timeBuffer, "%s.%06ld000Z", tempTime, tval.tv_usec);

	log_i("Now: %ld, TimePacket: %s, DateTime: %s", now, timeBuffer, asctime(&timeinfo));

	return tLen;
}

bool sendHttpPostJsonData(String *destination, String *auth, String *jsonData) {
	if (WiFi.status() == WL_CONNECTED) {   //Check WiFi connection status

	//Specify destination for HTTP request
		http.begin(*destination);
		http.setTimeout(1000);

		http.addHeader("Authorization", "Bearer " + *auth);
		http.addHeader("Content-Type", "application/json");

		debugSerial->printf("Send data to %s\n", destination->c_str());


		int httpResponseCode = http.POST(*jsonData);   //Send the actual POST request

		if (httpResponseCode > 0) {
			log_i("Response: %d", httpResponseCode);;   //Print return code
		}
		else {
			log_e("Error on sending request, Response: %d", httpResponseCode);
		}

		http.end();  //Free resources

		return true;
	}
	else {
		log_e("Error in WiFi connection");
		return false;
	}
}

void processIncomingPacket(uint8_t* temp, int len) {

	String* dataEncode;
	String* pTime;
	String* sendOutput;

	sendOutput = new String((char *)0);
	sendOutput->reserve(300);

	dataEncode = new String(b.encode(temp, len));

	// get the current time for metadata time
	int tLen = getSystemUTCTime(abTimePacket, sizeof(abTimePacket));
	pTime = new String(abTimePacket);

	StaticJsonDocument<200> doc;

	doc["dev_id"] = device_name.c_str();
	doc["payload_raw"] = dataEncode->c_str();

	JsonObject metadata = doc.createNestedObject("metadata");

	metadata["time"] = pTime->c_str();

	serializeJson(doc, *sendOutput);
	debugSerial->printf("%s\n", sendOutput->c_str());

	delete dataEncode;
	delete pTime;

	sendHttpPostJsonData(&backend_server_path, &auth, sendOutput);
	//if (!sendHttpPostJsonData(&test_server_path, &auth, sendOutput)) delay(1000);


	delete sendOutput;
}

void wscTask(void *parameter) {
	
	wsc_debug();

	// Connect to WiFi network
	WiFi.begin(wpsSsid.c_str(), wpsPassword.c_str());
	
	debugSerial->printf("Connecting...");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		debugSerial->printf(".");
	}
	debugSerial->printf("Connected to %s, IP Address: %s\n", wpsSsid.c_str(), WiFi.localIP().toString().c_str());

	// configure system time using NTP service
	configTime(3600 * timezone, daysavetime * 3600, "pool.ntp.org");
	
	for (;;) {

		h1 = ESP.getFreeHeap();

		processIncomingPacket(temp, sizeof(temp));

		h2 = ESP.getFreeHeap();

		log_i("Check free heap: h1:%d, h2:%d", h1, h2);

		delay(wsc_config.sendIntervalInMs);
	}
}

void wsc_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(
		wscTask,
		"wscTask",
		ulStackDepth,
		(void*)1,
		uxPriority,
		&xWscTask,
		xCoreID
	);
}

void wsc_taskDelete() {
	WiFi.disconnect(true, true);
	vTaskDelete(xWscTask);
}



