// 
// 
// 

#include <Arduino.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLERemoteService.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "WifiProvisioning.h"
#include "WebServerClient.h"
#include <esp_bt.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>       // ESP32 BLE
#include <esp_gap_ble_api.h>   // ESP32 BLE
#include "secrets.h"

static const char *PROVISION_TAG = "WIFI_PROVISIONING";
static HardwareSerial *debugSerial;
WPS_CONFIG_T wps_config = { .debug = false };

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/


std::string inWpsSsid;
std::string inWpsPassword;
std::string inAuthBearer;
std::string inBleAuth;

RTC_DATA_ATTR std::string wpsSsid;
RTC_DATA_ATTR std::string wpsPassword;
RTC_DATA_ATTR std::string authBearer;
RTC_DATA_ATTR std::string ble_auth = BLE_AUTH;
uint8_t configConfirmationState = 0;
uint32_t wsc_counter = 0;

#define PROP_READ		BLECharacteristic::PROPERTY_READ
#define PROP_WRITE		BLECharacteristic::PROPERTY_WRITE
#define PROP_NOTIFY		BLECharacteristic::PROPERTY_NOTIFY
#define PROP_INDICATE	BLECharacteristic::PROPERTY_INDICATE

#define WPS_SERVICE_UUID				"57505331-4d42-5342-0000-005a45535953"
#define WPS_SSID_CHAR_UUID				"53534944-4d42-5342-0001-005a45535953"
#define WPS_PASSWORD_CHAR_UUID			"50535744-4d42-5342-0002-005a45535953"
#define AUTH_BEARER_CHAR_UUID			"41555448-4d42-5342-0003-005a45535953"
#define CONFIG_CONFIRMATION_CHAR_UUID	"4346524d-4d42-5342-0004-005a45535953"

#define BLE_AUTH_SERVICE_UUID			"61757468-4d42-5342-0001-005a45535953"
#define BLE_AUTH_CHAR_UUID				"61757468-4d42-5342-0101-005a45535953"

BLEServer *pServer;
BLEService *pWpsService;
BLECharacteristic *pWpsSsidChar;
BLECharacteristic *pWpsPasswordChar;
BLECharacteristic *pAuthBearerChar;
BLECharacteristic *pConfigConfirmationChar;

BLEService *pBleAuthService;
BLECharacteristic * pBleAuthChar;

BLEDescriptor WpsSsidDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor WpsPasswordDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor AuthBearerDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor ConfigConfirmationDescriptor(BLEUUID((uint16_t)0x2901));

BLEAdvertising *pAdvertising;

bool isClientConnected = false;
bool isClientDisconnected = false;
bool isWpsSsidOnWrite = false;
bool isWpsPasswordOnWrite = false;
bool isAuthBearerOnWrite = false;
bool isConfigConfirmationOnWrite = false;
bool isBleAuthOnWrite = false;
bool isBleAuthCorrect = false;

TaskHandle_t xWpsTask;

class MobileSensorServerCallbacks : public BLEServerCallbacks {

	void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

		char remoteAddress[18];

		sprintf(
			remoteAddress,
			"%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
			param->connect.remote_bda[0],
			param->connect.remote_bda[1],
			param->connect.remote_bda[2],
			param->connect.remote_bda[3],
			param->connect.remote_bda[4],
			param->connect.remote_bda[5]
		);

		log_i("MobileSensorServerCallbacks onConnect, MAC: %s", remoteAddress);

		isClientConnected = true;
	}

	void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

		char remoteAddress[20];

		sprintf(
			remoteAddress,
			"%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
			param->disconnect.remote_bda[0],
			param->disconnect.remote_bda[1],
			param->disconnect.remote_bda[2],
			param->disconnect.remote_bda[3],
			param->disconnect.remote_bda[4],
			param->disconnect.remote_bda[5]
		);

		log_i("MobileSensorServerCallbacks onDisconnect, MAC: %s", remoteAddress);

		isClientConnected = false;
		isClientDisconnected = true;
		pAdvertising->start();
	}
};

class BleAuthCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		inBleAuth = pCharacteristic->getValue();
		log_i("Input bleAuth: %s", inBleAuth.c_str());

		if (ble_auth.compare(inBleAuth) == 0) isBleAuthCorrect = true;

		isBleAuthOnWrite = true;
	}
};

class WpsSsidCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		inWpsSsid = pCharacteristic->getValue();
		log_i("Input wpsSsid: %s", inWpsSsid.c_str());
		isWpsSsidOnWrite = true;
	}
};

class WpsPasswordCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		inWpsPassword = pCharacteristic->getValue();
		log_i("Input wpsPassword: %s", inWpsPassword.c_str());
		isWpsPasswordOnWrite = true;
	}
};

class AuthBearerCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		inAuthBearer = pCharacteristic->getValue();
		log_i("Input authBearer: %s", inAuthBearer.c_str());
		isAuthBearerOnWrite = true;
	}
};

class ConfigConfirmationCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		configConfirmationState = pCharacteristic->getData()[0];
		log_i("configConfirmationState: %d", configConfirmationState);
		isConfigConfirmationOnWrite = true;
	}
};

void wps_debug(bool debug, HardwareSerial &port) {
	wps_config.debug = debug;
	debugSerial = &port;
}

void initBLEDevice() {


	log_i("Init BLEDevice");
	BLEDevice::init("MobileSensorBox");
	log_i("Init BLEDevice success");

	log_i("Get the device MAC address");
	const uint8_t* bdAddr = esp_bt_dev_get_address();

	if (bdAddr != NULL) {
		char abdAddr[20];

		sprintf(
			abdAddr,
			"MSB_%.2X%.2X%.2X%.2X%.2X%.2X",
			bdAddr[0],
			bdAddr[1],
			bdAddr[2],
			bdAddr[3],
			bdAddr[4],
			bdAddr[5]
		);

		debugSerial->printf("BT MAC address : %s\n", abdAddr);

		log_i("Set BLE device name");
		esp_err_t errRc = ::esp_ble_gap_set_device_name(abdAddr);
	}

	else log_e("bdAddr == NULL");
}

void initBLEServer() {
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MobileSensorServerCallbacks());

	pBleAuthService = pServer->createService(BLEUUID(BLE_AUTH_SERVICE_UUID), 100);
	pBleAuthChar = pBleAuthService->createCharacteristic(BLE_AUTH_CHAR_UUID, PROP_WRITE);
	pBleAuthChar->setCallbacks(new BleAuthCharacteristicCallbacks());

	pWpsService = pServer->createService(BLEUUID(WPS_SERVICE_UUID), 100);

	pWpsSsidChar = pWpsService->createCharacteristic(WPS_SSID_CHAR_UUID, PROP_READ | PROP_WRITE);
	WpsSsidDescriptor.setValue("WiFi SSID");
	pWpsSsidChar->addDescriptor(&WpsSsidDescriptor);
	pWpsSsidChar->setCallbacks(new WpsSsidCharacteristicCallbacks());
	pWpsSsidChar->setValue("");

	pWpsPasswordChar = pWpsService->createCharacteristic(WPS_PASSWORD_CHAR_UUID, PROP_READ | PROP_WRITE);
	WpsPasswordDescriptor.setValue("WiFi Password");
	pWpsPasswordChar->addDescriptor(&WpsPasswordDescriptor);
	pWpsPasswordChar->setCallbacks(new WpsPasswordCharacteristicCallbacks());
	pWpsPasswordChar->setValue("");

	pAuthBearerChar = pWpsService->createCharacteristic(AUTH_BEARER_CHAR_UUID, PROP_READ | PROP_WRITE);
	AuthBearerDescriptor.setValue("Authorization: Bearer");
	pAuthBearerChar->addDescriptor(&AuthBearerDescriptor);
	pAuthBearerChar->setCallbacks(new AuthBearerCharacteristicCallbacks());
	pAuthBearerChar->setValue("");

	pConfigConfirmationChar = pWpsService->createCharacteristic(CONFIG_CONFIRMATION_CHAR_UUID, PROP_WRITE);
	ConfigConfirmationDescriptor.setValue("Confirmation");
	pConfigConfirmationChar->addDescriptor(&ConfigConfirmationDescriptor);
	pConfigConfirmationChar->addDescriptor(new BLE2902());
	pConfigConfirmationChar->setCallbacks(new ConfigConfirmationCharacteristicCallbacks());
	pConfigConfirmationChar->setValue("");

	pBleAuthService->start();
	pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(BLE_AUTH_SERVICE_UUID);
	pAdvertising->addServiceUUID(WPS_SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	BLEDevice::startAdvertising();

	debugSerial->printf("BLE Server started!\n");
}

void wpsTask(void * parameter) {

	wps_debug();

	log_i("Init BLE Device");

	initBLEDevice();

	initBLEServer();

	for (;;) {

		if (isClientConnected) {

			debugSerial->printf("Free Heap: %d\n", ESP.getFreeHeap());

			if (isBleAuthCorrect) {
				isBleAuthCorrect = false;
				pWpsService->start();
				
			}

			if (isConfigConfirmationOnWrite && configConfirmationState == 0x01) {

				wpsSsid = inWpsSsid;
				wpsPassword = inWpsPassword;
				authBearer = inAuthBearer;

				log_i("wifiSsid: %s", wpsSsid.c_str());
				log_i("wifiPassword: %s", wpsPassword.c_str());
				log_i("authBearer: %s", authBearer.c_str());
				log_i("configConfirmationState: %d", configConfirmationState);

				if (!wsc_counter) {
					wsc_taskInit();
					wsc_counter++;
				}
				else {
					wsc_taskDelete();
					wsc_taskInit();
				}
				configConfirmationState = 0;
				pConfigConfirmationChar->setValue(&configConfirmationState, sizeof(configConfirmationState));
			}
		}

		if (isClientDisconnected) {
			isClientDisconnected = false;
			
			pBleAuthChar->setValue("");
			pWpsService->stop();
		}

		delay(1000);
	}
}

void wps_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(
		wpsTask,
		"wpsTask",
		ulStackDepth,
		(void*)1,
		uxPriority,
		&xWpsTask,
		xCoreID
	);
}
