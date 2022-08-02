// lorawan.h

#ifndef _LORAWAN_h
#define _LORAWAN_h

#include <Arduino.h>

// workaround for arduino-espressif32 v2.0.0 (see isse #714 @ MCCI_LMIC)
#define hal_init LMICHAL_init

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "globals.h"
#include "lorawan_config.h"

typedef  __PACKED_PRE struct LORA_CONFIG_Ttag {
	bool		debug;
	uint8_t		adrMode;
	uint8_t		checkLinkMode;
	uint8_t		spreadFactor;
	uint8_t		txPower;
	uint8_t		ackConfirmed;
	uint32_t	sendIntervalMs;
	uint32_t	minSendIntervalMs;
	float		gpsDistanceThreshold;
} __PACKED_POST LORA_CONFIG_T;

void lora_debug(bool debug = false, HardwareSerial &port = Serial);
esp_err_t lora_init(bool debug = false, HardwareSerial &debugPort = Serial);
void lora_reverseBytes(uint8_t *b, size_t c);
char* appendKey(char *buf, const char *name, const uint8_t *key, size_t len, bool lsb);
char* appendDataInHex(char *buf, const char *name, uint8_t *key, uint8_t begin, size_t len);
void lora_showSessionKeys();
void lora_switch_SFTX(uint8_t sf, uint8_t tx);
size_t injectDataToLoraPacket(uint8_t* buf, uint8_t *input, size_t inputSize, bool first);
esp_err_t lora_checkIncomingPacket();
esp_err_t lora_enqueuedata();
void lora_send();
void onEvent(ev_t ev);
void loraTask(void * parameter);
void lora_taskInit(uint32_t ulStackDepth = 4096, UBaseType_t uxPriority = 3, const BaseType_t xCoreID = 1);



#endif

