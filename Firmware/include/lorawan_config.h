#ifndef _LORAWAN_CONFIG_h
#define _LORAWAN_CONFIG_h

#include <secrets.h>

// This EUI must be in little-endian format, so least-significant-byte first. The added EUI is in
// big-endian format and must be convert later to little-endian format
static const uint8_t PROGMEM DEVEUI[8] = LORA_DEVEUI;

// This EUI must be in little-endian format, so least-significant-byte first. The added EUI is in
// big-endian format and must be convert later to little-endian format
static const uint8_t PROGMEM APPEUI[8] = LORA_APPEUI;		

// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does 
// not really apply). In practice, a key taken from ttnctl can be copied as-is. The key shown here is the semtech default key.
static const uint8_t PROGMEM APPKEY[16] = LORA_APPKEY;	

#endif