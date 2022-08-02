#ifndef _LORAWAN_CONFIG_h
#define _LORAWAN_CONFIG_h

// This EUI must be in little-endian format, so least-significant-byte first. The added EUI is in
// big-endian format and must be convert later to little-endian format
//static const uint8_t PROGMEM DEVEUI[8] = { 0x00, 0x2C, 0x25, 0xC8, 0xB9, 0x74, 0x4E, 0x9C };		// ttgo-t-beam_thingsos
//static const uint8_t PROGMEM DEVEUI[8] = { 0x00, 0xA9, 0xF2, 0x85, 0x99, 0x97, 0x41, 0x83 };		// msb_tbeam_ttn
static const uint8_t PROGMEM DEVEUI[8] = LORA_DEVEUI;		// msb_zesys_001_ttn

// This EUI must be in little-endian format, so least-significant-byte first. The added EUI is in
// big-endian format and must be convert later to little-endian format
//static const uint8_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0x63, 0xE9 };		// ttgo-t-beam_thingsos
//static const uint8_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0x63, 0xE9 };		// msb_tbeam_ttn
static const uint8_t PROGMEM APPEUI[8] = LORA_APPEUI;		// msb_zesys_001_ttn

// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does 
// not really apply). In practice, a key taken from ttnctl can be copied as-is. The key shown here is the semtech default key.
//static const uint8_t PROGMEM APPKEY[16] = { 0xA3, 0xC4, 0x4D, 0x10, 0x28, 0xBD, 0x6F, 0xC0, 0x0B, 0x2D, 0xB4, 0xE2, 0x77, 0x17, 0x44, 0xA7 };	// ttgo-t-beam_thingsos 
//static const uint8_t PROGMEM APPKEY[16] = { 0xF0, 0x79, 0x46, 0x76, 0x9C, 0x33, 0xB5, 0x07, 0xDB, 0xD6, 0xD7, 0x40, 0x4D, 0x10, 0x06, 0xD3 }; // msb_tbeam_ttn
static const uint8_t PROGMEM APPKEY[16] = LORA_APPKEY;	// msb_zesys_001_ttn

#endif