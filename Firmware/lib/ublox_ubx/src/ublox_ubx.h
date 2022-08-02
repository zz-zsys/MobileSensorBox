#ifndef _ublox_ubx_h
#define _ublox_ubx_h

#include "Arduino.h"
#include "ubx_packet_def.h"



const char ENABLE_RTCM3_OUT_ON_UART1[] PROGMEM = {
	// Enable RTCM3 protocol out and UBX protocol in on uart1
	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x80,0x25,
	0x00,0x00,0x01,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xA9,0x33,
};

class UbxPacketHandler {
public:
	UbxPacketHandler(HardwareSerial &gpsSerial);
	~UbxPacketHandler();

	void calcChecksum(uint8_t* CK, int16_t msgSize);
	void calcChecksum(UBXMessage* msg, uint8_t * CK, int16_t msgSize);
	bool compareMsgHeader(const uint8_t* msgHeader);
	bool compareMsg(uint8_t &msgtype, int16_t &payloadsize);
	int16_t processGPS();
	void enableRTCM3outUBXin();
	void convRelPos2m(double &rpn, double &rpe, double &rpd);
	void convAccRelPos2m(double &accrpn, double &accrpe, double &accrpd);
	void calcDistanceAndAccuracy(double &entfernung2D, double &genauigkeit2D, double &entfernung3D, double &genauigkeit3D);
	uint8_t checkRTKSolution();
	uint8_t checkDiffSolution();
	void sendPacket(uint8_t *packet, uint8_t length);
	void sendUbxPacket(UBXMessage* msg, const uint8_t* clsid, uint16_t len);
	void pollUbxMessage(UBXMessage* msg, const uint8_t* clsid);

	UBXMessage ubxRxMessage;
	UBXMessage ubxTxMessage;

private:
	HardwareSerial *_gpsSerial;

	void setClassIdLength(UBXMessage* msg, const uint8_t *clsid, size_t len);
	void createPacket(uint8_t* packet, UBXMessage* msg, uint8_t* checksum, uint16_t packetLen);

};

#endif
