#include "ublox_ubx.h"

UbxPacketHandler::UbxPacketHandler(HardwareSerial &gpsPort)
{
	_gpsSerial = &gpsPort;
}

UbxPacketHandler::~UbxPacketHandler()
{
}

/***********************************************************************************************************************/
/*!
* @brief		calculate checksum value
* @param[in]	CK			(pointer to the checksum)
* @param[in]	msgSize		(size of the message)
* @retval		none
* @note			The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
* The procedure used to calculate this checksum is given as pseudo-code in the ublox m8 protocol (refer page 133 [RDPS])
*/
/***********************************************************************************************************************/
void UbxPacketHandler::calcChecksum(uint8_t * CK, int16_t msgSize)
{
	memset(CK, 0, 2);

	for (int i = 0; i < msgSize; i++) {
		CK[0] += ((uint8_t*)(&ubxRxMessage))[i];
		CK[1] += CK[0];

#ifdef DEBUG_ENABLE
		_debugSerial->print(F("CK "));
		_debugSerial->print(CK[0], HEX);
		_debugSerial->print(" ");
		_debugSerial->println(CK[1], HEX);
#endif
	}
}

void UbxPacketHandler::calcChecksum(UBXMessage* msg, uint8_t * CK, int16_t msgSize)
{
	memset(CK, 0, 2);

	for (int i = 0; i < msgSize; i++) {
		CK[0] += ((uint8_t*)msg)[i];
		CK[1] += CK[0];
	}
}



/***********************************************************************************************************************/
/*!
* @brief		compare the first two bytes of the ubxRxMessage struct with a specific message header
* @param[in]	msgHeader	(pointer to the message header location in the memory)
* @retval		true if the two bytes match
*/
/***********************************************************************************************************************/
bool UbxPacketHandler::compareMsgHeader(const uint8_t * msgHeader)
{
	uint8_t* ptr = (uint8_t*)(&ubxRxMessage);

#ifdef DEBUG_ENABLE
	_debugSerial->print(F("ptr "));
	_debugSerial->print(ptr[0], HEX);
	_debugSerial->print(F(" "));
	_debugSerial->println(ptr[1], HEX);
#endif

	return ((ptr[0] == msgHeader[0]) && (ptr[1] == msgHeader[1]));
}

/***********************************************************************************************************************/
/*!
* @brief		compare message header and set the message type and its payload size
* @param[in]	msgtype		(reference to the message type)
* @param[in]	payloadsize	(reference to the payload size of the message type)
* @retval		true if known message type, false if message type is unknown
*/
/***********************************************************************************************************************/
bool UbxPacketHandler::compareMsg(uint8_t & msgtype, int16_t & payloadsize)
{
	if (compareMsgHeader(NAV_RELPOSNED_HEADER)) {
		msgtype = MT_NAV_RELPOSNED;
		payloadsize = sizeof(NAV_RELPOSNED);
		return true;
	}

	else if (compareMsgHeader(NAV_TIMEUTC_HEADER)) {
		msgtype = MT_NAV_TIMEUTC;
		payloadsize = sizeof(NAV_TIMEUTC);
		return true;
	}

	else if (compareMsgHeader(NAV_POSLLH_HEADER)) {
		msgtype = MT_NAV_POSLLH;
		payloadsize = sizeof(NAV_POSLLH);
		return true;
	}

	else if (compareMsgHeader(NAV_HPPOSLLH_HEADER)) {
		msgtype = MT_NAV_HPPOSLLH;
		payloadsize = sizeof(NAV_HPPOSLLH);
		return true;
	}

	else if (compareMsgHeader(NAV_HPPOSECEF_HEADER)) {
		msgtype = MT_NAV_HPPOSECEF;
		payloadsize = sizeof(NAV_HPPOSECEF);
		return true;
	}

	else if (compareMsgHeader(NAV_STATUS_HEADER)) {
		msgtype = MT_NAV_STATUS;
		payloadsize = sizeof(NAV_STATUS);
		return true;
	}

	else if (compareMsgHeader(NAV_SVIN_HEADER)) {
		msgtype = MT_NAV_SVIN;
		payloadsize = sizeof(NAV_SVIN);
		return true;
	}

	else if (compareMsgHeader(ACK_ACK_HEADER)) {
		msgtype = MT_ACK_ACK;
		payloadsize = sizeof(ACK_ACK);
		return true;
	}

	else if (compareMsgHeader(ACK_NAK_HEADER)) {
		msgtype = MT_ACK_NAK;
		payloadsize = sizeof(ACK_NAK);
		return true;
	}

	else {
		/// unknown message type, reset to beginning state
		return false;
	}
}

/***********************************************************************************************************************/
/*!
* @brief		process the incoming GPS UBX data from the gps serial.
* @retval		the type of message found if successful, or MT_NONE if no message was found
* @note			reads in bytes from the GPS module and checks to see if a valid message has been constructed. After a
* successful return, the contents of the ubxRxMessage union will be valid for the message type that was found. It means
* that further calls to this function can invalidate the message content, so the obtained values must be used or
* transfered to another variables before calling this function again as the memory will overwriten on the next call of
* the function
*/
/***********************************************************************************************************************/
int16_t UbxPacketHandler::processGPS()
{
	static int16_t fpos = 0;
	static uint8_t checksum[2];

	static uint8_t currentMsgType = MT_NONE;
	static int16_t payloadSize = sizeof(UBXMessage);		//!< the size of UBXMessage depends on the largest struct
															//!< which is NAV_RELPOSNED struct

	while (_gpsSerial->available()) {

		//! read the incoming byte from gps serial
		uint8_t c = _gpsSerial->read();

		//! search from the first two bytes a match with the UBX header bytes (0xB5,0x62)
		if (fpos < 2) {
			if (c == UBX_HEADER[fpos]) {
				fpos++;				//!< accumulate fpos when header bytes are matched
			}
			else fpos = 0;									//!< reset to beginning state
		}

		//! A match with the UBX_HEADER is found, begin to read bytes for the payload.
		//! The incoming bytes will be placed into the ubxRxMessage struct.
		//! The position is fpos-2 because the struct does not include the initial two-byte header (UBX_HEADER).
		else {
			//! place the incoming bytes into the ubxRxMessage struct
			if ((fpos - 2) < payloadSize) {
				((uint8_t*)(&ubxRxMessage))[fpos - 2] = c;
			}

			fpos++;

			//! The second byte of the message type header is just received
			//! compare message header to obtain the current message type from its class and id
			if (fpos == 4) {
				if (!compareMsg(currentMsgType, payloadSize)) {
					fpos = 0;
					continue;
				}
			}

			//! All payload bytes have now been received, so the expected checksum value will be calculated
			//! to be compared with the next two incoming bytes
			if (fpos == (payloadSize + 2)) {

				calcChecksum(checksum, payloadSize);
			}

			//! The first byte after payload which is the first byte of the checksum.
			//! Check if it is matched with the calculated checksum value
			else if (fpos == (payloadSize + 3)) {

				//! Reset to beginning state if the checksum value not matched
				if (c != checksum[0]) {
					fpos = 0;
				}
			}

			//! The second byte after payload which is the second byte of the checksum.
			//! Check if it is matched with the calculated checksum value.
			else if (fpos == (payloadSize + 4)) {
				//! reset the state regardless of whether the checksum matches
				fpos = 0;

				//! check if checksum matches, return the message type if it is matched.
				if (c == checksum[1]) {
					return currentMsgType;
				}
			}

			//! in condition where we read more bytes than both the expected payload and checksum together,
			//! so something went wrong. Reset to beginning state and try again (useful esp. when debugging)
			//! this condition should not be happened and writen for the reliablity of the code
			else if (fpos > (payloadSize + 4)) {
				fpos = 0;
			}
		}
	}

	return MT_NONE;
}

/***********************************************************************************************************************/
/*!
* @brief		enable RTCM3 protocol outgoing message and UBX protocol for ingoing message for base mode
* @param[in]	none
* @retval		none
* @note			none
*/
/***********************************************************************************************************************/
void UbxPacketHandler::enableRTCM3outUBXin()
{
	//! send configuration data in UBX protocol
	for (unsigned int i = 0; i < sizeof(ENABLE_RTCM3_OUT_ON_UART1); i++) {
		_gpsSerial->write(pgm_read_byte(ENABLE_RTCM3_OUT_ON_UART1 + i));
		//! simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
		delay(5);
	}
}

/***********************************************************************************************************************/
/*!
* @brief		convert the relative position NED vector in m
* @param[in]	rpn		(reference for north component of the rel. pos. vector)
* @param[in]	rpe		(reference for east component of the rel. pos. vector)
* @param[in]	rpd		(reference for down component of the rel. pos. vector)
* @retval		none
* @note			the incoming NED component of rel. pos. vector will be parsed with its high precision value. The parsed
* value will be then converted into meter unit.
*/
/***********************************************************************************************************************/
void UbxPacketHandler::convRelPos2m(double &rpn, double &rpe, double &rpd)
{
	rpn = ((double)ubxRxMessage.navRelposned.relPosN / 100.0) + ((double)ubxRxMessage.navRelposned.relPosHPN * 0.0001);
	rpe = ((double)ubxRxMessage.navRelposned.relPosE / 100.0) + ((double)ubxRxMessage.navRelposned.relPosHPE * 0.0001);
	rpd = ((double)ubxRxMessage.navRelposned.relPosD / 100.0) + ((double)ubxRxMessage.navRelposned.relPosHPD * 0.0001);
}

/***********************************************************************************************************************/
/*!
* @brief		convert the accuracy of relative position NED vector in m
* @param[in]	accrpn		(reference for accuracy of north component of the rel. pos. vector)
* @param[in]	accrpe		(reference for accuracy of east component of the rel. pos. vector)
* @param[in]	accrpd		(reference for accuracy of down component of the rel. pos. vector)
* @retval		none
* @note			the incoming accuracy of rel. pos. in NED vector will be converted into meter unit.
*/
/***********************************************************************************************************************/
void UbxPacketHandler::convAccRelPos2m(double &accrpn, double &accrpe, double &accrpd)
{
	accrpn = ((double)ubxRxMessage.navRelposned.accN * 0.0001);
	accrpe = ((double)ubxRxMessage.navRelposned.accE * 0.0001);
	accrpd = ((double)ubxRxMessage.navRelposned.accD * 0.0001);
}

/***********************************************************************************************************************/
/*!
* @brief		calculate the distance and its relative accuracy based on NED vector in m
* @param[in]	distance2D	(reference for 2D distance)
* @param[in]	distanceAcc2D	(reference for relative 2D accuracy)
* @param[in]	distance3D	(reference for 3D distance)
* @param[in]	distanceAcc3D	(reference for relative 3D accuracy)
* @retval		none
* @note			none
*/
/***********************************************************************************************************************/
void UbxPacketHandler::calcDistanceAndAccuracy(double &distance2D, double &distanceAcc2D, double &distance3D, double &distanceAcc3D)
{
	double rpn, rpe, rpd, accrpn, accrpe, accrpd;
	convRelPos2m(rpn, rpe, rpd);
	convAccRelPos2m(accrpn, accrpe, accrpd);

	distance2D = sqrt(sq(rpn) + sq(rpe));
	distance3D = sqrt(sq(distance2D) + sq(rpd));
	distanceAcc2D = sqrt(sq(accrpn) + sq(accrpe));
	distanceAcc3D = sqrt(sq(distanceAcc2D) + sq(accrpd));
}
/***********************************************************************************************************************/
/*!
* @brief		check the RTK solution type from relPosNED message
* @retval		rtk solution (0 = no fix, 1 = rtk double, 2 = rtk fix)
* @note			the flag value will be masked with 0x18 and shifted 3 times to the right.
*/
/***********************************************************************************************************************/
uint8_t UbxPacketHandler::checkRTKSolution()
{
	return ((uint8_t)ubxRxMessage.navRelposned.flags & 0x18) >> 3;
}

/***********************************************************************************************************************/
/*!
* @brief		check the if the differential correction were applied from relPosNED message
* @retval		differential solution (0 = not applied, 1 = applied)
* @note			the flag value will be masked with 0x02 and shifted 1 times to the right.
*/
/***********************************************************************************************************************/
uint8_t UbxPacketHandler::checkDiffSolution()
{
	return (((uint8_t)ubxRxMessage.navRelposned.flags & 0x02) >> 1);
}

void UbxPacketHandler::sendPacket(uint8_t *packet, uint8_t length) {
		
	size_t packetLen;

	while (!_gpsSerial->availableForWrite()) {};

	packetLen = _gpsSerial->write(packet, length);

	if (packet != NULL) {
		uint8_t *a = packet;
		Serial.printf("Packet->GPS: ");
		for (int i = 0; i < length; i++) {
			Serial.printf("0x%.2X ", *a++);
		}
		Serial.printf("\n");
	}
	else log_e("packet == NULL!");

}


void UbxPacketHandler::setClassIdLength(UBXMessage * msg, const uint8_t * clsid, size_t len)
{
	// msg->base.cls = clsid[0];
	// msg->base.id = clsid[1];
	// msg->base.len = len;

	memcpy((uint8_t*)msg, clsid, 2);
	memcpy(((uint8_t*)msg) + 2, &len, 2);
}

void UbxPacketHandler::createPacket(uint8_t* packet, UBXMessage* msg, uint8_t* checksum, uint16_t packetLen) {

	uint16_t tempSize = 0;
	uint16_t msgLen = packetLen - 4;

	memcpy(packet, UBX_HEADER, sizeof(UBX_HEADER));

	tempSize += sizeof(UBX_HEADER);

	memcpy(packet + tempSize, msg, msgLen);

	tempSize += msgLen;

	memcpy(packet + tempSize, checksum, 2); 
}

void UbxPacketHandler::sendUbxPacket(UBXMessage* msg, const uint8_t* clsid, uint16_t msgLen) {
	static uint8_t checksum[2];
	//uint8_t tempSize = 0;
	uint8_t totalSize = sizeof(UBX_HEADER) + msgLen + sizeof(checksum);

	uint8_t* buff = (uint8_t*)malloc(totalSize * sizeof(uint8_t));

	setClassIdLength(msg, clsid, msgLen - 4);

	calcChecksum(msg, checksum, msgLen);

	createPacket(buff, msg, checksum, totalSize);

	sendPacket(buff, totalSize);

	free(buff);
}

void UbxPacketHandler::pollUbxMessage(UBXMessage* msg, const uint8_t* clsid) {

	sendUbxPacket(msg, clsid, sizeof(UBX_BASE));

}







