#ifndef _UBX_PACKET_DEF_H_
#define _UBX_PACKET_DEF_H_

//! Parameter definition for GPS function
const uint8_t UBX_HEADER[] = { 0xB5, 0x62 };	//!< UBX header 
const uint8_t NAV_RELPOSNED_HEADER[] = { 0x01, 0x3C };	//!< NAV_RELPOSNED message header
const uint8_t NAV_TIMEUTC_HEADER[] = { 0x01, 0x21 };	//!< NAV_TIMEUTC message header
const uint8_t NAV_POSLLH_HEADER[] = { 0x01, 0x02 };	//!< NAV_POSLLH message header
const uint8_t NAV_HPPOSLLH_HEADER[] = { 0x01, 0x14 };	//!< NAV_HPPOSLLH message header
const uint8_t NAV_HPPOSECEF_HEADER[] = { 0x01,0x13 };	//!< NAV_HPPOSECEF message header
const uint8_t NAV_STATUS_HEADER[] = { 0x01, 0x03 };	//!< NAV_STATUS message header
const uint8_t NAV_SVIN_HEADER[] = { 0x01,0x3B };	//!< NAV_SVIN message header
const uint8_t RXM_PMREQ_HEADER[] = { 0x02, 0x41 };	//!< RXM_PMREQ message header
const uint8_t ACK_ACK_HEADER[] = { 0x05,0x01 };	//!< ACK_ACK message header
const uint8_t ACK_NAK_HEADER[] = { 0x05,0x00 };	//!< ACK_NAK message header
const uint8_t CFG_PRT_HEADER[] = { 0x06,0x00 };	//!< CFG_PRT message header


enum _ubxMsgType {
	MT_NONE,							//!< unknown message type
	MT_NAV_RELPOSNED,					//!< NAV_RELPOSNED message type
	MT_NAV_TIMEUTC,						//!< NAV_TIMEUTC message type
	MT_NAV_POSLLH,						//!< NAV_POSLLH message type
	MT_NAV_HPPOSLLH,					//!< NAV_HPPOSLLH message type
	MT_NAV_HPPOSECEF,					//!< NAV_HPPOSECEF message type
	MT_NAV_STATUS,						//!< NAV_STATUS message type
	MT_NAV_SVIN,						//!< NAV_SVIN message type
	MT_ACK_ACK,							//!< ACK_ACK message type
	MT_ACK_NAK,							//!< ACK_NAK message type
	MT_CFG_PRT,							//!< CFG_PRT message type
	MT_RXM_PMREQ,						//!< RXM_PMREQ message type
};

enum _carrSoln {
	NO_RTK = (uint8_t)0,
	RTK_FLOAT = (uint8_t)1,
	RTK_FIX = (uint8_t)2
};

struct __attribute__((__packed__)) UBX_BASE {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
};

//! NAV_RELPOSNED message struct (refer <a href="https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=315"> this </a>)
struct __attribute__((__packed__)) NAV_RELPOSNED {
	uint8_t		cls;					//!< class number 
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		version;				//!< message version (0x00 for this version)
	uint8_t		rsv1;					//!< reserved for future expansion
	uint16_t	refStationId;			//!< reference station ID (range 0..4095)
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch (ms)
	int32_t		relPosN;				//!< north component of rel. position vector * 1e-2					(m)
	int32_t		relPosE;				//!< east component of rel. position vector * 1e-2					(m)
	int32_t		relPosD;				//!< down component of rel. position vector * 1e-2					(m)
	int8_t		relPosHPN;				//!< high-precision north component of rel. position vector	* 1e-4	(m)
	int8_t		relPosHPE;				//!< high-precision east component of rel. position vector	* 1e-4	(m)
	int8_t		relPosHPD;				//!< high-precision down component of rel. position vector	* 1e-4	(m)
	uint8_t		rsv2;					//!< reserved for future expansion
	uint32_t	accN;					//!< accuracy of rel. position north component	* 1e-4				(m)
	uint32_t	accE;					//!< accuracy of rel. position east component	* 1e-4				(m)
	uint32_t	accD;					//!< accuracy of rel. position down component	* 1e-4				(m)
	uint32_t	flags;					//!< condition flags (refer <a href="https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=316"> this </a>)
};

struct __attribute__((__packed__)) NAV_TIMEUTC_FLAGS_T {
	uint8_t validTOW : 1;	//!< 1 = valid time of week
	uint8_t validWKN : 1;	//!< 1 = valid week number 
	uint8_t validUTC : 1;	//!< 1 = valid UTC (leap seconds already known)
	uint8_t reserved : 5;
};

//! NAV_TIMEUTC message struct (refer <a href="https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=333"> this </a>)
struct __attribute__((__packed__)) NAV_TIMEUTC {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch	(ms)
	uint32_t	tAcc;					//!< time accuracy estimate (UTC)				(ns)
	int32_t		nano;					//!< fraction of second, range -1e9..1e9 (UTC)	(ns)
	uint16_t	year;					//!< year, range 1999..2099 (UTC)				(y)
	uint8_t		month;					//!< month, range 1..12 (UTC)					(month)
	uint8_t		day;					//!< day of month, range 1..31 (UTC)			(d)
	uint8_t		hour;					//!< hour of day, range 0..23 (UTC)				(h)
	uint8_t		minute;					//!< minute of hour, range 0..59 (UTC)			(min)
	uint8_t		seconds;				//!< seconds of minute, range 0..60 (UTC)		(s)
	union {
		NAV_TIMEUTC_FLAGS_T tFlags;	//!< bitfield flags struct
		uint8_t		valid;				//!< validity flags(refer <a href = "https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=333"> this < / a>)
	};
};

//! NAV_POSLLH message struct (refer <a href="https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=311"> this </a>)
struct __attribute__((__packed__)) NAV_POSLLH {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch	(ms)
	int32_t		lon;					//!< longitude	* 1e-7							(deg)
	int32_t		lat;					//!< latitude	* 1e-7							(deg)
	int32_t		height;					//!< height above ellipsoid	* 1e-3				(m)
	int32_t		hMSL;					//!< height above mean sea level * 1e-3			(m)
	uint32_t	hAcc;					//!< horizontal accuracy estimate * 1e-3		(m)
	uint32_t	vAcc;					//!< vertical accuracy estimate	* 1e-3			(m)
};

//! NAV_HPPOSLLH message struct (refer <a href="https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf#page=306"> this </a>)
struct __attribute__((__packed__)) NAV_HPPOSLLH {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		version;				//!< message version (0 for this version)
	uint8_t		rsv;					//!< reserved
	uint16_t	rsv1;					//!< reserved
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch			(ms)
	int32_t		lon;					//!< longitude	* 1e-7									(deg)
	int32_t		lat;					//!< latitude	* 1e-7									(deg)
	int32_t		height;					//!< height above ellipsoid * 1e-3						(m)
	int32_t		hMSL;					//!< height above mean sea level (MSL) * 1e-3			(m)
	int8_t		lonHp;					//!< high precision comp. of longitude * 1e-9			(deg)
	int8_t		latHp;					//!< high precision comp. of latitude * 1e-9			(deg)
	int8_t		heightHp;				//!< high precision comp. of height a. ellipsoid * 1e-4	(m)
	int8_t		hMSLHp;					//!< high precision comp. of height above MSL * 1e-4	(m) 
	uint32_t	hAcc;					//!< horizontal accuracy estimate * 1e-4				(m)
	uint32_t	vAcc;					//!< vertical accuracy estimate	*1e-4					(m)
};

struct __attribute__((__packed__)) NAV_HPPOSECEF {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		version;				//!< message version (0 for this version)
	uint8_t		rsv;					//!< reserved
	uint16_t	rsv1;					//!< reserved
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch			(ms)
	int32_t		ecefX;					//!< ECEF X coordinate * 1e-2							(m)
	int32_t		ecefY;					//!< ECEF Y coordinate * 1e-2							(m)
	int32_t		ecefZ;					//!< ECEF Z coordinate * 1e-2							(m)
	int8_t		ecefXHP;				//!< high precision comp. of ECEF X coordinate * 1e-4	(m)
	int8_t		ecefYHP;				//!< high precision comp. of ECEF Y coordinate * 1e-4	(m)
	int8_t		ecefZHP;				//!< high precision comp. of ECEF Z coordinate * 1e-4	(m)
	uint8_t		rsv2;					//!< reserved
	uint32_t	pAcc;					//!< Position Accuracy Estimate * 1e-4					(m)
};

struct __attribute__((__packed__)) NAV_STATUS_FLAGS_T {
	uint8_t gpsFixOk : 1;
	uint8_t diffSoln : 1;
	uint8_t wknSet : 1;
	uint8_t towSet : 1;
	uint8_t reserved : 4;
};

struct __attribute__((__packed__)) NAV_STATUS {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch									(ms)
	uint8_t		gpsFix;					//!< GPSfix type
	union {
		NAV_STATUS_FLAGS_T tFlags;		//!< bitfield flags struct
		uint8_t		bFlags;				//!< Navigation status flags (0:gpsFixOk, 1:diffSoln, 2:wknSet, 3:towSet)
	};
	uint8_t		fixStat;				//!< Fix Status Information
	uint8_t		flags2;					//!< further information about navigation output
	uint32_t	ttff;					//!< Time to first fix (millisecond time tag)
	uint32_t	msss;					//!< Milliseconds since Startup/Reset											(ms)
};

struct __attribute__((__packed__)) NAV_SVIN {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		version;				//!< message version (0 for this version)
	uint8_t		rsv;					//!< reserved
	uint16_t	rsv1;					//!< reserved
	uint32_t	iTOW;					//!< GPS time of week of the navigation epoch									(ms)
	uint32_t	dur;					//!< passed survey-in observation time											(s)
	int32_t		meanX;					//!< current survey in mean position ECEF X coordinate * 1e-2					(m)
	int32_t		meanY;					//!< current survey in mean position ECEF Y coordinate * 1e-2					(m)
	int32_t		meanZ;					//!< current survey in mean position ECEF Z coordinate * 1e-2					(m)
	int8_t		meanXHP;				//!< current high-precision survey in mean position ECEF X coordinate * 1e-4	(m)
	int8_t		meanYHP;				//!< current high-precision survey in mean position ECEF Y coordinate * 1e-4	(m)
	int8_t		meanZHP;				//!< current high-precision survey in mean position ECEF Z coordinate * 1e-4	(m)
	uint8_t		rsv2;					//!< reserved
	uint32_t	meanAcc;				//!< current survey-in mean position accuracy * 1e-4							(m)
	uint32_t	obs;					//!< number of position observations used during survey-in
	uint8_t		valid;					//!< survey-in position validity flag, 1=valid, otherwise 0
	uint8_t		active;					//!< survey-in progress flag, 1=in-progress, otherwise 0
	uint16_t	rsv3;					//!< reserved
};

struct __attribute__((__packed__)) RXM_PMREQ_FLAGS_T {
	uint32_t reserved : 1;			//!< reserved
	uint32_t backup : 1;			//!< The receiver goes into backup mode for a time period defined by duration
};

struct __attribute__((__packed__)) RXM_PMREQ {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint32_t	duration;				//!< duration of the requested task, set to zero for infinite duration			(ms)
	union {
		uint32_t	bFlags;				//!< task flags
		RXM_PMREQ_FLAGS_T tFlags;		//!< task flags struct
	};
};

struct __attribute__((__packed__)) CFG_PRT_TXREADY_T {
	uint16_t en : 1;
	uint16_t pol : 1;
	uint16_t pin : 5;
	uint16_t thres : 9;
};

struct __attribute__((__packed__)) CFG_PRT_UART_MODE_T {
	uint32_t reserved0 : 4;
	uint32_t reserved1 : 1;
	uint32_t reserved2 : 1;
	uint32_t charLen : 2;
	uint32_t reserved3 : 1;
	uint32_t parity : 3;
	uint32_t nStopBits : 2;
	uint32_t reserved4 : 18;
};

struct __attribute__((__packed__)) CFG_PRT_DDC_MODE_T {
	uint32_t reserved0 : 1;
	uint32_t slaveAddr : 7;
};

union  CFG_PRT_MODE_T {
	CFG_PRT_UART_MODE_T uart;
	CFG_PRT_DDC_MODE_T	ddc;
};

struct __attribute__((__packed__)) CFG_PRT_PROTOMASK_T {
	uint16_t ubx : 1;
	uint16_t nmea : 1;
};

struct __attribute__((__packed__)) CFG_PRT {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		portID;
	uint8_t		reserved0;
	union {
		uint16_t	txReady;
		CFG_PRT_TXREADY_T tTxReady;
	};
	union {
		uint32_t reserved2;
		uint32_t mode;
		CFG_PRT_MODE_T tMode;
	};
	union {
		uint32_t	baudRate;
		uint32_t	reserved3;
	};
	union {
		CFG_PRT_PROTOMASK_T tInProtoMask;
		uint16_t inProtoMask;
	};
	union {
		CFG_PRT_PROTOMASK_T tOutProtoMask;
		uint16_t outProtoMask;
	};
	uint16_t	reserved4;
	uint16_t	reserved5;
};

struct __attribute__((__packed__)) ACK_ACK {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		classID;				//!< class ID of the acknowledged message
	uint8_t		msgID;					//!< message ID of the acknowledged message
};

struct __attribute__((__packed__)) ACK_NAK {
	uint8_t		cls;					//!< class number
	uint8_t		id;						//!< ID number
	uint16_t	len;					//!< length of message
	uint8_t		classID;				//!< class ID of the not-acknowledged message
	uint8_t		msgID;					//!< message ID of the not-acknowledged message
};

//! UBX message union (to store different data types in the same memory location)
union UBXMessage {
	UBX_BASE		base;				//!< BASE UBX structure (class, id, len)
	NAV_RELPOSNED	navRelposned;		//!< NAV-RELPOSNED message member
	NAV_TIMEUTC		navTimeutc;			//!< NAV_TIMEUTC message member
	NAV_POSLLH		navPosllh;			//!< NAV_POSLLH message member
	NAV_HPPOSLLH	navHPPosllh;		//!< NAV_HPPOSLLH message member
	NAV_HPPOSECEF	navHPPosECEF;		//!< NAV_HPPOSECEF message member
	NAV_SVIN		navSvin;			//!< NAV_SVIN message member
	NAV_STATUS		navStatus;			//!< MAV_STATUS message member
	ACK_ACK			ackAck;				//!< ACK_ACK message member
	ACK_NAK			ackNak;				//!< ACK_NAK message member
	CFG_PRT			cfgPrt;				//!< CFG_PRT message member
	RXM_PMREQ		rxmPmreq;			//!< RXM_PMREQ message member
};


#endif