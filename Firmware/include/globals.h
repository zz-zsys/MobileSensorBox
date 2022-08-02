#ifndef _GLOBALS_H
#define _GLOBALS_H

#include <Arduino.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

/* packing of structure members */
#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE
#define __PACKED_POST  __attribute__ ((packed))
#endif
#endif

extern std::string wpsSsid;
extern std::string wpsPassword;
extern std::string authBearer;

/************************************************************************************************************************/
/*!						  .oooooo.     .oooooo.   ooooo      ooo oooooooooooo ooooo   .oooooo.
						 d8P'  `Y8b   d8P'  `Y8b  `888b.     `8' `888'     `8 `888'  d8P'  `Y8b
						888          888      888  8 `88b.    8   888          888  888
						888          888      888  8   `88b.  8   888oooo8     888  888
						888          888      888  8     `88b.8   888    "     888  888     ooooo
						`88b    ooo  `88b    d88'  8       `888   888          888  `88.    .88'
						 `Y8bood8P'   `Y8bood8P'  o8o        `8  o888o        o888o  `Y8bood8P'
*/
/************************************************************************************************************************/

/************************************************************************************************************************/
/*!									   ____ ____ ____ ____ ____ ____ ____ ____ ____
									  ||C |||O |||N |||V |||E |||R |||T |||E |||R ||
									  ||__|||__|||__|||__|||__|||__|||__|||__|||__||
									  |/__\|/__\|/__\|/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define uS_TO_S_FACTOR					1000000		// Conversion factor for micro seconds to seconds
#define mS_TO_S_FACTOR					1000		// Conversion factor for milli seconds to seconds

/************************************************************************************************************************/
/*!													  ____ ____ ____
													 ||G |||P |||S ||
													 ||__|||__|||__||
													 |/__\|/__\|/__\|
*/
/************************************************************************************************************************/
// Serial port number definition
#define GPS_SERIAL_PORT_NR				1			// Serial1 for gps serial port number
#define GPS_SEND_INTERVAL_MS			500			// GPS packet send interval to LoRaWAN queue in ms

/************************************************************************************************************************/
/*!			____ ____ ____ _________ ____ ____ ____ ____ ____ ____ ____ _________ ____ ____ ____ ____ ____ ____
		   ||N |||O |||2 |||       |||D |||I |||G |||I |||T |||A |||L |||       |||S |||E |||N |||S |||O |||R ||
		   ||__|||__|||__|||_______|||__|||__|||__|||__|||__|||__|||__|||_______|||__|||__|||__|||__|||__|||__||
		   |/__\|/__\|/__\|/_______\|/__\|/__\|/__\|/__\|/__\|/__\|/__\|/_______\|/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define DGS_SERIAL_PORT_NR				2			// Serial2 for NO2 sensor serial port number
#define DGS_SEND_INTERVAL_MS			1000		// DGS packet send interval to LoRAWAN queue in ms

/************************************************************************************************************************/
/*!											____ ____ ____ ____ ____ ____ ____ 
                                           ||L |||o |||R |||a |||W |||A |||N ||
                                           ||__|||__|||__|||__|||__|||__|||__||
                                           |/__\|/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define CLOCK_ERROR_PROCENTAGE			5			// For LMIC setClockError function. The higher, the bigger
													// LMIC receive windows. Reduce it, if limited on battery
#define LORA_GPS_DISTANCE_THRESHOLD		50.0		// GPS distance threshold in meter for LoRaWAN packet sending
#define LORA_SPREAD_FACTOR				10			// LoRa spread factor

#define LORA_MIN_SEND_INTERVAL_MS		10000		// LoRa min. send packet interval (http://tiny.cc/mlm56y)
//#define LORA_SEND_INTERVAL_S			300			// LoRa send packet interval in seconds
#define LORA_SEND_INTERVAL_S			60			// LoRa send packet interval in seconds			
#define LORA_SEND_INTERVAL_MS			LORA_SEND_INTERVAL_S * 	mS_TO_S_FACTOR	

/************************************************************************************************************************/
/*!											   ____ ____ ____ ____ ____ ____
											  ||B |||M |||E |||2 |||8 |||0 ||
											  ||__|||__|||__|||__|||__|||__||
											  |/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define BME_SEND_INTERVAL_MS			500			// BME send packet interval in ms 

/************************************************************************************************************************/
/*!												 ____ ____ ____ ____ ____
												||S |||P |||S |||3 |||0 ||
												||__|||__|||__|||__|||__||
												|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define SPS30_SEND_INTERVAL_MS			1000		// SPS30 send packet interval in ms
#define AUTOSTART_FAN_CLEANING			1			// autostart fan cleaning on startup (0: disable, 1: enable)

/************************************************************************************************************************/
/*!											____ ____ ____ ____ ____ ____ ____
										   ||D |||I |||S |||P |||L |||A |||Y ||
										   ||__|||__|||__|||__|||__|||__|||__||
										   |/__\|/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define DISPLAY_REFRESH_TIME_MS			250			// display refresh time in ms

/************************************************************************************************************************/
/*!									 ____ ____ ____ _________ ____ ____ ____ ____ ____
									||L |||O |||W |||       |||P |||O |||W |||E |||R ||
									||__|||__|||__|||_______|||__|||__|||__|||__|||__||
									|/__\|/__\|/__\|/_______\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define LP_TIME_TO_CHECK_EVENT			600			// low power watchdog event check interval in seconds
#define LP_TIME_TO_SLEEP				0			// deep sleep time in seconds
#define LP_EVENT_INTERVAL_TIME			LP_TIME_TO_CHECK_EVENT * mS_TO_S_FACTOR / portTICK_PERIOD_MS
#define LP_SLEEP_TIME					uS_TO_S_FACTOR * LP_TIME_TO_SLEEP

/************************************************************************************************************************/
/*!										  ____ ____ ____ ____ ____ ____ ____ ____
										 ||W |||A |||T |||C |||H |||D |||O |||G ||
										 ||__|||__|||__|||__|||__|||__|||__|||__||
										 |/__\|/__\|/__\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define WD_TIME_TO_CHECK_EVENT			10			// watchdog event check interval in seconds
#define WD_EVENT_INTERVAL_TIME			WD_TIME_TO_CHECK_EVENT * mS_TO_S_FACTOR / portTICK_PERIOD_MS

/************************************************************************************************************************/
/*!												 ____ ____ ____ ____ ____
												||D |||E |||B |||U |||G ||
												||__|||__|||__|||__|||__||
												|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define GPS_DEBUG						false
#define SPS30_DEBUG						false
#define SPS30_LIBDEBUG					false
#define DGS_DEBUG						false
#define DGS_LIBDEBUG					false
#define BME_DEBUG						false
#define LORAWAN_DEBUG					false
#define BATTERY_DEBUG					true
#define DISPLAY_DEBUG					false
#define LP_DEBUG						false
#define DEBUG_SEND_PACKET				false
#define SEND_GPS_PACKET_WHEN_FIX_ONLY	0			// GPS packet sending condition (0: unconditional, 1: fix only

/************************************************************************************************************************/
/*!													____ ____ ____ ____
												   ||M |||I |||S |||C ||
												   ||__|||__|||__|||__||
												   |/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/
#define MAX_PAYLOAD_SIZE				50			// max payload size for message buffer structure
#define SEND_QUEUE_SIZE					10			// queue size

/************************************************************************************************************************/
/*!			  oooooooooooo                              ooooooooo.   ooooooooooooo   .oooooo.    .oooooo..o
			  `888'     `8                              `888   `Y88. 8'   888   `8  d8P'  `Y8b  d8P'    `Y8
			   888         oooo d8b  .ooooo.   .ooooo.   888   .d88'      888      888      888 Y88bo.
			   888oooo8    `888""8P d88' `88b d88' `88b  888ooo88P'       888      888      888  `"Y8888o.
			   888    "     888     888ooo888 888ooo888  888`88b.         888      888      888      `"Y88b
			   888          888     888    .o 888    .o  888  `88b.       888      `88b    d88' oo     .d8P
			  o888o        d888b    `Y8bod8P' `Y8bod8P' o888o  o888o     o888o      `Y8bood8P'  8""88888P'
*/
/************************************************************************************************************************/
/* Task Handle */
extern TaskHandle_t xTaskBme;						// task handle for BME280 task
extern TaskHandle_t xTaskDisplay;					// task handle for display task
extern TaskHandle_t xTaskSps;						// task handle for SPS30 task
extern TaskHandle_t xTaskDgs;						// task handle for DGS NO2 sensor task
extern TaskHandle_t xTaskGps;						// task handle for GPS task
extern TaskHandle_t xTaskLora;						// task handle for LoRaWAN task
extern TaskHandle_t xTaskWatchdog;					// task handle for watchdog task
extern TaskHandle_t xTaskLp;						// task handle for low power task
extern TaskHandle_t xTaskBattery;					// task handle for battery reading task
extern TaskHandle_t xTaskOta;						// task handle for OTA update function						

/* Semaphore for the task */
extern SemaphoreHandle_t xSemaphoreI2C;				// semaphore handle for I2C task (for I2C resource sharing)
extern SemaphoreHandle_t xSemaphoreUart;			// semaphore handle for UART task (not used at the moment)
extern SemaphoreHandle_t xSemaphoreSpi;				// semaphore handle for SPI task (not used at the moment)
extern SemaphoreHandle_t xSemaphoreBattAdc;			// semaphore handle for battery reading task;


// I2C bus sharing access control via semaphore
//#define I2C_MUTEX_LOCK()		xSemaphoreTake(xSemaphoreI2C, pdMS_TO_TICKS(250)) == pdTRUE
#define I2C_MUTEX_LOCK()		xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY) == pdTRUE
#define I2C_MUTEX_UNLOCK()		xSemaphoreGive(xSemaphoreI2C)
#define BATT_ADC_MUTEX_LOCK()	do {} while (xSemaphoreTake(xSemaphoreBattAdc, portMAX_DELAY) != pdTRUE)
#define BATT_ADC_MUTEX_UNLOCK()	xSemaphoreGive(xSemaphoreBattAdc)
#define DISPLAY_MUTEX_LOCK()	I2C_MUTEX_LOCK()
#define DISPLAY_MUTEX_UNLOCK()	I2C_MUTEX_UNLOCK()
#define SPS30_MUTEX_LOCK()		I2C_MUTEX_LOCK()
#define SPS30_MUTEX_UNLOCK()	I2C_MUTEX_UNLOCK()
#define BME_MUTEX_LOCK()		I2C_MUTEX_LOCK()
#define BME_MUTEX_UNLOCK()		I2C_MUTEX_UNLOCK()

/* Queue for the Intertask communication */
extern QueueHandle_t xQueueBme;						// queue handle for BME280
extern QueueHandle_t xQueueSps;						// queue handle for SPS30		
extern QueueHandle_t xQueueDgs;						// queue handle for DGS NO2 sensor
extern QueueHandle_t xQueueGps;						// queue handle for GPS 
extern QueueHandle_t xQueueLora;					// queue handle for LoRaWAN
extern QueueHandle_t xQueueDisplay;					// queue handle for display

// Event group handle for task watchdog 
extern EventGroupHandle_t xWatchdogEvent;			// event group handle for watchdog task
extern EventGroupHandle_t xLowPowerEvent;			// event group handle for low power task

/* Event ID for the watchdog group event function */
const EventBits_t lorawanWdEventId	= (1 << 0);		// LoRaWAN event ID
const EventBits_t bmeWdEventId		= (1 << 1);		// BME280 event ID 
const EventBits_t spsWdEventId		= (1 << 2);		// SPS30 event ID 
const EventBits_t dgsWdEventId		= (1 << 3);		// DGS event ID 
const EventBits_t gpsWdEventId		= (1 << 4);		// GPS event ID 
const EventBits_t displayWdEventId	= (1 << 5);		// Display event ID 

// Event ID mask for all member in watchdog group event
const EventBits_t allWdEventId = (lorawanWdEventId | bmeWdEventId | spsWdEventId | dgsWdEventId	| gpsWdEventId | displayWdEventId);	

/* Event ID for the low power group event function */
const EventBits_t gpsLpEventId = (1 << 0);			// GPS event id

// event ID mask for all member in low power group event
const EventBits_t allLpEventId = (gpsLpEventId);

/************************************************************************************************************************/
/*!			  
 __     __         _       _     _          ____  _                   _                      _   _       _
  \ \   / /_ _ _ __(_) __ _| |__ | | ___    / ___|| |_ _ __ _   _  ___| |_ _   _ _ __ ___    | | | |_ __ (_) ___  _ __
   \ \ / / _` | '__| |/ _` | '_ \| |/ _ \   \___ \| __| '__| | | |/ __| __| | | | '__/ _ \   | | | | '_ \| |/ _ \| '_ \
	\ V / (_| | |  | | (_| | |_) | |  __/_   ___) | |_| |  | |_| | (__| |_| |_| | | |  __/_  | |_| | | | | | (_) | | | |
	 \_/ \__,_|_|  |_|\__,_|_.__/|_|\___( ) |____/ \__|_|   \__,_|\___|\__|\__,_|_|  \___( )  \___/|_| |_|_|\___/|_| |_|
										|/                                               |/
										____        __ _       _ _   _
									   |  _ \  ___ / _(_)_ __ (_) |_(_) ___  _ __
									   | | | |/ _ \ |_| | '_ \| | __| |/ _ \| '_ \
									   | |_| |  __/  _| | | | | | |_| | (_) | | | |
									   |____/ \___|_| |_|_| |_|_|\__|_|\___/|_| |_|
*/
/************************************************************************************************************************/
/* flag for the watchdog function*/
extern bool isLorawanStopResponding;
extern bool isBmeStopResponding;
extern bool isSpsStopResponding;
extern bool isDgsStopResponding;
extern bool isGpsStopResponding;
extern bool isDisplayStopResponding;
extern bool isLowPowerActivated;

// time zone variables
extern const char* tz_berlin;
extern const char* tz_london;
extern const char* tz_utc;

//! NAV-TIMEUTC variables
extern uint16_t year;
extern uint8_t	month, day, hour, minute, seconds;

//! NAV-STATUS variables;
extern bool timeValid;
extern bool gpsValid;

//! NAV-POSLLH variables
extern float lat, lon, altitude;

//! Flag for UBX message read
extern bool flagTimeUTC, flagPosLLH, flagStatus;

extern time_t lastConfigTime;
extern bool isTimeSet;
extern time_t currentTime;

typedef union BYTE2FLOAT_Ttag {							// byte array float structure
	float fValue;
	uint8_t abValue[4];
	uint32_t ulValue;
} BYTE2FLOAT_T;

typedef  __PACKED_PRE struct BME_PACKET_STRUCT_Ttag {	// BME280 packet structure
	int16_t			sTemperature_x100;					// ambient temperature, SF 0.01			[°C]
	uint8_t			bHumidity;							// ambient humidity						[%RH]
	uint16_t		usPressure_hPa_x10;					// ambient pressure, SF 0.1				[hPa] 
} __PACKED_POST BME_PACKET_STRUCT_T;

typedef union BME_PACKET_Ttag {							// BME280 packet union
	BME_PACKET_STRUCT_T tPacket;
	uint8_t	abPacket[sizeof(BME_PACKET_STRUCT_T)];
} BME_PACKET_T;

typedef  __PACKED_PRE struct SPS_PACKET_STRUCT_Ttag {	// SPS30 packet structure
	float		MassPM2;								// mass concentration PM2.5				[μg/m3]
	float		MassPM10;								// mass concentration PM10				[μg/m3]
	uint16_t	usAvgParticleSize_x1000;				// typical particle size, SF 0.001		[μm]
} __PACKED_POST SPS_PACKET_STRUCT_T;

typedef union SPS_PACKET_Ttag {							// SPS30 packet union
	SPS_PACKET_STRUCT_T tPacket;
	uint8_t	abPacket[sizeof(SPS_PACKET_STRUCT_T)];
} SPS_PACKET_T;

typedef __PACKED_PRE struct DGS_PACKET_STRUCT_Ttag {	// DGS NO2 sensor packet structure
	int16_t sPpb;										// number of concentration of NO2 pollutant as part per billion 
	uint16_t usPpbRaw;									// raw ADC value from NO2 sensor
} __PACKED_POST DGS_PACKET_STRUCT_T;

typedef union DGS_PACKET_Ttag {							// DGS NO2 sensor packet union
	DGS_PACKET_STRUCT_T tPacket;						
	uint8_t abPacket[sizeof(DGS_PACKET_STRUCT_T)];
} DGS_PACKET_T;

typedef __PACKED_PRE struct GPS_PACKET_STRUCT_Ttag {	// GPS packet structure
	float fLatitude;									// latitude								[°]
	float fLongitude;									// longitude							[°]
	int16_t sAltitude_x100;								// altitude, SF 0.01					[m]
} __PACKED_POST GPS_PACKET_STRUCT_T;

typedef union GPS_PACKET_Ttag {							// GPS packet union
	GPS_PACKET_STRUCT_T tPacket;
	uint8_t abPacket[sizeof(GPS_PACKET_STRUCT_T)];
} GPS_PACKET_T;

typedef __PACKED_PRE struct LORA_PACKET_STRUCT_Ttag {	// LoRaWAN packet structure
	BME_PACKET_T	tBmePacket;							// BME280 packet
	DGS_PACKET_T	tDgsPacket;							// DGS packet
	GPS_PACKET_T	tGpsPacket;							// GPS packet
	SPS_PACKET_T	tSpsPacket;							// SPS30 packet
	uint16_t		usBatteryStatus;					// battery voltage						[mV]
}__PACKED_POST LORA_PACKET_STRUCT_T;


typedef union LORA_PACKET_Ttag {
	LORA_PACKET_STRUCT_T tPacket;
	uint8_t abPacket[sizeof(LORA_PACKET_STRUCT_T)];
} LORA_PACKET_T;

typedef __PACKED_PRE struct MESSAGE_BUFFER_Ttag {		// Message buffer structure
	uint8_t bPort;										// port number (refer to eDisplayDataPort_t)
	uint8_t bMessageSize;								// message size
	uint8_t abPacket[MAX_PAYLOAD_SIZE];					// message packet
}__PACKED_POST MESSAGE_BUFFER_T;

typedef enum eDisplayDataPort_tTag {					// data port enumeration for display queue
	BME_DATAPORT,
	SPS_DATAPORT,
	DGS_DATAPORT,
	GPS_DATAPORT
} eDisplayDataPort_t;

typedef __PACKED_PRE struct GPIO_Ttag {
	const gpio_num_t	pin;
	uint8_t				mode;
	uint8_t				intMode;
	volatile bool		pressed;
	volatile bool		release;
}__PACKED_POST GPIO_T;

extern uint16_t battery_read_voltage();
extern void lora_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);
extern void bme_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);
extern void sps_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);
extern void dgs_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);
extern void gps_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);
extern void watchdog_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID);

#endif