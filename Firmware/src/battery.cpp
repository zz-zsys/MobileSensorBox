#include "battery.h"
#include "led.h"

// Battery config struct initialisation
BATTERY_CONFIG_T batt_config = {
	.debug			= BATTERY_DEBUG,		// debug flag
	.adc_channel	= BAT_ADC_GPIO_CHANNEL,	// battery adc1 gpio channel, defined in battery.h 
	.atten			= ADC_ATTEN_DB_11,		// attenuation level
	.unit			= ADC_UNIT_1,			// ADC unit (only ADC_UNIT_1 is supported at the moment)
	.width			= ADC_WIDTH_BIT_12		// bit capture width
};

static HardwareSerial* debugSerial;

esp_adc_cal_characteristics_t *adc_characs = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));

TaskHandle_t xTaskBattery;

uint32_t batteryCheckTime = 0;


void battery_debug(bool debug, HardwareSerial &port) {
	batt_config.debug = debug;
	debugSerial = &port;
}

/************************************************************************************************************************/
/*!
* @brief		read the battery voltage with set sampling count
* @retval		voltage in mV
*/
/************************************************************************************************************************/
uint16_t battery_read_voltage() {

	BATT_ADC_MUTEX_LOCK();

	// multisample ADC
	uint32_t adc_reading = 0;
	for (int i = 0; i < BAT_ADC_SAMPLING_COUNT; i++) {
		adc_reading += adc1_get_raw(batt_config.adc_channel);
	}
	adc_reading /= BAT_ADC_SAMPLING_COUNT;

	// Convert raw ADC reading to voltage in mV
	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_characs);
	voltage *= BAT_VOLTAGE_DIVIDER;

	BATT_ADC_MUTEX_UNLOCK();

	return (uint16_t)voltage;
}

/************************************************************************************************************************/
/*!
* @brief		configure the ADC for battery measurement
* @retval		none
*/
/************************************************************************************************************************/
void battery_init(bool debug, HardwareSerial &port) {

	BATT_ADC_MUTEX_LOCK();

	battery_debug(debug, port);

	batt_led_init();

	// configure ADC
	ESP_ERROR_CHECK(adc1_config_width(batt_config.width));									
	ESP_ERROR_CHECK(adc1_config_channel_atten(batt_config.adc_channel, batt_config.atten));

	// characterize ADC
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
		batt_config.unit,
		batt_config.atten,
		ADC_WIDTH_BIT_12,
		DEFAULT_VREF,
		adc_characs
	);

	if (batt_config.debug) {
		// show ADC characterization base
		if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) log_i("ADC characterization based on Two Point values stored in eFuse");
		else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) log_i("ADC characterization based on reference voltage stored in eFuse");
		else log_i("ADC characterization based on default reference voltage");
	}

	BATT_ADC_MUTEX_UNLOCK();

	debugSerial->printf("Current battery voltage: %d mV\n", battery_read_voltage());


}

/************************************************************************************************************************/
/*!
* @brief		check battery level
* @retval		the battery level status (refer eBatteryStatus_t enumeration)
*/
/************************************************************************************************************************/
eBatteryStatus_t battery_checkLevel() {

	uint16_t battLevel = battery_read_voltage();

	if(batt_config.debug) debugSerial->printf("battLevel: %d mV\n", battLevel);

	if (battLevel >= MAX_BATT_THRESHOLD) return BATT_LVL_FULL;
	else if (battLevel <= MIN_BATT_THRESHOLD) return BATT_LVL_LOW;
	else return BATT_LVL_MIDDLE;
}



void batteryTask(void * parameter) {
	
	battery_debug(false);
	debugSerial->printf("Start battery task!\n");

	battery_init(BATTERY_DEBUG);

	for (;;) {

		eBatteryStatus_t battStatus = battery_checkLevel();

		if(batt_config.debug) debugSerial->printf("battStatus: %d\n", battStatus);

		switch (battStatus) {
			case BATT_LVL_FULL: {
				batt_full_led_state(LED_ON);
				batt_low_led_state(LED_OFF);
				break;
			};
			case BATT_LVL_MIDDLE: {
				batt_full_led_state(LED_OFF);
				batt_low_led_state(LED_OFF);
				break;
			};
			case BATT_LVL_LOW: {
				batt_full_led_state(LED_OFF);
				batt_low_led_state(LED_ON);
				break;
			}
		}

		vTaskDelay(60000/portTICK_PERIOD_MS);

	}
}

void battery_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(batteryTask, "batteryTask", ulStackDepth, (void*)1, uxPriority, &xTaskBattery, xCoreID);
}
