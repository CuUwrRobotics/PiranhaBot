#include "Hardware/Device_Adc_Mcp3008.h"

// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
// char HARDWARE_NAME[8] = "MCP3008";
//
// // Informating About The Chip Used
// // ===============================.
// const static uint8_t PIN_COUNT = 8;
// const static Device_t deviceTypeId = DEVICE_ADC;
//
// // Pin Modes That This Chip can Accept
// // ==============================================
// const static uint8_t VALID_PIN_MODE_COUNT = 1;
// const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT};
//
// // Other Variables (Don't change these)
// // ====================================
// Interface_t reservedPins[PIN_COUNT];
// // For storing tick rate (duty cycle)
// float pinValues[PIN_COUNT];

// These are specific to the MCP3008 on the interfacing board REV A
// const float ADC_STEPS = 1024;
// const float AVCC_THEORETICAL_VALUE = 5.00;


/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool Device_Adc_Mcp3008::deviceInit(){
	// Init chip here

	// Default modes and states assigned here

	// Pin modes never change, and are not in update data function, so they are
	// set here and final
	for (uint8_t i = 0; i < getPinCount(); i++) {
		requestedPinBus.setPinMode(i, MODE_INPUT);
		currentPinBus.setPinMode(i, MODE_INPUT);
	}
	updateData();
	return true;
} /* deviceInit */

/**
 */

DataError_t Device_Adc_Mcp3008::getPinValue(PinValue_t *value){
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;
	if (value->fmt == VALUE_ADC_DIRECT) { // Data from a pin
		value->data[0] = pinValues[value->pin];
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // getPinValue

/**
 * Can't set the pin value on an ADC
 */

DataError_t Device_Adc_Mcp3008::setPinValue(PinValue_t *value) {
	return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t Device_Adc_Mcp3008::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Device_Adc_Mcp3008::readDeviceConfig(DeviceConfig_t *cfg) {
	if (cfg->fmt == DCFG_ADC_STEPS) { // How many steps there are in the ADC measurements
		cfg->data[0] = ADC_STEPS;
		return ERROR_SUCCESS;
	}
	if (cfg->fmt == DCFG_ADC_AVCC_VOLTAGE) { // Voltage of the ADC
		cfg->data[0] = AVCC_THEORETICAL_VALUE;
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

/**
 * Reads data
 */

bool Device_Adc_Mcp3008::updateData(){
	if (!ready())
		return false;
	// Check if any data is readable on any pins. If so, read it.
	printf("Reading data from pins (TODO).\n");
	// READ DATA HERE, THESE ARE FOR TESTING:
	for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
		pinValues[pin] = (0.625 * ADC_STEPS / AVCC_THEORETICAL_VALUE) * (pin + 1);
	}
	// Hacky way of getting correct telemetry data
	if (deviceIndex == 7) // CURRENT 0
		pinValues[0] = 389; // -10 amps
	if (deviceIndex == 7) // CURRENT 1
		pinValues[1] = 635; // +10 amps
	if (deviceIndex == 7) // VREF calibration, should be around 3V
		pinValues[2] = 614; // 3v (no adjustemnt)
	// pinValues[2] = 635;    // 3.1v (data is adjusted)
	if (deviceIndex == 7) // TEMP 0
		pinValues[3] = 178; // 25'C
	if (deviceIndex == 7) // PL 0
		pinValues[4] = 676; // 3.3v
	if (deviceIndex == 7) // PL 1
		pinValues[5] = 1024; // 5v
	if (deviceIndex == 7) // PL 2
		pinValues[6] = 768; // 12v ==> 3.75v with resitor divider
	if (deviceIndex == 7) // PL 3
		pinValues[7] = 819; // 48v ==> 4v with resitor divider
	return true;
} // updateData
