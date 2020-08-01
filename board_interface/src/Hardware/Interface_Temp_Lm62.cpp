#include "Interface_Temp_Lm62.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 1;
// // IDs which indicate what this is and what it should be connected to
// const static Interface_t interfaceTypeId = INTF_TEMP;
// const static Device_t parentDeviceTypeId = DEVICE_ADC;
//
// // Calibration ratios, preset to a default offset and a tolerance +- 100%
// float avccOffsetRatio = 1; // multiply by measured to get actual.
// float avccOffsetToleranceRatio = 1; // multiply by actual to get ± tolerance value
// // For data conversions
// float adcSteps = 1024; // number of steps the ADC uses to save data (ie reading at AVCC)
// float avccTheoretical = 5.00; // Theoretical AVCC
//
// // Sensor Specific
// // ===============
// // For conversion to temperature (TODO: Move to YAML config)
// const float VOLTS_PER_DEG_C = 0.0156; // V/'C
// const float SENSOR_TOLERANCE_VOLTS = 0.0486; // +-3'C for LM62C
// const float SENSOR_ZERO_C_OFFSET_VOLTS = 0.480; // V

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Temp_Lm62::prepareInterface(){
	pinBus.setAllPins(MODE_INPUT);
	commDevice->setPinModes(pinBus);
	// Get conversion values from the ADC device
	DeviceConfig_t cfg;
	DataError_t errorVal;
	// Collect the ADC steps value
	cfg.fmt = DCFG_ADC_STEPS;
	cfg.data = &adcSteps; // Assigns value to adcSteps
	errorVal = commDevice->readDeviceConfig(&cfg);
	if (!(errorVal == ERROR_SUCCESS))
		log_error("Interface #%d Could not get DCFG_ADC_STEPS from device: %s",
		          interfaceIndex, errorCharArray(errorVal));

	// Collect the ADC AVCC voltage value
	cfg.fmt = DCFG_ADC_AVCC_VOLTAGE;
	cfg.data = &avccTheoretical; // Assigns value to avccTheoretical
	errorVal = commDevice->readDeviceConfig(&cfg);
	if (!(errorVal == ERROR_SUCCESS))
		log_error(
			"Interface #%d Could not get DCFG_ADC_AVCC_VOLTAGE from device: %s",
			interfaceIndex, errorCharArray(errorVal));
} // prepareInterface

DataError_t Interface_Temp_Lm62::readPin(PinValue_t *valueIn) {
	if (!(valueIn->pin >= 0 && valueIn->pin < PIN_COUNT))
		return ERROR_INTF_PIN_INVALID;

	PinValue_t val;
	DataError_t errorVal;
	// Reads the pin on the device. Formatting/scaling/other data changes happen below.
	val.fmt = VALUE_ADC_DIRECT; // Set format
	val.pin = pinBus.getPin(valueIn->pin); // Go from local pin to the device pin
	val.data = valueIn->data; // Uses input data to store data
	errorVal = commDevice->getPinValue(&val); // Get the data

	// Format data and return with the error/success code from device
	switch (valueIn->fmt) {
	case VALUE_ADC_DIRECT:
		return errorVal;
		break;
	case VALUE_TEMP_C_WITH_TOLERANCE:
		valueIn->data[0] = valueIn->data[0] * (avccTheoretical / adcSteps) *
		                   avccOffsetRatio; // Actual voltage
		valueIn->data[1] = valueIn->data[0] * avccOffsetToleranceRatio +
		                   SENSOR_TOLERANCE_VOLTS; // Tolerance value
		valueIn->data[0] -= SENSOR_ZERO_C_OFFSET_VOLTS;
		valueIn->data[0] /= VOLTS_PER_DEG_C; // Set voltage to temp in deg C
		valueIn->data[1] /= VOLTS_PER_DEG_C; // Set voltage to temp in deg C
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readPin

DataError_t Interface_Temp_Lm62::writeConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		cfg->data[0] = avccOffsetRatio;
		cfg->data[1] = avccOffsetToleranceRatio;
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // writeConfig

DataError_t Interface_Temp_Lm62::readConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		avccOffsetRatio = cfg->data[0];
		avccOffsetToleranceRatio = cfg->data[1];
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readConfig

DataError_t Interface_Temp_Lm62::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Temp_Lm62::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param hd TODO
 * @return TODO
 */

uint8_t Interface_Temp_Lm62::setPinMode(uint8_t pinNumber, PinMode_t pinMode){
	ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
	         interfaceIdToCharArray(interfaceTypeId));
	return 0;
} /* setPinMode */
