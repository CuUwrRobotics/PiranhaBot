 #include "Interface_Current_Acs781.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 1;
// // IDs which indicate what this is and what it should be connected to
// const static Interface_t interfaceTypeId = INTF_CURRENT;
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
// // The linear analog output of the current sensor
// const float VOLTS_PER_AMP = 0.060; // V/A for the ACS780LLRTR-050U
// // Tolerance of the current device, ideally at ~25'C, in amps
// const float CURRENT_TOLERANCE_VOLTS = 0.054;
// // For bidiectional versions (***B) = VCC/2
// // For unidirectional versions (***U) = VCC * 0.1
// const float CURRENT_ZERO_OFFSET_VOLTS = 2.5; // Bidirectional setting, = 5/2
// const float CURRENT_ZERO_OFFSET_VOLTS = 0.5; // Unidirectional setting, = 1/5

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Current_Acs781::prepareInterface(){
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

/**
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

DataError_t Interface_Current_Acs781::readPin(PinValue_t *valueIn) {
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
	case VALUE_CURRENT_AMPS:
		valueIn->data[0] = (valueIn->data[0] *
		                    (avccTheoretical / adcSteps) * avccOffsetRatio) /
		                   VOLTS_PER_AMP;
		valueIn->data[0] -= CURRENT_ZERO_OFFSET_VOLTS; // Zero offset correction
		return errorVal;
		break;
	case VALUE_CURRENT_AMPS_WITH_TOLERANCE:
		valueIn->data[0] = valueIn->data[0] *
		                   (avccTheoretical / adcSteps) * avccOffsetRatio; // Voltage
		// Get tolerance in volts
		valueIn->data[1] = (valueIn->data[0] * avccOffsetToleranceRatio) +
		                   CURRENT_TOLERANCE_VOLTS;
		// Convert data to amps
		valueIn->data[0] -= CURRENT_ZERO_OFFSET_VOLTS; // Zero offset correction
		valueIn->data[0] /= VOLTS_PER_AMP; // Set to amps
		valueIn->data[1] /= VOLTS_PER_AMP;
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readPin

DataError_t Interface_Current_Acs781::writePin(PinValue_t *valueIn) {
	return ERROR_NOT_AVAIL;
} /* writePin */

DataError_t Interface_Current_Acs781::writeConfig(InterfaceConfig_t *cfg) {
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

DataError_t Interface_Current_Acs781::readConfig(InterfaceConfig_t *cfg) {
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

DataError_t Interface_Current_Acs781::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Current_Acs781::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

uint8_t Interface_Current_Acs781::setPinMode(uint8_t pinNumber, PinMode_t pinMode){
	ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
	         interfaceIdToCharArray(interfaceTypeId));
	return 0;
} /* setPinMode */
