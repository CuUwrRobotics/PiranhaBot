#include "Interface_Voltage_Div.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 1;
// // IDs which indicate what this is and what it should be connected to
// const static Interface_t interfaceTypeId = INTF_VOLTAGE_DIVIDER;
// const static Device_t parentDeviceTypeId = DEVICE_ADC;
//
// // Calibration ratios, preset to a default offset and a tolerance +- 100%
// float avccOffsetRatio = 1; // multiply by measured to get actual.
// float avccOffsetToleranceRatio = 1; // multiply by actual to get ± tolerance value
// // For data conversions
// float adcSteps = 1024; // number of steps the ADC uses to save data (ie reading at AVCC)
// float avccTheoretical = 5.00; // Theoretical AVCC
//
// float highResistor = 1; // Higher voltage resistor in circuit
// float highResistorTolerance = 0.05; // 5% resistor if highResistor is measured, this is 0.
// float lowResistor = 1; // Lower voltage resistor in cicuit
// float lowResistorTolerance = 0.05;
// float voltageMeasured = 0; // Voltage on the power line, after calculating for resistor divider
// float toleranceVolts = 0; // Tolerance on voltageMeasured

/**
 * @return TODO
 */

bool Interface_Voltage_Div::calculateValues(){
	float voltage;
	DataError_t errorVal;
	PinValue_t val;
	// Get the voltage
	val.fmt = VALUE_ADC_DIRECT; // Set format
	val.pin = pinBus.getPin(0); // Go from local pin to the device pin
	val.data = &voltage; // Uses voltage to store data
	errorVal = commDevice->getPinValue(&val); // Get the data
	voltage = voltage * (avccTheoretical / adcSteps) * avccOffsetRatio;
	if (highResistor == 0 || lowResistor == 0) { // No resistors; no calcs needed
		voltageMeasured = voltage;
		toleranceVolts = voltage * avccOffsetToleranceRatio;
		return true;
	}
	voltageMeasured = (voltage * (highResistor + lowResistor)) / lowResistor;
	toleranceVolts = voltageMeasured * (avccOffsetToleranceRatio +
	                                    lowResistorTolerance +
	                                    highResistorTolerance); // Tolerance value
	return true;
} // calculateValues

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Voltage_Div::prepareInterface(){
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

DataError_t Interface_Voltage_Div::readPin(PinValue_t *valueIn) {
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
	case VALUE_ADC_VOLTAGE_WITH_TOLERANCE:
		if (!calculateValues()) {
			return ERROR_INTF_N_READY;
		}
		valueIn->data[0] = voltageMeasured;
		valueIn->data[1] = toleranceVolts;
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readPin

DataError_t Interface_Voltage_Div::writePin(PinValue_t *value) {
	return ERROR_NOT_AVAIL;
} /* writePin */

DataError_t Interface_Voltage_Div::writeConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		cfg->data[0] = avccOffsetRatio;
		cfg->data[1] = avccOffsetToleranceRatio;
		return ERROR_SUCCESS;
		break;
	case ICFG_PL_LOW_RESISTOR_WITH_TOLERANCE:
		lowResistor = cfg->data[0];
		lowResistorTolerance = cfg->data[1];
		return ERROR_SUCCESS;
		break;
	case ICFG_PL_HIGH_RESISTOR_WITH_TOLERANCE:
		highResistor = cfg->data[0];
		highResistorTolerance = cfg->data[1];
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // writeConfig

DataError_t Interface_Voltage_Div::readConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		avccOffsetRatio = cfg->data[0];
		avccOffsetToleranceRatio = cfg->data[1];
		return ERROR_SUCCESS;
		break;
	case ICFG_PL_LOW_RESISTOR_WITH_TOLERANCE:
		cfg->data[0] = lowResistor;
		cfg->data[1] = lowResistorTolerance;
		return ERROR_SUCCESS;
		break;
	case ICFG_PL_HIGH_RESISTOR_WITH_TOLERANCE:
		cfg->data[0] = highResistor;
		cfg->data[1] = highResistorTolerance;
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readConfig

DataError_t Interface_Voltage_Div::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Voltage_Div::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

uint8_t Interface_Voltage_Div::setPinMode(uint8_t pinNumber, PinMode_t pinMode){
	ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
	         interfaceIdToCharArray(interfaceTypeId));
	return 0;
} /* setPinMode */
