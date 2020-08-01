#include "Interface_Voltage_Refrence.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 1;
// // IDs which indicate what this is and what it should be connected to
// const static Interface_t interfaceTypeId = INTF_VREF;
// const static Device_t parentDeviceTypeId = DEVICE_ADC;
//
// // For data conversions
// float adcSteps = -99; // number of steps the ADC uses to save data (ie reading at AVCC)
// float avccTheoretical = -99; // Theoretical AVCC
// // Values for voltage offset ratio calcs
// float knownDiodeVoltage = -99; // This should be measured for accuracy
// float knownDiodeTolerance = -99; // 2%
// float knownAdcTolerance = -99; // (5v/2^9) for a 10-bit ADC after removing LSB
// float measuredDiodeVoltage = -99; // Measured voltage.
// float offsetRatio = -99;
// float toleranceRatio = -99; // Multiply by a voltage to get tolerance of estimate.
//
// // How many times to measure the refrence valtage before calculating average
// uint8_t measureCycles = 10;
// float refMeasurementAverage = 0;

/**
 * @return If all data needed to run calcs, true.
 */

bool Interface_Voltage_Refrence::calculateValues() {
	if (adcSteps == -99 ||
	    avccTheoretical == -99 ||
	    knownDiodeVoltage == -99 ||
	    knownDiodeTolerance == -99 ||
	    knownAdcTolerance == -99 ||
	    measureCycles < 10) {
		log_error("Interface #%d: Data setup invalid", interfaceIndex);
		return false; // Not enough data was set up, can't calc
	}
	refMeasurementAverage = 0;
	float adcMeasured; // Data read by device is packed in here
	PinValue_t val; // For pin values
	DataError_t errorVal;
	// For devicedata reads
	val.fmt = VALUE_ADC_DIRECT; // Set format
	val.pin = pinBus.getPin(0); // Go from local pin to the device pin
	val.data = &adcMeasured; // Uses adcMeasured to store data

	for (int i = 0; i < measureCycles; i++) {
		commDevice->updateData();
		errorVal = commDevice->getPinValue(&val); // Get the data
		if (!(errorVal == ERROR_SUCCESS)) {
			log_error("Interface #%d: getPinValue from commDevice failed: %s",
			          interfaceIndex, errorCharArray(errorVal));
			return false;
		}
		refMeasurementAverage += adcMeasured;
	}
	refMeasurementAverage /= measureCycles; // Set to average
	measuredDiodeVoltage = refMeasurementAverage * (avccTheoretical / adcSteps);
	offsetRatio = knownDiodeVoltage / measuredDiodeVoltage; // Find ratio
	// Gets a tolerance ratio; multiply by the voltage to get tolerance.
	toleranceRatio = ((knownDiodeTolerance / knownDiodeVoltage) +
	                  (knownAdcTolerance / measuredDiodeVoltage));
	return true;
} // calculateValues

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Voltage_Refrence::prepareInterface(){
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

DataError_t Interface_Voltage_Refrence::readPin(PinValue_t *valueIn) {
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
	case VALUE_REF_VOLTAGE_NO_CORRECT:
		if (!calculateValues()) {
			return ERROR_INTF_N_READY;
		}
		valueIn->data[0] = measuredDiodeVoltage;
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readPin

DataError_t Interface_Voltage_Refrence::writePin(PinValue_t *valueIn) {
	return ERROR_NOT_AVAIL;
} /* writePin */

DataError_t Interface_Voltage_Refrence::writeConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		// This interface will recieve this because it connects to an ADC, but
		// does not use it.
		return ERROR_SUCCESS;
		break;
	case ICFG_REF_KNOWN_VOLTS_WITH_TOLERANCE:
		knownDiodeVoltage = cfg->data[0];
		knownDiodeTolerance = cfg->data[1];
		return ERROR_SUCCESS;
		break;
	case ICFG_REF_ADC_TOLERANCE: // TODO: get from ADC
		knownAdcTolerance = cfg->data[0];
		return ERROR_SUCCESS;
		break;
	case ICFG_REF_NUM_CYCLES:
		measureCycles = (uint8_t)cfg->data[0];
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // writeConfig

DataError_t Interface_Voltage_Refrence::readConfig(InterfaceConfig_t *cfg) {
	switch (cfg->fmt) {
	case ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS:
		if (!calculateValues()) {
			return ERROR_INTF_N_READY;
		}
		cfg->data[0] = offsetRatio;
		cfg->data[1] = toleranceRatio;
		return ERROR_SUCCESS;
		break;
	case ICFG_REF_READY:
		// same if statement as used by \ref calculateValues()
		if (adcSteps == -99 ||
		    avccTheoretical == -99 ||
		    knownDiodeVoltage == -99 ||
		    knownDiodeTolerance == -99 ||
		    knownAdcTolerance == -99 ||
		    measureCycles < 10)
			return ERROR_INTF_N_READY;
		return ERROR_SUCCESS;
		break;
	case ICFG_REF_NUM_CYCLES:
		cfg->data[0] = (float)measureCycles;
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readConfig

DataError_t Interface_Voltage_Refrence::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Voltage_Refrence::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param hd TODO
 * @return TODO
 */

uint8_t Interface_Voltage_Refrence::setPinMode(uint8_t pinNumber, PinMode_t
                                               pinMode){
	ROS_INFO("setPinMode: Pin Modes cannot be written to the %s interface",
	         interfaceIdToCharArray(interfaceTypeId));
	return 0;
} /* setPinMode */
