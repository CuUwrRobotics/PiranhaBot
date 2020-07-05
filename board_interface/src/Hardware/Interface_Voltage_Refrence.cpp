// Generic headers
 #include "HwHeader.h"
 #include "Devices_interfaces.h"

// Specific to the device
// #include "Device_Adc_Mcp3008.h"

/**
 * Interface_Adc
 * @author
 */
class Interface_Voltage_Refrence : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[5] = "VREF"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 1;
// IDs which indicate what this is and what it should be connected to
const static uint8_t interfaceTypeId = HardwareDescriptor::INTF_VREF;
const static uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_ADC;

// For data conversions
float adcSteps = -99; // number of steps the ADC uses to save data (ie reading at AVCC)
float avccTheoretical = -99; // Theoretical AVCC
// Values for voltage offset ratio calcs
float knownDiodeVoltage = -99; // This should be measured for accuracy
float knownDiodeTolerance = -99; // 2%
float knownAdcTolerance = -99; // (5v/2^9) for a 10-bit ADC after removing LSB
float measuredDiodeVoltage = -99; // Measured voltage.
float offsetRatio = -99;
float toleranceRatio = -99; // Multiply by a voltage to get tolerance of estimate.

// How many times to measure the refrence valtage before calculating average
uint8_t measureCycles = 10;
float refMeasurementAverage = 0;

/**
 * @return If all data needed to run calcs, true.
 */

bool calculateValues() {
	if (adcSteps == -99 ||
	    avccTheoretical == -99 ||
	    knownDiodeVoltage == -99 ||
	    knownDiodeTolerance == -99 ||
	    knownAdcTolerance == -99 ||
	    measureCycles < 1) {
		ROS_ERROR("VREF error: Data setup invalid");
		return false; // Not enough data was set up, can't calc
	}
	refMeasurementAverage = 0;
	for (int i = 0; i < measureCycles; i++) {
		commDevice->updateData();
		refMeasurementAverage += commDevice->getPinValue(pinBus.getPin(0),
		                                                 PACKET_ADC_DIRECT);
	}
	refMeasurementAverage /= measureCycles; // Set to average
	measuredDiodeVoltage = refMeasurementAverage * (avccTheoretical / adcSteps);
	offsetRatio = knownDiodeVoltage / measuredDiodeVoltage; // Find ratio
	// Gets a tolerance ratio; multiply by the voltage to get tolerance.
	toleranceRatio = ((knownDiodeTolerance / knownDiodeVoltage) +
	                  (knownAdcTolerance / measuredDiodeVoltage));
	return true;
} // calculateValues

public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline uint8_t getInterfaceTypeId(){
	return interfaceTypeId;
} // getInterfaceTypeId

//
inline uint8_t getParentTypeId(){
	return parentDeviceTypeId;
} // getParentTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

//
inline char *getInterfaceName(){
	return INTERFACE_NAME;
} // getPinCount

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void prepareInterface(){
	pinBus.setAllPins(MODE_INPUT);
	commDevice->setPinModes(pinBus, interfaceTypeId);
	// Get conversion values from the ADC device
	adcSteps = commDevice->getPinValue(pinBus.getPin(0), PACKET_ADC_STEPS);
	avccTheoretical = commDevice->getPinValue(pinBus.getPin(0),
	                                          PACKET_ADC_AVCC_VOLTAGE);
} // prepareInterface

/**
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

float *readPin(uint8_t pin, DataType dataType) {
	static float data[2]; // Needs to pack multiple pieces of data sometimes
	// Requested format is available directly from device
	if (dataType == PACKET_ADC_DIRECT) {
		data[0] = commDevice->getPinValue(pinBus.getPin(pin), PACKET_ADC_DIRECT);
		return data;
	}
	// Not corrected
	if (dataType == PACKET_REF_VOLTAGE_NO_CORRECT) {
		if (!calculateValues()) {
			// Setup not ready, so just return as if meaured = true
			data[0] = knownDiodeVoltage;
			return data;
		}
		data[0] = measuredDiodeVoltage;
		return data;
	}
	if (dataType == PACKET_ADC_OFFSET_AND_TOLERANCE_RATIOS) {
		if (!calculateValues()) {
			// Setup not ready, so just return as if meaured = true
			data[0] = knownDiodeVoltage;
			data[1] = knownDiodeTolerance;
			return data;
		}
		data[0] = offsetRatio;
		data[1] = toleranceRatio;
		return data;
	}
	if (dataType == PACKET_REF_READY) {
		// Don't go to device for this one
		data[0] = (calculateValues() ? 1 : 0); // Check by running calculations
		return data;
	}
	if (dataType == PACKET_REF_NUM_CYCLES) {
		data[0] = (float)measureCycles;
		return data;
	}
	ROS_ERROR("readPin for %s got a bad datatype: ", INTERFACE_NAME, dataType);
	return 0;
} // readPin

// **** THESE OVERRIDE OVER PARENT CLASS ****

uint8_t writePin(uint8_t pinNumber, float *data, DataType dataType,
                 uint64_t hd){
	// Technically not a pin or output, but this sets the actual analog voltage
	// supply level for calculating real voltage levels.
	if (dataType == PACKET_REF_KNOWN_VOLTS_WITH_TOLERANCE) {
		knownDiodeVoltage = data[0];
		knownDiodeTolerance = data[1];
		return 1;
	}
	if (dataType == PACKET_REF_ADC_TOLERANCE) {
		knownAdcTolerance = data[0];
		return 1;
	}
	if (dataType == PACKET_REF_NUM_CYCLES) {
		measureCycles = (uint8_t)data[0];
		return 1;
	}
	if (dataType == PACKET_ADC_OFFSET_AND_TOLERANCE_RATIOS) {
		// This interface will recieve this because it is connected to an ADC, but
		// does not need it. Just ignore it, without sending below error.
		return 1;
	}
	ROS_INFO(
		"writePin: Other than setup data, data cannot be written to the %s interface.",
		INTERFACE_NAME);
	return 0;
} /* writePin */

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param hd TODO
 * @return TODO
 */

uint8_t setPinMode(uint8_t pinNumber, PinMode pinMode, uint64_t hd){
	ROS_INFO("setPinMode: Pin Modes cannot be written to the %s interface",
	         INTERFACE_NAME);
	return 0;
} /* setPinMode */
}

;
