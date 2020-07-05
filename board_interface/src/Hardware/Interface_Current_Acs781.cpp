// Generic headers
 #include "HwHeader.h"
 #include "Devices_interfaces.h"

// Specific to the device
// #include "Interface_Current_Acs781.h"

/**
 * Interface_Adc
 * @author
 */
class Interface_Current_Acs781 : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[15] = "ACS781_CURRENT"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 1;
// IDs which indicate what this is and what it should be connected to
const static uint8_t interfaceTypeId = HardwareDescriptor::INTF_CURRENT;
const static uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_ADC;

// Calibration ratios, preset to a default offset and a tolerance +- 100%
float avccOffsetRatio = 1; // multiply by measured to get actual.
float avccOffsetToleranceRatio = 1; // multiply by actual to get ± tolerance value
// For data conversions
float adcSteps = 1024; // number of steps the ADC uses to save data (ie reading at AVCC)
float avccTheoretical = 5.00; // Theoretical AVCC

// Sensor Specific
// ===============
// The linear analog output of the current sensor
const float VOLTS_PER_AMP = 0.060; // V/A for the ACS780LLRTR-050U
// Tolerance of the current device, ideally at ~25'C, in amps
const float CURRENT_TOLERANCE_VOLTS = 0.054;
// For bidiectional versions (***B) = VCC/2
// For unidirectional versions (***U) = VCC * 0.1
const float CURRENT_ZERO_OFFSET_VOLTS = 2.5; // Bidirectional setting, = 5/2
// const float CURRENT_ZERO_OFFSET_VOLTS = 0.5; // Unidirectional setting, = 1/5

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
		data[0] = commDevice->getPinValue(pinBus.getPin(pin),
		                                  PACKET_ADC_DIRECT);
		return data;
	}
	// Gets data, corrects for AVCC, then scales to current in amps.
	if (dataType == PACKET_CURRENT_AMPS) {
		data[0] = commDevice->getPinValue(pinBus.getPin(pin),
		                                  PACKET_ADC_DIRECT);
		data[0] = (data[0] * (avccTheoretical / adcSteps) * avccOffsetRatio) /
		          VOLTS_PER_AMP;
		data[0] -= CURRENT_ZERO_OFFSET_VOLTS; // Zero offset correction
		return data;
	}
	if (dataType == PACKET_CURRENT_AMPS_WITH_TOLERANCE) {
		data[0] = commDevice->getPinValue(pinBus.getPin(pin),
		                                  PACKET_ADC_DIRECT);
		data[0] = data[0] * (avccTheoretical / adcSteps) * avccOffsetRatio; // Voltage
		// Get tolerance in volts
		data[1] = (data[0] * avccOffsetToleranceRatio) + CURRENT_TOLERANCE_VOLTS;
		// Convert data to amps
		data[0] -= CURRENT_ZERO_OFFSET_VOLTS; // Zero offset correction
		data[0] /= VOLTS_PER_AMP; // Set to amps
		data[1] /= VOLTS_PER_AMP;
		return data; // Return packed data
	}
	if (dataType == PACKET_ADC_OFFSET_AND_TOLERANCE_RATIOS) {
		// Don't go to device for this one
		data[0] = avccOffsetRatio;
		data[1] = avccOffsetToleranceRatio;
		return data;
	}
	ROS_ERROR("readPin for ADC got a bad datatype: ", dataType);
	return 0;
} // readPin

// **** THESE OVERRIDE OVER PARENT CLASS ****

uint8_t writePin(uint8_t pinNumber, float *data, DataType dataType,
                 uint64_t hd){
	// Technically not a pin or output, but this sets the actual analog voltage
	// supply level for calculating real voltage levels.
	if (dataType == PACKET_ADC_OFFSET_AND_TOLERANCE_RATIOS) {
		avccOffsetRatio = data[0]; 
		avccOffsetToleranceRatio = data[1]; 
		return 1;
	}
	ROS_INFO(
		"writePin: Other than AVCC voltage calibration, data cannot be written to the ADC interface!");
	return 0;
} /* writePin */

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param hd TODO
 * @return TODO
 */

uint8_t setPinMode(uint8_t pinNumber, PinMode pinMode, uint64_t hd){
	ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
	         INTERFACE_NAME);
	return 0;
} /* setPinMode */
}

;
