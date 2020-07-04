/*
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "PinData.h"

 #include "Device_Pwm_Pca9685.h"

/**
 * Interface_Pwm
 * @author Nicholas Steele
 */
class Interface_Pwm : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[4] = "PWM"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 16; // number of pins
uint8_t interfaceTypeId = HardwareDescriptor::INTF_PWM; // The ID for this intf
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_PWM; // The IF for the device
// ----------------------------------------------------------------------------

// uint8_t pinMapToDevice[PIN_COUNT]; // Contains a map to the devices pins.
// uint8_t pinModes[PIN_COUNT]; // Contains a map to the devices pins.
// interface based on HD
// class values
public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline uint8_t getInterfaceTypeId(){
	return interfaceTypeId;
} // getTypeId

//
inline uint8_t getParentTypeId(){
	return parentDeviceTypeId;
} // getTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/**
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

uint16_t readPin(uint8_t pin, DataType dataType) {
	// Requested format is available directly from device
	if (dataType == PACKET_PWM_FREQ || dataType == PACKET_PWM_ON_TICKS)
		return commDevice->getPinValue(pinBus.getPin(pin),
		                               dataType);
	// If program got here, data reformatting is needed from any other possible formats
	if (dataType == PACKET_INVALID) {
		ROS_ERROR("writePin: Recieved invalid dataType.\n");
		return 0;
	}
	if (dataType == PACKET_PWM_DUTY_100) {
		uint16_t pinValue = commDevice->getPinValue(pinBus.getPin(pin),
		                                            PACKET_PWM_ON_TICKS);

		// printf("Read: %d", pinValue);
		return (pinValue / (MAX_PWM_TICKS / 100));
	} else {
		ROS_ERROR("writePin: Recieved invalid dataType: %d\n", dataType);
		return 0;
	}
} // readPin

/**
 * writePin interprets a value to be assigned to a pin, then tells the parent
 * device commDevice to setPinValue using the commDevice's data conventions.
 * TODO: enumerate errors better
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
uint8_t writePin(uint8_t pinNumber, uint16_t *data, DataType dataType,
                 uint64_t hd){
	// Check that HD matches
	if (hd != getHardwareDescriptor(pinNumber)) {
		ROS_ERROR("writePin: Bad HD, not driving pin.\n");
		return 0;
	}
	if (!(commDevice->ready())) {
		ROS_ERROR("writePin: Device for interface not ready.\n");
		return 0;
	}
	if (dataType == PACKET_INVALID) {
		ROS_ERROR("writePin: Recieved invalid dataType.\n");
		return 0;
	}

	/* Code to assign pin goes here. The specific code depends on the device to
	 * connect to. Data must be formatted and then sent off to the parent device.
	 * This codes is an example for PWM.
	 */
	uint16_t dataForDevice;
	DataType dataTypeForDevice;
	if (dataType == PACKET_PWM_FREQ) {
		dataTypeForDevice = PACKET_PWM_FREQ;
		// check/set frequency
		if (data[0] > MAX_PWM_FREQUENCY_VALUE) // Max value for frequency
			dataForDevice = MAX_PWM_FREQUENCY_VALUE;
		else if (data[0] < 1) // Min value for frequency
			dataForDevice = 1;
		else dataForDevice = data[0];
	} else if (dataType == PACKET_PWM_DUTY_100) {
		dataTypeForDevice = PACKET_PWM_ON_TICKS;
		if (data[0] > 100) // Max
			dataForDevice = MAX_PWM_TICKS;
		else
			dataForDevice = (data[0] * MAX_PWM_TICKS) / 100;
		// set PWM on/off ticks based on duty cycle
	} else if (dataType == PACKET_PWM_ON_TICKS) {
		dataTypeForDevice = PACKET_PWM_ON_TICKS;
		if (data[0] > MAX_PWM_TICKS) // Max
			dataForDevice = MAX_PWM_TICKS;
		else
			dataForDevice = data[0];
		// check/set PWM on/off ticks based on on ticks
	} else {
		ROS_ERROR("writePin: Recieved invalid dataType: %d\n", dataType);
		return 0;
	} // Parent device expects data format PACKET_PWM_ON_TICKS
	// printf("Writing: %d", dataForDevice);
	return commDevice->setPinValue(pinBus.getPin(pinNumber), &dataForDevice,
	                               dataTypeForDevice, interfaceTypeId);
} /* writePin */
}

;
