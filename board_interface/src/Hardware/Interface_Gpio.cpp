/*
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 

/**
 * Interface_Gpio
 * @author Nicholas Steele
 */
class Interface_Gpio : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[9] = "GPIO"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 16; // number of pins
uint8_t interfaceTypeId = HardwareDescriptor::INTF_GPIO; // The ID for this intf
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The IF for the device
// ----------------------------------------------------------------------------

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
} // prepareInterface

/**
 * Reads data from the comm device and formats into requested data format, if possible.
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

float *readPin(uint8_t pin, DataType dataType) {
	if (dataType == PACKET_GPIO_STATE) {
		static float data[1];
		data[0] = commDevice->getPinValue(pinBus.getPin(pin),
		                                  PACKET_GPIO_STATE);
		return data;
	} else {
		ROS_ERROR("readPin: Recieved invalid dataType: %d\n", dataType);
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
uint8_t writePin(uint8_t pinNumber, float *data, DataType dataType,
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
		ROS_ERROR("writePin: Recieved invalid format.\n");
		return 0;
	}
	float dataForDevice;
	DataType dataTypeForDevice;
	if (dataType == PACKET_GPIO_STATE) {
		dataTypeForDevice = PACKET_GPIO_STATE;
		if (data[0] == 1)
			dataForDevice = 1;
		else if (data[0] == 0)
			dataForDevice = 0;
		else {
			ROS_ERROR("writePin: Recieved invalid state: %d\n", data[0]);
			return 0;
		}
	} else {
		ROS_ERROR("writePin: Recieved invalid dataType: %d\n", dataType);
		return 0;
	} // Parent device expects data format PACKET_PWM_ON_TICKS
	return commDevice->setPinValue(pinBus.getPin(pinNumber), &dataForDevice,
	                               dataTypeForDevice, interfaceTypeId);
} /* writePin */
}

;
