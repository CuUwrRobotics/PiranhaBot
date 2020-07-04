/*
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "PinData.h"

/**
 * Interface_EmergIO
 * @author Nicholas Steele
 */
class Interface_EmergIO : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[13] = "EMERGENCY_IO"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 3; // number of pins
const static uint8_t interfaceTypeId =
	HardwareDescriptor::INTF_TEL_EMERGENCY_IO; // The ID for this
                                             // intf
const static uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The ID for the device
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

void setDefaultModes(){
	pinBus.setAllPins(MODE_INPUT);
	commDevice->setPinModes(pinBus, interfaceTypeId);
}  // setDefaultModes

/**
 * Reads data from the comm device and formats into requested data format, if possible.
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

uint16_t readPin(uint8_t pin, DataType dataType) {
	if (dataType == PACKET_GPIO_STATE) {
		return commDevice->getPinValue(pinBus.getPin(pin),
		                               PACKET_GPIO_STATE);
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
		ROS_ERROR("writePin: Recieved invalid format.\n");
		return 0;
	}
	uint16_t dataForDevice;
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
}  /* writePin */
}

;
