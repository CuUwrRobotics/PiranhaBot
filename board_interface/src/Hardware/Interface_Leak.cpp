/*
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "PinData.h"

/**
 * Interface_Leak
 * @author Nicholas Steele
 */
class Interface_Leak : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[5] = "LEAK"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 8; // number of pins
uint8_t interfaceTypeId = HardwareDescriptor::INTF_TEL_LEAK; // The ID for this intf
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

void setDefaultModes(){
	pinBus.setAllPins(MODE_INPUT);
	commDevice->setPinModes(pinBus, interfaceTypeId);
} // setDefaultModes

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
	ROS_INFO("writePin: Data cannot be written to the LEAK interface!");
	return 0;
} /* writePin */

// **** OVERRIDE OVER PARENT CLASS ****
uint8_t setPinMode(uint8_t pinNumber, PinMode pinMode, uint64_t hd){
	ROS_INFO("setPinMode: Data cannot be written to the LEAK interface!");
	return 0;
} /* setPinMode */
}

;
