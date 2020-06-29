/* Interface_Leak.cpp
 * Checks leak sensors on GPIO3.
 * Also interacts with telemetry interface
 */

 #include "../HwHeader.h"
 #include "Devices_interface.h"

/**
 * PowerControl
 * @author
 */
class Interface_EmergIO : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ----------------------------------------------------------------------------
const static uint8_t PIN_COUNT = 3; // number of pins
uint8_t interfaceTypeId = HardwareDescriptor::INTF_TEL_EMERGENCY_IO; // The ID for this intf
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The IF for the device
// ----------------------------------------------------------------------------

// uint8_t pinMapToDevice[PIN_COUNT]; // Contains a map to the devices pins.
// uint8_t pinModes[PIN_COUNT]; // Contains a map to the devices pins.
// interface based on HD
// class values
public:

/* Initialize the interface.
 * @param device Pointer for Device object that the interface communicates with. MUST already be
 * initialized
 * @param devicePins An array of the pin numbers to be used on the given device.
 * @param hd The full hardware descriptor.
 * @return true of sucessful
 */
 bool start(Device *device, PinBus pb, uint8_t ifaceIndex) {
 	initSuccessful = false;
 	// Verification code: don't put any init stuff here!
 	// Check that device is the correct type
 	commDevice = device;
	commDeviceExists = true;
 	if (parentDeviceTypeId != commDevice->getTypeId()) {
 		printf("ERROR: parentDeviceTypeId bad. Expeccted: 0x%.16X, Got: 0x%.16X\n",
 		       parentDeviceTypeId, commDevice->getTypeId());
 		return false;
 	}
 	// Check that device is initialized
 	if (!commDevice->ready()) {
 		printf("ERROR: parentDevice not ready\n");
 		return false;
 	}
 	// Check that devicePins is not longer than PIN_COUNT
 	if (pb.getPinCount() != PIN_COUNT) {
 		printf("ERROR: device pins size bad: %d, pin count = %d\n",
 		       pb.getPinCount(),
 		       PIN_COUNT);
 		return false;
 	}
 	// Verification done. Add any code for initialization here.
 	// ***************************************************************************
 	// Some variable setups
 	interfaceIndex = ifaceIndex;
 	// Create a base hardware descriptor that uses in invalid pin number
 	baseHardwareDescriptor =
 		HardwareDescriptor::createHardwareDescriptor(interfaceTypeId,
 		                                             parentDeviceTypeId,
 		                                             interfaceIndex,
 		                                             commDevice->getDeviceIndex(),
 		                                             HardwareDescriptor::
 		                                             INVALID_PIN_NUMBER);
 	// for (uint8_t i = 0; i < PIN_COUNT; i++) {
 	// pinMapToDevice[i] = devicePins[i];
 	// pinModes[i] = 0x10;
 	// }
 	pinBus = pb;
 	if (commDevice->attachInterface(pinBus, interfaceTypeId))
 		initSuccessful = true;
 	return initSuccessful;

 	// return true
 } /* start */

/**
 * @param pinNumber TODO
 * @return TODO
 */

uint64_t getHardwareDescriptor(uint8_t pinNumber){
	if (!(commDevice->ready())) {
		printf("Device for interface not ready!\n");
		return 0;
	}
	return HardwareDescriptor::addPinNumber(baseHardwareDescriptor,
	                                        pinNumber);
} // getHardwareDescriptor

/*
 * TODO: enumerate errors better
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
int drivePin(PinState pinState, uint8_t pinNumber, uint64_t hd){
	// Check that HD matches
	if (hd != getHardwareDescriptor(pinNumber)) {
		printf("Interface -> bad HD, not driving pin.\n");
		return 0;
	}
	if (!(commDevice->ready())) {
		printf("Device for interface not ready!\n");
		return 0;
	}
	// Try to drive a pin in commDevice
	return commDevice->setPinState(pinBus.getPin(pinNumber), pinState,
	                               HardwareDescriptor::getInterfaceId(hd));
} /* verifyHardwareDescriptor */

/*
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
int pinMode(PinMode pinMode, uint8_t pinNumber, uint64_t hd){
	// Check that HD matches
	if (hd != HardwareDescriptor::addPinNumber(baseHardwareDescriptor,
	                                           pinNumber)) {
		printf("Power interface -> bad HD, not changing pin modes.\n");
		return 0;
	}
	if (!(commDevice->ready())) {
		printf("Device for interface not ready!\n");
		return 0;
	}
	// Try to drive a pin in commDevice
	return commDevice->setPinMode(pinBus.getPin(pinNumber), pinMode,
	                              HardwareDescriptor::getInterfaceId(hd));
} /* verifyHardwareDescriptor */

/**
 * @return TODO
 */

inline PinMode getPinMode(uint8_t pin){
	return commDevice->getPinMode(pinBus.getPin(pin));
} // getPinMode

/**
 * @return TODO
 */

inline PinState getPinState(uint8_t pin){
	return commDevice->getPinState(pinBus.getPin(pin));
} // getPinState

/**
 * @return TODO
 */

inline uint8_t getTypeId(){
	return interfaceTypeId;
} // getTypeId

/**
 * @return TODO
 */

// inline uint8_t getParentDeviceIndex(){
// 	if (!initSuccessful)
// 		return HardwareDescriptor::DEVICE_INVALID;
// 	return commDevice->getDeviceIndex();
// } // getParentDeviceIndex

/*
 * @return The parent device's index.
 */
void printPins(){
	for (uint8_t i = 0; i < PIN_COUNT - 1; i++) {
		printf("%d, ", pinBus.getPin(i));
	}
	printf("%d", pinBus.getPin(PIN_COUNT - 1));
}; // pinsToCharArray

/*
 * @return The parent device's index.
 */
inline bool ready(){
	return initSuccessful;
} // ready

/**
 * @return TODO
 */

inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

/**
 * @return TODO
 */

inline uint8_t getDevPin(uint8_t pin){
	return pinBus.getPin(pin);
} // getMappedPin
}

;
