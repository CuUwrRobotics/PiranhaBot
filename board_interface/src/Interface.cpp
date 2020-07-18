/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "PinBus.h"

/**
 * Interface
 * @author Nicholas Steele
 */
class Interface {
protected:
private:
public:

// How this interface will communicate with real hardware
Device *commDevice; // The device to actually communicate using
PinBus pinBus;

// Hardware descirption things
// uint8_t getInterfaceTypeId() = HardwareDescriptor::INTF_INVALID; // The type ID
bool initSuccessful = false;
bool commDeviceExists = false;
// uint8_t getParentTypeId() = HardwareDescriptor::DEVICE_GPIO; // The type ID
uint8_t interfaceIndex = 0xFF; // Index of the interface object
uint64_t baseHardwareDescriptor; // Stores all of the data for the hardware descriptor EXCEPT the

// Must be overwritten by subclasses
// ****************************************************************************
virtual uint8_t writePin(uint8_t pinNumber, float *data, DataType dataType,
                         uint64_t hd) = 0;

virtual float *readPin(uint8_t pin, DataType dataType) = 0;

virtual void prepareInterface() = 0;

virtual uint8_t getPinCount() = 0;

virtual char *getInterfaceName() = 0;

virtual uint8_t getInterfaceTypeId() = 0;

virtual uint8_t getParentTypeId() = 0;

// Regular fuctions
// ****************************************************************************

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
	if (getParentTypeId() != commDevice->getDeviceTypeId()) {
		printf("ERROR: getParentTypeId() bad. Expeccted: 0x%.16X, Got: 0x%.16X\n",
		       getParentTypeId(), commDevice->getDeviceTypeId());
		return false;
	}
	// Check that device is initialized
	if (!commDevice->ready()) {
		printf("ERROR: parentDevice not ready\n");
		return false;
	}
	// Check that devicePins is not longer than getPinCount()
	if (pb.getPinCount() != getPinCount()) {
		printf("ERROR: device pins size bad: %d, pin count = %d\n",
		       pb.getPinCount(),
		       getPinCount());
		return false;
	}
	// Some variable setups
	interfaceIndex = ifaceIndex;
	// Create a base hardware descriptor that uses in invalid pin number
	baseHardwareDescriptor =
		HardwareDescriptor::createHardwareDescriptor(getInterfaceTypeId(),
		                                             getParentTypeId(),
		                                             interfaceIndex,
		                                             commDevice->getDeviceIndex(),
		                                             HardwareDescriptor::
		                                             INVALID_PIN_NUMBER);
	pinBus = pb;
	if (commDevice->attachInterface(pinBus, getInterfaceTypeId())) {
		prepareInterface(); // Set interface-specific default modes to pinBus
		initSuccessful = true;
	}
	return initSuccessful;

	// return true
} /* start */

/*
 * Some interfaces cannot have pin modes changed. They need to override this.
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
virtual uint8_t setPinMode(uint8_t pinNumber, PinMode pinMode, uint64_t hd){
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
}; /* setPinMode */

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
 * @return The parent device's index.
 */

inline uint8_t getParentDeviceIndex(){
	if (!commDeviceExists)
		return HardwareDescriptor::DEVICE_INVALID;
	return commDevice->getDeviceIndex();
} // getParentDeviceIndex

/*
 * @return This devices index.
 */

inline uint8_t getInterfaceIndex(){
	return interfaceIndex;
} // getParentDeviceIndex

/**
 * @return TODO
 */

inline PinMode getPinMode(uint8_t pin){
	return commDevice->getPinMode(pinBus.getPin(pin));
} // getPinMode

/**
 * @return TODO
 */

inline bool ready(){
	return initSuccessful;
} // ready

/**
 * @return TODO
 */

inline uint8_t getDevPin(uint8_t pin){
	return pinBus.getPin(pin);
} // getMappedPin

/**
 * @return TODO
 */

PinBus getPinBus() {
	return pinBus;
} // getPinBus
}

;
