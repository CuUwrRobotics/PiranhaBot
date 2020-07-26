/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

 #include "HwHeader.h"
 #include "AllDevicesInterfaces.h"
 #include "PinBus.h"

/**
 * Interface
 * @author Nicholas Steele
 */
class Interface {
private:
public:

Device *commDevice; // The device to actually communicate using
PinBus pinBus;

bool initerrorVal = false;
bool commDeviceExists = false;
uint8_t interfaceIndex = 0xFF; // Index of the interface object

// Must be overwritten by subclasses
// ****************************************************************************

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writePin(PinValue_t *valueIn) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readPin(PinValue_t *valueIn) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writeConfig(InterfaceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readConfig(InterfaceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writeDeviceConfig(DeviceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readDeviceConfig(DeviceConfig_t *cfg) = 0;

virtual void prepareInterface() = 0;

virtual uint8_t getPinCount() = 0;

virtual Interface_t getInterfaceTypeId() = 0;

virtual uint8_t getParentTypeId() = 0;

// Regular fuctions
// ****************************************************************************

/** Initialize the interface.
 * @param device Pointer for Device object that the interface communicates with. MUST already be
 * initialized
 * @param devicePins An array of the pin numbers to be used on the given device.
 * @param hd The full hardware descriptor.
 * @return true of sucessful
 */
bool start(Device *device, PinBus pb, uint8_t ifaceIndex) {
	initerrorVal = false;
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
	pinBus = pb;

	if (commDevice->attachInterface(pinBus, getInterfaceTypeId())) {
		prepareInterface(); // Set interface-specific default modes to pinBus
		initerrorVal = true;
	} else ROS_ERROR("Interface #%d (%s) could not attatch to the parent device.",
		               interfaceIndex, getInterfaceName());
	return initerrorVal;

	// return true
} /* start */

/*
 * Some interfaces cannot have pin modes changed. They need to override this.
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
virtual uint8_t setPinMode(uint8_t pinNumber, PinMode_t pinMode){
	if (!(commDevice->ready())) {
		printf("Device for interface not ready!\n");
		return 0;
	}
	// Try to drive a pin in commDevice
	return commDevice->setPinMode(pinBus.getPin(pinNumber), pinMode);
}; /* setPinMode */

/*
 * @return The parent device's index.
 */

inline uint8_t getParentDeviceIndex(){
	if (!commDeviceExists)
		return 0;
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

inline PinMode_t getPinMode(uint8_t pin){
	return commDevice->getPinMode(pinBus.getPin(pin));
} // getPinMode

/**
 * @return TODO
 */

inline bool ready(){
	return initerrorVal;
} // ready

/**
 * @return TODO
 */

inline uint8_t getMappedDevPin(uint8_t pin){
	return pinBus.getPin(pin);
} // getMappedPin

/**
 * @return TODO
 */

PinBus getPinBus() {
	return pinBus;
} // getPinBus

/**
 * @return The subclass's interface name.
 */

const char *getInterfaceName() {
	return interfaceIdToCharArray(getInterfaceTypeId());
} // getInterfaceName
}

;
