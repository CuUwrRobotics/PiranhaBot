// Handler for hardware devices.

#include "HwHeader.h"
#include "Devices_interfaces.h"
#include "PinBus.h"

/**
 * Device
 * @author
 */
class Device {
private:

/**
 * @param addr TODO
 * @return TODO
 */

virtual bool deviceInit() = 0;

/**
 * Used to give base class access to local variables.
 */

virtual uint8_t getPinCount()  = 0;

/**
 * Used to give base class access to local variables.
 * @return TODO
 */

virtual uint8_t getCommType()  = 0;

/**
 * Used to give base class access to local variables.
 * @return The valid pin modes array declared at top of file.
 */

virtual PinMode getValidPinModes(uint8_t i) = 0;

/**
 * Used to give base class access to local variables.
 * @return Size of the valid pin modes array, which cannot be extrapolated from
 * the pointer returned by getValidPinModes()
 */

virtual uint8_t getValidPinModesCount() = 0;

/**
 * Used to give base class access to local variables.
 * @return reservedPins array of size getPinCount()
 */

virtual uint8_t &getReservedPins(uint8_t i) = 0;

/**
 * Used to give base class access to local variables.
 * @return String of the function that the chip used provides
 */

virtual char *getHardwareFunction() = 0;

/**
 * Used to give base class access to local variables.
 * @return String of the name of the chip used
 */

virtual char *getHardwareName() = 0;

public:

PinBus currentPinBus; // Current state of all pins
PinBus requestedPinBus; // State of pins not yet set
bool writeDataPending = false;
bool pinModeChangePending = false;
uint8_t address; // Address for communications
bool deviceIsSetup = false;

uint8_t deviceIndex; // Index of the device object

/**
 * Used to give base class access to local variables.
 * @return TODO
 */

virtual uint8_t getDeviceTypeId() = 0;

/**
 * Used to get the value of a pin. If pin is readable, value will be the last read data.
 * @return Pin value of the given pin
 */

virtual float getPinValue(uint8_t pin, DataType dataType) = 0;

/**
 * Used to set the value of a pin
 * If pin is readable, this will fail.
 * @return If settign was successful.
 */

virtual bool setPinValue(uint8_t pin, float *data, DataType dataType,
                         uint8_t interfaceId) = 0;

/**
 * @return If given mode is valied for this device.
 */
bool pinModeIsValid(PinMode mode) {
	for (int i = 0; i < getValidPinModesCount(); i++) {
		if (mode == getValidPinModes(i))
			return true;
	}
	return false;
} // pinModeIsValid

/**
 * Init the variables and device for this object.
 * WARINGING: Do NOT overwrite this in subclasses! Instead, use the provided
 * deviceInit, which will be called by this method before finishing.
 * @return Whether init was successful
 */

// virtual bool init(uint8_t index, uint8_t addr, BusType busType) = 0; /* getTypeId */
bool init(uint8_t index, uint8_t addr, BusType busType){
	printf("Starting %s device %s at address 0x%.2X.\n",
	       getHardwareFunction(), getHardwareName(), addr);
	deviceIsSetup = false;
	deviceIndex = index;
	address = addr;
	requestedPinBus.setBusType(busType);
	currentPinBus.setBusType(busType);
	// Set each pin to defaults and available for control
	for (uint8_t i = 0; i < getPinCount(); i++) {
		getReservedPins(i) = HardwareDescriptor::INTF_INVALID;
		requestedPinBus.setPinMode(i, MODE_INPUT);
	}

	if (deviceInit()) {
		deviceIsSetup = true;
	}
	return deviceIsSetup;
} /* init */

/**
 * Hands control to pins over to an interface. Once control is granted, nothing
 * else can control these pins.
 * @param pinNumbers An array of pin numbers to assigne to the interface.
 * @return If handover was successful
 */

bool attachInterface(PinBus pinBus, uint8_t interfaceId) {
	// Check pins and ensure they are not yet set to an interface.
	if (!verifyPins(pinBus, HardwareDescriptor::INTF_INVALID)) {
		printf("%s DEVICE -> ERROR: Bad pin configs; not changing modes\n",
		       getHardwareName());
		return false;
	}
	if (pinBus.getBusType() != currentPinBus.getBusType()) {
		printf("%s DEVICE attachInterface request got bad bus type!\n",
		       getHardwareName());
		return false;
	}

	// Pins are all within range and available, so assign control
	// For each pin:
	// -> assign pins
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) { // for each pin
		getReservedPins(pinBus.getPin(i)) = interfaceId;
		if (currentPinBus.getPinMode(pinBus.getPin(i)) != pinBus.getPinMode(i)) {
			requestedPinBus.setPinMode(pinBus.getPin(i), pinBus.getPinMode(i));
			pinModeChangePending = true;
		}
	}
	return true;
} // attachInterface

/**
 * @return This device's index.
 */

uint8_t getDeviceIndex(){
	return deviceIndex;
} // getDeviceIndex

/* Updates any data from the device. If this device is readable, this will always check any pins
 * assigned as inputs. If it is not, if there is new data to be written, that data will be sent to
 * the device.
 * @return True if reads and writes were successful or unneccesary.
 */
virtual bool updateData() = 0;

/**
 * @param pinNumbers TODO
 * @param pinModes TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinModes(PinBus pinBus, uint8_t interfaceId) {
	if (!verifyPins(pinBus, interfaceId)) {
		printf("%s DEVICE -> ERROR: Bad pin configs; not chainging pin modes\n",
		       getHardwareName());
		return false;
	}
	// Pins are all within range and available, so assign control
	// For each pin:
	// -> assign pins
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) { // for each pin
		if (currentPinBus.getPinMode(pinBus.getPin(i)) != pinBus.getPinMode(i)) {
			requestedPinBus.setPinMode(pinBus.getPin(i), pinBus.getPinMode(i));
			pinModeChangePending = true;
		}
	}
	return true;
} // setPinModes

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinMode(uint8_t pinNumber, PinMode pinMode, uint8_t interfaceId) {
	if (getReservedPins(pinNumber) != interfaceId) {
		printf("%s DEVICE -> ERROR: pin in use\n", getHardwareName());
		return false;
	}
	if (!pinModeIsValid(pinMode)) {
		printf("%s DEVICE -> ERROR: bad pin mode\n", getHardwareName());
		return false;
	}
	// Pins are all within range and available, so assign control
	// For each pin:
	// -> assign pins
	// for (uint8_t i = 0; i < pinNumbersLength; i++) { // for each pin
	if (currentPinBus.getPinMode(pinNumber) != pinMode) {
		requestedPinBus.setPinMode(pinNumber, pinMode);
		pinModeChangePending = true;
	}
	// }
	return true;
} // setPinModes

/**
 * @return TODO
 */

bool verifyPins(PinBus pinBus, uint8_t interfaceId) {
	if (pinBus.getPinCount() < 0 || pinBus.getPinCount() > getPinCount()) {
		printf("%s DEVICE: ERROR: pin counts too high or low\n", getHardwareName());
		return false;
	}
	// For each pin:
	// -> Check that all pins are within range
	// -> Check that pins are not in use
	// -> verify pin modes
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) { // for each pin
		if (pinBus.getPin(i) < 0 || pinBus.getPin(i) >= getPinCount()) { // If not exist
			printf("%s DEVICE -> ERROR: bad pin number at i = %d: %d\n",
			       getHardwareName(), i, pinBus.getPin(i));
			return false;
		}
		if (getReservedPins(pinBus.getPin(i)) != interfaceId) {
			printf("%s DEVICE -> ERROR: pins in use\n", getHardwareName());
			return false;
		}
		if (!pinModeIsValid(pinBus.getPinMode(i))) {
			printf("%s DEVICE -> ERROR: bad pin mode at i=%d\n",
			       getHardwareName(), i);
			return false;
		}
	}
	return true;
} // verifyPins

/**
 * @return TODO
 */

bool readableDataAvailable() {
	for (uint8_t pin = 0; pin < getPinCount(); pin++) // For each pin
		if (currentPinBus.getPinMode(pin) == MODE_INPUT) // If the pin uses this mode
			return true; // Then there is some readable data
	return false; // There was no readable data
} // readableDataAvailable

/**
 * @return TODO
 */

inline bool pinIsReadable(uint8_t pin) {
	if (currentPinBus.getPinMode(pin) == MODE_INPUT) // If the pin is set to input
		return true; // Then there is some readable data
	return false; // There was no readable data
} // readableDataAvailable

/**
 * @return TODO
 */

inline PinMode getPinMode(uint8_t pin){
	return currentPinBus.getPinMode(pin);
} // getPinModes

/**
 * @param pin TODO
 * @return TODO
 */

uint8_t getPinInterface(uint8_t pin) {
	return getReservedPins(pin);
} // getPinInterfaceOnPin

/*
 * @return
 */
PinBus getPinBus() {
	return currentPinBus;
} // getPinBus

/**
 * @return If the device has been initialized yet.
 */

inline bool ready(){
	return deviceIsSetup;
} /* ready */
}

;
