// Handler for hardware devices.

#include "Device.h"

// PinBus currentPinBus; // Current state of all pins
// PinBus requestedPinBus; // State of pins not yet set
// bool writeDataPending = false;
// bool pinModeChangePending = false;
// uint8_t address; // Address for communications
// bool deviceIsSetup = false;
//
// uint8_t deviceIndex; // Index of the device object

/**
 * @return If given mode is valied for this device.
 */
bool Device::pinModeIsValid(PinMode_t mode) {
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
 * @return Whether init was errorVal
 */

// virtual bool init(uint8_t index, uint8_t addr, BusType_t busType) = 0; /* getTypeId */
bool Device::init(uint8_t index, uint8_t addr, BusType_t busType){
	printf("Starting %s device %s at address 0x%.2X.\n",
	       deviceHardwareFunction(getDeviceTypeId()), getHardwareName(), addr);
	deviceIsSetup = false;
	deviceIndex = index;
	address = addr;
	requestedPinBus.setBusType(busType);
	currentPinBus.setBusType(busType);
	// Set each pin to defaults and available for control
	for (uint8_t i = 0; i < getPinCount(); i++) {
		getReservedPins(i) = INTF_INVALID; // Un-reserve all pins
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
 * @return If handover was errorVal
 */

bool Device::attachInterface(PinBus pinBus, Interface_t interfaceId) {
	// Check pins and ensure they are not yet set to an interface.
	if (!verifyPins(pinBus)) {
		log_error("Device #%d (%s): Bad pin configs; not changing modes",
		          deviceIndex, getHardwareName());
		return false;
	}
	if (pinBus.getBusType() != currentPinBus.getBusType()) {
		log_error("Device #%d (%s):  Request got bad bus type.",
		          deviceIndex, getHardwareName());
		return false;
	}

	// Pins are all within range, so allow control if available
	// For each pin
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) {
		if (getReservedPins(pinBus.getPin(i)) != INTF_INVALID) { // Check if pin is reserved
			log_error(
				"Device #%d (%s): Got a request by %s for a pin %d reserved by %s.",
				deviceIndex, getHardwareName(),
				interfaceIdToCharArray(interfaceId), pinBus.getPin(i),
				interfaceIdToCharArray(getReservedPins(pinBus.getPin(i))));
			return false;
		}
		getReservedPins(pinBus.getPin(i)) = interfaceId; // Set pin to reserved
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

uint8_t Device::getDeviceIndex(){
	return deviceIndex;
} // getDeviceIndex

/**
 * @param pinNumbers TODO
 * @param pinModes TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool Device::setPinModes(PinBus pinBus) {
	if (!verifyPins(pinBus)) {
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

bool Device::setPinMode(uint8_t pinNumber, PinMode_t pinMode) {
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

bool Device::verifyPins(PinBus pinBus) {
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

bool Device::readableDataAvailable() {
	for (uint8_t pin = 0; pin < getPinCount(); pin++) // For each pin
		if (currentPinBus.getPinMode(pin) == MODE_INPUT) // If the pin uses this mode
			return true; // Then there is some readable data
	return false; // There was no readable data
} // readableDataAvailable

/**
 * @param pin TODO
 * @return TODO
 */

Interface_t Device::getPinInterface(uint8_t pin) {
	return getReservedPins(pin);
} // getPinInterfaceOnPin

/*
 * @return
 */
PinBus Device::getPinBus() {
	return currentPinBus;
} // getPinBus
