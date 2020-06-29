/* Main class that represents MCP23017 devices as Device objects. This object should NOT be directly
 * written to, but should be controlled by the interfaces, escept for when int main() tell this
 * object to update.
 */
// Generic headers
#include "../HwHeader.h"
#include "Devices_interface.h"

// Header file custom to this specific chip
#include "Mcp23017Gpio_Device.h"

/**
 * Device
 * @author
 */
class Mcp23017Gpio_Device : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW INTERFACE SUBCLASS!!
// ***************************************************************************
// ---------------------------------------------------------------------------
const static uint8_t PIN_COUNT = 16; // Defined in subclasses; must be used to size all pin
                                     // arrays.
const static uint8_t COMM_TYPE = COMM_TYPE_I2C; // Defines type of communication
const static uint8_t deviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The type ID for this device

// based on HD
//
inline bool pinModeIsValid(PinMode mode) {
	return (mode == GPIO_INPUT_MODE || mode == GPIO_INPUT_X_MODE ||
	        mode == GPIO_OUTPUT_MODE);
} // pinModeIsGpioMode

//
inline bool pinStateIsValid(PinState state) {
	return (state == ON_STATE || state == OFF_STATE ||
	        state == NO_STATE);
} // pinModeIsGpioMode
// ---------------------------------------------------------------------------
// ***************************************************************************

// uint8_t currentPinStates[PIN_COUNT]; // List of pin states from last update
// uint8_t requestedPinStates[PIN_COUNT]; // List of pin states requested
// uint8_t currentPinModes[PIN_COUNT]; // List of pin r/w states requested
// uint8_t requestedPinModes[PIN_COUNT]; // List of pin r/w states requested
uint8_t reservedPins[PIN_COUNT]; // Stores what interfaces reserved what pins.

/**
 * Init the device. Overwrites Device Init from Device class. Called when init() is called
 * @return Whether init was successful
 */

bool deviceInit(uint8_t addr){
	printf("GPIO DEVICE -> Init for MCP23017 device at address 0x%.2X (TODO).\n",
	       addr);

	pinModeChangePending = true;
	writeDataPending = true;

	// Check if any pins need their modes changed
	if (pinModeChangePending) {
		printf("GPIO DEVICE -> Init is changing pin modes (TODO).\n");
		pinModeChangePending = false;
	}
	// Check if any data is readable. If so, read it.
	if (readDataPending) {
		printf("GPIO DEVICE -> Init is reading data from pins (TODO).\n");
		readDataPending = false;
	}
	// Check if any data needs to be written. If so, write it.
	if (writeDataPending) {
		printf("GPIO DEVICE -> Init is writng data to pins (TODO).\n");
		writeDataPending = false;
	}
	return true;
} /* getTypeId */

public:

// Descriptions for this device
// class values

/**
 * Drives a pin as requested from an interface. This will not actually change
 * the pin state yet, but will add it to the 'que'.
 * WARNING: SHOULD ONLY BE CALLED BY INTERFACES!
 *
 * @param pinNumber Number of the pin to drive
 * @param hd Full hardware descriptor
 * @return TODO
 */

int setPinState(uint8_t pinNumber, PinState pinState, uint8_t interfaceId) {
	// Check that pin number is acceptable
	if (pinNumber < 0 || pinNumber >= PIN_COUNT) {
		printf(
			"GPIO DEVICE -> ERROR: Pin state change request on out of range pin\n");
		return 0;
	}
	// Check pin state
	if (pinStateIsValid(pinState)) {
		printf(
			"GPIO DEVICE -> ERROR: Pin state change request with invalid state\n");
		return 0;
	}
	// Check that pin number & interfaceId match up
	if (reservedPins[pinNumber] != interfaceId) {
		printf(
			"GPIO DEVICE -> ERROR: Pin state change request by non-controlling interface\n");
		return 0;
	}
	// Check requested pin modes, not current, insce pin modes are always updated before read/write
	// performed
	if (requestedPinBus.getPinMode(pinNumber) != GPIO_OUTPUT_MODE) {
		printf("GPIO DEVICE -> ERROR: Pin state change request on non-write pin\n");
		return 0;
	}
	// Write the assignment to the que and update other data
	if (requestedPinBus.getPinState(pinNumber) != pinState) {
		requestedPinBus.setPinState(pinNumber, pinState);
		writeDataPending = true; // Mark for data to be written
	}
	return 1;
} // addInterface

/**
 * @param index TODO
 * @param addr TODO
 * @return TODO
 */

bool init(uint8_t index, uint8_t addr, BusType busType){
	deviceIsSetup = false;
	deviceIndex = index;
	address = addr;
	requestedPinBus.setBusType(busType);
	currentPinBus.setBusType(busType);
	// Set each pin to defaults and available for control
	for (uint8_t i = 0; i < PIN_COUNT; i++) {
		reservedPins[i] = HardwareDescriptor::INTF_INVALID;
		requestedPinBus.setPinMode(i, GPIO_INPUT_MODE);
		requestedPinBus.setPinState(i, OFF_STATE);
	}

	if (deviceInit(address)) {
		deviceIsSetup = true;
	}
	return deviceIsSetup;
} /* getTypeId */

/**
 * @return TODO
 */

uint8_t getTypeId(){
	return deviceTypeId;
} /* getTypeId */

/**
 * @return TODO
 */

bool ready(){
	return deviceIsSetup;
} /* ready */

/**
 * TODO: Pin states
 * Hands control to pins over to an interface. Once control is granted, nothing
 * else can control these pins.
 * @param pinNumbers An array of pin numbers to assigne to the interface.
 * @return If handover was successful
 */

bool attachInterface(PinBus pinBus, uint8_t interfaceId) {
	// Check pins and ensure they are not yet set to an interface.
	if (!verifyPins(pinBus, HardwareDescriptor::INTF_INVALID)) {
		printf("GPIO DEVICE -> ERROR: Bad pin configs; not changing modes\n");
		return false;
	}
	// Pins are all within range and available, so assign control
	// For each pin:
	// -> assign pins
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) { // for each pin
		reservedPins[pinBus.getPin(i)] = interfaceId;
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

/**
 * @return TODO
 */

bool updateData(){
	// Check if ready
	if (!ready())
		return false;
	// Check if any pins need their modes changed
	if (pinModeChangePending) {
		printf("GPIO DEVICE -> Changing pin modes (TODO).\n");
		// For now, just push data over
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++)
			currentPinBus.setPinMode(pin, requestedPinBus.getPinMode(pin));
		pinModeChangePending = false;
	}
	// Check if any data is readable. If so, read it.
	if (readDataPending) {
		printf("GPIO DEVICE -> Reading data from pins (TODO).\n");
		// For now, just push data over
		readDataPending = false;
	}
	// Check if any data needs to be written. If so, write it.
	if (writeDataPending) {
		printf("GPIO DEVICE -> Writng data to pins (TODO).\n");
		// For now, just push data over
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++)
			currentPinBus.setPinState(pin, requestedPinBus.getPinState(pin));
		writeDataPending = false;
	}
	return true;
} // updateData

/**
 * @param pinNumbers TODO
 * @param pinModes TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinModes(PinBus pinBus, uint8_t interfaceId) {
	if (!verifyPins(pinBus, interfaceId)) {
		printf("GPIO DEVICE -> ERROR: Bad pin configs; not chainging pin modes\n");
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
	if (reservedPins[pinNumber] != interfaceId) {
		printf("GPIO DEVICE -> ERROR: pin in use\n");
		return false;
	}
	if (pinModeIsValid(pinMode)) {
		printf("GPIO DEVICE -> ERROR: bad pin mode\n");
		return false;
	}
	// Pins are all within range and available, so assign control
	// For each pin:
	// -> assign pins
	// for (uint8_t i = 0; i < pinNumbersLength; i++) { // for each pin
	if (currentPinBus.getPinMode(pinNumber) != pinMode) {
		requestedPinBus.setPinMode(pinNumber, pinMode);
		pinModeChangePending = true;
		// printf("GPIO DEVICE #%d -> Changing pin mode for pin %d: 0x%X\n",
		//        deviceIndex, pinNumber, pinMode);
	}
	// }
	return true;
} // setPinModes

/**
 * @return TODO
 */

bool verifyPins(PinBus pinBus, uint8_t interfaceId) {
	if (pinBus.getPinCount() < 0 || pinBus.getPinCount() > PIN_COUNT) {
		printf("GPIO DEVICE -> ERROR: pin counts too high or low\n");
		return false;
	}
	// For each pin:
	// -> Check that all pins are within range
	// -> Check that pins are not in use
	// -> verify pin modes
	for (uint8_t i = 0; i < pinBus.getPinCount(); i++) { // for each pin
		if (pinBus.getPin(i) < 0 || pinBus.getPin(i) >= PIN_COUNT) { // If not exist
			printf("GPIO DEVICE -> ERROR: bad pin number at i = %d: %d\n", i,
			       pinBus.getPin(i));
			return false;
		}
		if (reservedPins[pinBus.getPin(i)] != interfaceId) {
			printf("GPIO DEVICE -> ERROR: pins in use\n");
			return false;
		}
		if (!pinModeIsValid(pinBus.getPinMode(i))) {
			printf("GPIO DEVICE -> ERROR: bad pin mode at i=%d\n", i);
			return false;
		}
	}
	return true;
} // verifyPins

/**
 * @return TODO
 */

inline PinMode getPinMode(uint8_t pin){
	return currentPinBus.getPinMode(pin);
} // getPinModes

/**
 * @return TODO
 */

inline PinState getPinState(uint8_t pin){
	return currentPinBus.getPinState(pin);
} // getPinModes

/**
 * @param pin TODO
 * @return TODO
 */

uint8_t getPinInterfaceOnPin(uint8_t pin) {
	return 0;
}; // getPinInterfaceOnPin
}

;
