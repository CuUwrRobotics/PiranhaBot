// Handler for hardware devices.

#include "../HwHeader.h"
#include "Devices_interface.h"

/**
 * Device
 * @author
 */
class GpioDevice : public Device {
private:
const static uint8_t PIN_COUNT = 16; // Defined in subclasses; must be used to size all pin
                                     // arrays.
const static uint8_t COMM_TYPE = COMM_TYPE_I2C; // Defines type of communication

// Hardware descirptor things
uint8_t deviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The type ID for this device based on HD

/**
 * Init the device. Overwrites Device Init from Device class. Called when init() is called
 * @return Whether init was successful
 */

bool deviceInit(uint8_t addr){
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

int drivePin(uint8_t pinState, uint8_t pinNumber, uint8_t interfaceId) {
	// Check that pin number is acceptable
	if (pinNumber < 0 || pinNumber >= PIN_COUNT)
		return 0;
	// Check pin state
	if (pinState != GPIO_PIN_ON || pinState != GPIO_PIN_OFF)
		return 0;
	// Check that pin number & interfaceId match up
	if (reservedPins[pinNumber] != interfaceId)
		return 0;
	// Write the assignment to the que
	requestedPinStates[pinNumber] = pinState;
	return 1;
} // addInterface

/**
 * @param index TODO
 * @param addr TODO
 * @return TODO
 */

bool init(uint8_t index, uint8_t addr){
	deviceIsSetup = false;
	deviceIndex = index;
	address = addr;
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
 * Hands control to pins over to an interface. Once control is granted, nothing
 * else can control these pins.
 * @param pinNumbers An array of pin numbers to assigne to the interface.
 * @return If handover was successful
 */

bool attachInterface(uint8_t *pinNumbers, uint8_t interfaceId) {
	// Check that this has an ok ammount of pin numbers.
	// For each pin:
	// -> Check that all pins are within range
	// -> Check that pins are not in use
	// For each pin:
	// -> Only actually assign pins if all conditions above are met
	uint8_t pinNumbersLength = sizeof(pinNumbers) / sizeof(pinNumbers[0]);
	if (pinNumbersLength > PIN_COUNT) { // Make sure number of pins is good
		printf("Error: pin count\n");
		return false;
	}
	for (uint8_t i = 0; i < pinNumbersLength; i++) { // for each pin
		if (pinNumbers[i] < 0 || pinNumbers[i] >= PIN_COUNT) { // If not exist
			printf("Error: bad pin number at i = %d: %d\n", i, pinNumbers[i]);
			return false;
		}
		if (reservedPins[pinNumbers[i]] != HardwareDescriptor::INTF_INVALID) { // If in use
			printf("Error: pins in use\n");
			return false;
		}
	}
	// Pins are all within range and available, so assign control
	for (uint8_t i = 0; i < pinNumbersLength; i++) { // for each pin
		reservedPins[pinNumbers[i]] = interfaceId;
	}
	return true;
} // attachInterface

/**
 * @return This device's index.
 */

uint8_t getDeviceIndex(){
	return deviceIndex;
} // getDeviceIndex
}

;
