/* Handles information for sets or pins, at least 1, at most 16.
 * Once set, some values are unchangable, like bus type and pin count.
 */
// #include "HwHeader.h"
#include "PinBus.h"

/**
 * @param mode TODO
 * @return TODO
 */

bool PinBus::setPinMode(uint8_t index, PinMode_t mode){
	if (mode == MODE_INVALID) {
		log_error("Invalid mode selected\n");
		return false;
	}
	if (!(index < 0 || index > MAX_PINS)) {
		pinModes[index] = mode;
		return true;
	}
	log_error("Bad pin index: %d, can't assign mode.\n",
	          index);
	return false;
} // setPinMode

/**
 * @param modes TODO
 * @return TODO
 */

bool PinBus::setAllPins(PinMode_t mode) {
	if (pinCount == MAX_PINS + 1) {
		log_error("Pin count not yet set, can't assign modes.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		pinModes[i] = mode;
	}
	return true;
} // setPins

/**
 * @param modes TODO
 * @return TODO
 */

bool PinBus::setPinModes(const PinMode_t *modes){
	if (pinCount == MAX_PINS + 1) {
		log_error("Pin count not yet set, can't assign modes.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		if (modes[i] == MODE_INVALID) {
			log_error("Invalid mode selected for pin #%d\n", i);
			continue;
		}
		pinModes[i] = modes[i];
	}
	return true;
} // setPinModes

// Assign a pin. Can only happen once per pin.
bool PinBus::assignPin(uint8_t index, uint8_t pinNumber) {
	if (pinCount == MAX_PINS + 1) {
		log_error("'pinCount' not yet set, cannot assign pin\n",
		          index);
		return false;
	}
	// Check index
	if ((index < 0 || index >= pinCount)) {
		log_error("Could not find pin for assignment: %d\n",
		          index);
		return false;
	}
	pinAssignments[index] = pinNumber;
	return true;
} // assignPin

// Assign a pin. Can only happen once.
bool PinBus::assignPins(uint8_t *pinNumbers, uint8_t pinNumbersLength){
	// If not already set, set pin count.
	if (pinCount == MAX_PINS + 1) {
		pinCount = pinNumbersLength;
	}
	// Verify lengths
	if ((pinNumbersLength < 0 || pinNumbersLength > pinCount)) {
		log_error("Bad pin count for assignment: %d\n",
		          pinNumbersLength);
		return false;
	}
	for (uint8_t pin = 0; pin < pinCount; pin++)
		pinAssignments[pin] = pinNumbers[pin];
	return true;
} // assignPins

/* Assigns all pins as a list from startPin to endPin.
 * If not yet assigned, this will set pinCount.
 * endPin - startPin = pinCount.
 * Ex: endPin = 3, startPin = 8; pin assignments: {3, 4, 5, 6, 7, 8}
 */
bool PinBus::assignPinSet(uint8_t startPin, uint8_t endPin) {
	if (startPin > endPin) {
		log_error("Start value cannot be morethan end\n");
		return false;
	}
	uint8_t newPinCount = (endPin - startPin) + 1; // Get a new pin count
	// Pin count was not yet set
	if (pinCount == MAX_PINS + 1) {
		// Verify the new pin count
		if (newPinCount <= 0 || newPinCount > MAX_PINS) {
			log_error("Bad pin count computed from start/end values\n");
			return false;
		}
		pinCount = newPinCount;
		// return true;
	}	else if (pinCount != newPinCount) {
		log_error("Pin count computed from start / end values does not match preset pin count\n");
		return false;
	}
	// Make assignemnts
	for (uint8_t pin = 0; pin < pinCount; pin++) {
		pinAssignments[pin] = startPin + pin;
	}
	return true;
} // assignPinSet


/**
 * @param type TODO
 * @return TODO
 */

bool PinBus::setBusType(BusType_t type) {
	if (busType != BUS_INVALID) {
		log_error("Bus type already assigned!\n");
		return false;
	}
	busType = type;
	return true;
} // setBusType


/**
 * @param type TODO
 * @return TODO
 */

bool PinBus::setPinCount(uint8_t count) {
	if (pinCount != MAX_PINS + 1) {
		log_error("Pin count already assigned!\n");
		return false;
	}
	if (!(count < 0 || count > MAX_PINS)) {
		pinCount = count;
		return true;
	}
	printf("Bad pin count: %d\n", count);
	return false;
} // setBusType

/**
 * @param busType TODO
 * @param pinCount TODO
 */

bool PinBus::createPinBus(BusType_t busType, uint8_t pinCount){
	if (!(setBusType(busType) && setPinCount(pinCount)))
		return false;
	// this->pinCount = pinCount;
	return true;
} // createPinBus

/**
 * @param bus TODO
 * @param pinCount TODO
 * @param pinTypes TODO
 * @param pinModes TODO
 */

bool PinBus::createPinBus(BusType_t busType, uint8_t *pinAssignments,
                          uint8_t pinCount,
                          const PinMode_t *pinModes){
	if (!createPinBus(busType, pinCount))
		return false;
	// this->busType = busType;
	// this->pinCount = pinCount;
	if (!(assignPins(pinAssignments, pinCount)))
		return false;
	if (!setPins(pinModes))
		return false;
	return true;
} // createPinBus

/**
 * @param busType TODO
 * @param start TODO
 * @param end TODO
 * @param pinCount TODO
 * @param pinModes TODO
 * @return TODO
 */

bool PinBus::createPinBusFromSet(BusType_t busType, uint8_t start,
                                 uint8_t end,
                                 const PinMode_t *pinModes){
	if (!createPinBus(busType, ((end - start) + 1)))
		return false;
	if (!(assignPinSet(start, end)))
		return false;
	if (!setPins(pinModes))
		return false;
	return true;
} // createPinBus

/**
 * @param busType TODO
 * @param start TODO
 * @param end TODO
 * @param pinMode TODO
 * @return TODO
 */

bool PinBus::createUniformPinBusFromSet(BusType_t busType, uint8_t start,
                                        uint8_t end,
                                        PinMode_t pinMode){
	if (!createPinBus(busType, ((end - start) + 1)))
		return false;
	if (!(assignPinSet(start, end)))
		return false;
	if (!setAllPins(pinMode))
		return false;
	return true;
} // createPinBus

/**
 * Full reset of the object and pins
 */

void PinBus::resetAll(){
	for (uint8_t i = 0; i < MAX_PINS; i++) {
		pinModes[i] = MODE_INVALID;
		pinAssignments[i] = 0xFF;
	}
	busType = BUS_INVALID;
	pinCount = MAX_PINS + 1;
} // releaseAll

/**
 * @param pinType TODO
 * @return TODO
 */

bool PinBus::busEquals(BusType_t pinType) {
	return (pinType == busType);
} // busTypeIs

/**
 * @param pin TODO
 * @return TODO
 */

// char RED[10] = "\033[1;31m";
// char NO_COLOR[7] = "\033[0m";
const char *PinBus::getModeString(uint8_t pin, bool colorizeBadOutputs) {
	switch (pinModes[pin]) {
	case MODE_INVALID:
		if (colorizeBadOutputs)
			return "\033[1;31mMODE_INVALID\033[0m";
		else return "MODE_INVALID";
		break;
	case MODE_INPUT:
		return "MODE_INPUT";
		break;
	case MODE_OUTPUT:
		return "MODE_OUTPUT";
	default:
		if (colorizeBadOutputs)
			return "\033[1;31munknown_mode\033[0m";
		else return "unknown_mode";
		break;
	} // switch
} // busTypeIs

/**
 * @return TODO
 */

const char *PinBus::getBusTypeString(bool colorizeBadOutputs){
	switch (busType) {
	case BUS_INVALID:
		if (colorizeBadOutputs)
			return "\033[1;31mBUS_INVALID\033[0m";
		else return "BUS_INVALID";
		break;
	case BUS_GPIO:
		return "BUS_GPIO";
		break;
	case BUS_ADC:
		return "BUS_ADC";
		break;
	case BUS_PWM:
		return "BUS_PWM";
		break;
	case BUS_OTHER:
		return "BUS_OTHER";
		break;
	default:
		if (colorizeBadOutputs)
			return "\033[1;31munknown_bus\033[0m";
		else return "unknown_bus";
		break;
	} // switch
} // PinBus::getBusTypeString

/**
 * Quick dump of pin configuration.
 */

void PinBus::dumpInfo(bool colorful = true) {
	printf("\nPinBus info dump: \n");
	printf("Pin Bus Type: %s\n", getBusTypeString(colorful));
	printf("Pin count: %d\n", pinCount);
	printf("Pins: \n");
	for (uint8_t i = 0; i < pinCount; i++) {
		printf("\t Index: %d\tPin: %d\tMode: %s\n",
		       i, pinAssignments[i], getModeString(i, colorful));
	}
	printf("\n");
} // busTypeIs
