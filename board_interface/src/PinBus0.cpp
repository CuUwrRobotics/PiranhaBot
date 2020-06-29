/* Handles information for sets or pins, at least 1, at most 16.
 * Once set, some values are unchangable, like bus type and pin count.
 */
#include "HwHeader.h"
#include "PinBus.h"

/**
 * @param mode TODO
 * @param state TODO
 * @return TODO
 */

inline bool PinBus::setPin(uint8_t pin, PinBus::PinMode mode, PinBus::PinState
                           state){
	return (setPinMode(pin, mode) && setPinState(pin, state));
} // setPin

/**
 * @param mode TODO
 * @return TODO
 */

bool PinBus::setPinMode(uint8_t index, PinBus::PinMode mode){
	if (mode == INVALID_MODE) {
		printf("ERROR: invalid mode selected\n");
		return false;
	}
	if (!(index < 0 || index > MAX_PINS)) {
		pinModes[index] = mode;
		return true;
	}
	printf("BAD PIN CONFIG: Bad pin index: %d, can't assign mode.\n", index);
	return false;
} // setPinMode

/**
 * @param state TODO
 * @return TODO
 */

bool PinBus::setPinState(uint8_t index, PinBus::PinState state){
	if (state == INVALID_STATE) {
		printf("BAD PIN CONFIG: invalid state selected\n");
		return false;
	}
	if (!(index < 0 || index >= MAX_PINS)) {
		pinStates[index] = state;
		return true;
	}
	printf("BAD PIN CONFIG: Bad index number: %d, can't assign state.\n", index);
	return false;
} // setPinState

/**
 * @param modes TODO
 * @param states TODO
 * @return TODO
 */

inline bool PinBus::setPins(const PinBus::PinMode *modes, const
                            PinBus::PinState *states) {
	return (setPinModes(modes) && setPinStates(states));
} // setPins

/**
 * @param modes TODO
 * @param states TODO
 * @return TODO
 */

bool PinBus::setAllPins(PinBus::PinMode mode, PinBus::PinState state) {
	if (pinCount == MAX_PINS + 1) {
		printf(
			"BAD PIN CONFIG: Pin count not yet set, can't assign states/modes.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		pinStates[i] = state;
		pinModes[i] = mode;
	}
	return true;
} // setPins

/**
 * @param states TODO
 * @return TODO
 */

bool PinBus::setPinStates(const PinBus::PinState *states){
	if (pinCount == MAX_PINS + 1) {
		printf("BAD PIN CONFIG: Pin count not yet set, can't assign states.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		if (states[i] == INVALID_STATE) {
			printf("BAD PIN CONFIG: Invalid state selected for pin #%d\n", i);
			continue;
		}
		pinStates[i] = states[i];
	}
	return true;
} // setPinStates

/**
 * @param states TODO
 * @return TODO
 */

bool PinBus::setPinModes(const PinBus::PinMode *modes){
	if (pinCount == MAX_PINS + 1) {
		printf("BAD PIN CONFIG: Pin count not yet set, can't assign modes.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		if (modes[i] == INVALID_MODE) {
			printf("BAD PIN CONFIG: Invalid mode selected for pin #%d\n", i);
			continue;
		}
		pinModes[i] = modes[i];
	}
	return true;
} // setPinModes

// Assign a pin. Can only happen once per pin.
bool PinBus::assignPin(uint8_t index, uint8_t pinNumber) {
	if (pinCount == MAX_PINS + 1) {
		printf("BAD PIN CONFIG: pinCount not yet set, cannot assign pin\n", index);
		return false;
	}
	// Check index
	if ((index < 0 || index >= pinCount)) {
		printf("BAD PIN CONFIG: Could not find pin for assignment: %d\n", index);
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
		printf("BAD PIN CONFIG: Bad pin count for assignment: %d\n",
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
		printf("BAD PIN CONFIG: assign pins: start value cannot be morethan end\n");
		return false;
	}
	uint8_t newPinCount = (endPin - startPin) + 1; // Get a new pin count
	// Pin count was not yet set
	if (pinCount == MAX_PINS + 1) {
		// Verify the new pin count
		if (newPinCount <= 0 || newPinCount > MAX_PINS) {
			printf(
				"BAD PIN CONFIG: assign pins: bad pin count computed from start/end values\n");
			return false;
		}
		pinCount = newPinCount;
		// return true;
	}	else if (pinCount != newPinCount) {
		printf(
			"BAD PIN CONFIG: assign pins: pin count computed from start / end values does not match preset pin count\n");
		return false;
	}
	// Make assignemnts
	for (uint8_t pin = 0; pin < pinCount; pin++) {
		pinAssignments[pin] = startPin + pin;
	}
	return true;
} // assignPinSet

// Returns the pin assignment for the pin at index
inline uint8_t PinBus::getPin(uint8_t index) {
	return pinAssignments[index];
} // getPin

// Returns full array of pin assignemnts
inline uint8_t *PinBus::getPins() {
	return pinAssignments;
} // getPins

// Returns full array of pin assignemnts
inline uint8_t PinBus::getPinCount() {
	return pinCount;
} // PinBus::getPins

/**
 * @param type TODO
 * @return TODO
 */

bool PinBus::setBusType(PinBus::BusType type) {
	if (busType != INVALID_BUS) {
		printf("%sBAD PIN CONFIG: Bus type already assigned!%s\n", ERROR_COLOR,
		       NO_COLOR);
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
		printf("%sBAD PIN CONFIG: Pin count already assigned!%s\n", ERROR_COLOR,
		       NO_COLOR);
		return false;
	}
	if (!(count < 0 || count > MAX_PINS)) {
		pinCount = count;
		return true;
	}
	printf("%sBAD PIN CONFIG: Bad pin count: %d%s\n", ERROR_COLOR, count,
	       NO_COLOR);
	return false;
} // setBusType

/**
 * @param busType TODO
 * @param pinCount TODO
 */

bool PinBus::createPinBus(PinBus::BusType busType, uint8_t pinCount){
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
 * @param pinStates TODO
 */

bool PinBus::createPinBus(PinBus::BusType busType, uint8_t *pinAssignments,
                          uint8_t pinCount,
                          const PinBus::PinMode *pinModes, const
                          PinBus::PinState *pinStates){
	if (!createPinBus(busType, pinCount))
		return false;
	// this->busType = busType;
	// this->pinCount = pinCount;
	if (!(assignPins(pinAssignments, pinCount)))
		return false;
	if (!setPins(pinModes, pinStates))
		return false;
	return true;
} // createPinBus

/**
 * @param busType TODO
 * @param start TODO
 * @param end TODO
 * @param pinCount TODO
 * @param pinModes TODO
 * @param pinStates TODO
 * @return TODO
 */

bool PinBus::createPinBusFromSet(PinBus::BusType busType, uint8_t start,
                                 uint8_t end,
                                 const PinBus::PinMode *pinModes,
                                 const PinBus::PinState *pinStates){
	if (!createPinBus(busType, ((end - start) + 1)))
		return false;
	if (!(assignPinSet(start, end)))
		return false;
	if (!setPins(pinModes, pinStates))
		return false;
	return true;
} // createPinBus

/**
 * @param busType TODO
 * @param start TODO
 * @param end TODO
 * @param pinMode TODO
 * @param pinState TODO
 * @return TODO
 */

bool PinBus::createUniformPinBusFromSet(PinBus::BusType busType, uint8_t start,
                                        uint8_t end,
                                        PinBus::PinMode pinMode,
                                        PinBus::PinState pinState){
	if (!createPinBus(busType, ((end - start) + 1)))
		return false;
	if (!(assignPinSet(start, end)))
		return false;
	if (!setAllPins(pinMode, pinState))
		return false;
	return true;
} // createPinBus

/**
 * @param pinType TODO
 * @return TODO
 */

bool PinBus::busEquals(PinBus::BusType pinType) {
	return (pinType == busType);
} // busTypeIs

/**
 * @param pin TODO
 * @return TODO
 */

char *PinBus::getModeString(uint8_t pin) {
	switch (pinModes[pin]) {
	case PinBus::INVALID_MODE:
		return "INVALID_MODE";
		break;
	case PinBus::GPIO_INPUT_MODE:
		return "GPIO_INPUT_MODE";
		break;
	case PinBus::GPIO_INPUT_X_MODE:
		return "GPIO_INPUT_X_MODE";
		break;
	case PinBus::GPIO_OUTPUT_MODE:
		return "GPIO_OUTPUT_MODE";
		break;
	case PinBus::PWM_ON_MODE:
		return "PWM_ON_MODE";
		break;
	case PinBus::PWM_OFF_MODE:
		return "PWM_OFF_MODE";
		break;
	default:
		return "unknown_mode";
		break;
	} // switch
} // busTypeIs

/**
 * @param pin TODO
 * @return TODO
 */

char *PinBus::getStateString(uint8_t pin) {
	switch (pinStates[pin]) {
	case PinBus::INVALID_STATE:
		return "INVALID_STATE";
		break;
	case PinBus::ON_STATE:
		return "ON_STATE";
		break;
	case PinBus::OFF_STATE:
		return "OFF_STATE";
		break;
	case PinBus::NO_STATE: // For inputs
		return "NO_STATE";
		break;
	case PinBus::OTHER_STATE:
		return "OTHER_STATE";
		break;
	default:
		return "unknown_state";
		break;
	} // switch
} // busTypeIs

/**
 * @return TODO
 */

char *PinBus::getBusTypeString(){
	switch (busType) {
	case PinBus::INVALID_BUS:
		return "INVALID_BUS";
		break;
	case PinBus::GPIO_BUS:
		return "GPIO_BUS";
		break;
	case PinBus::ANALOG_BUS:
		return "ANALOG_BUS";
		break;
	case PinBus::PWM_BUS:
		return "PWM_BUS";
		break;
	case PinBus::OTHER_BUS:
		return "OTHER_BUS";
		break;
	default:
		return "unknown_bus";
		break;
	} // switch
} // PinBus::getBusTypeString

/**
 * @return TODO
 */

void PinBus::dumpInfo() {
	printf("\nPinBus info dump: \n");
	printf("Pin count: %d\n", pinCount);
	printf("Pins: \n");
	for (uint8_t i = 0; i < pinCount; i++) {
		printf("\t Index: %d\tPin: %d\tMode: %s\tState: %s\n",
		       i, pinAssignments[i], getModeString(i), getStateString(i));
	}
	printf("\n");
} // busTypeIs
