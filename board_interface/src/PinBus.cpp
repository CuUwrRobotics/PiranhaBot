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

inline bool PinBus::setPin(uint8_t pin, PinMode mode, PinState
                           state){
	return (setPinMode(pin, mode) && setPinState(pin, state));
} // setPin

/**
 * @param mode TODO
 * @return TODO
 */

bool PinBus::setPinMode(uint8_t index, PinMode mode){
	if (mode == MODE_INVALID) {
		ROS_ERROR("PinBus::setPinMode: invalid mode selected\n");
		return false;
	}
	if (!(index < 0 || index > MAX_PINS)) {
		pinModes[index] = mode;
		return true;
	}
	ROS_ERROR("PinBus::setPinMode: Bad pin index: %d, can't assign mode.\n",
	          index);
	return false;
} // setPinMode

/**
 * @param state TODO
 * @return TODO
 */

bool PinBus::setPinState(uint8_t index, PinState state){
	if (state == STATE_INVALID) {
		ROS_ERROR("PinBus::setPinState: invalid state selected\n");
		return false;
	}
	if (!(index < 0 || index >= MAX_PINS)) {
		pinStates[index] = state;
		return true;
	}
	ROS_ERROR("PinBus::setPinState: Bad index number: %d, can't assign state.\n",
	          index);
	return false;
} // setPinState

/**
 * @param pin TODO
 * @return TODO
 */

inline PinMode PinBus::getPinMode(uint8_t pin) {
	return pinModes[pin];
} // PinBus::getPinMode

/**
 * @param pin TODO
 * @return TODO
 */

inline PinState PinBus::getPinState(uint8_t pin){
	return pinStates[pin];
} // PinBus::getPinState

/**
 * @param modes TODO
 * @param states TODO
 * @return TODO
 */

inline bool PinBus::setPins(const PinMode *modes, const
                            PinState *states) {
	return (setPinModes(modes) && setPinStates(states));
} // setPins

/**
 * @param modes TODO
 * @param states TODO
 * @return TODO
 */

bool PinBus::setAllPins(PinMode mode, PinState state) {
	if (pinCount == MAX_PINS + 1) {
		ROS_ERROR(
			"PinBus::setAllPins: Pin count not yet set, can't assign states/modes.\n");
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

bool PinBus::setPinStates(const PinState *states){
	if (pinCount == MAX_PINS + 1) {
		ROS_ERROR(
			"PinBus::setPinStates: Pin count not yet set, can't assign states.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		if (states[i] == STATE_INVALID) {
			ROS_ERROR("PinBus::setPinStates: Invalid state selected for pin #%d\n",
			          i);
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

bool PinBus::setPinModes(const PinMode *modes){
	if (pinCount == MAX_PINS + 1) {
		ROS_ERROR(
			"PinBus::setPinModes: Pin count not yet set, can't assign modes.\n");
		return false;
	}
	for (uint8_t i = 0; i < pinCount; i++) {
		if (modes[i] == MODE_INVALID) {
			ROS_ERROR("PinBus::setPinModes: Invalid mode selected for pin #%d\n", i);
			continue;
		}
		pinModes[i] = modes[i];
	}
	return true;
} // setPinModes

// Assign a pin. Can only happen once per pin.
bool PinBus::assignPin(uint8_t index, uint8_t pinNumber) {
	if (pinCount == MAX_PINS + 1) {
		ROS_ERROR("PinBus::assignPin: pinCount not yet set, cannot assign pin\n",
		          index);
		return false;
	}
	// Check index
	if ((index < 0 || index >= pinCount)) {
		ROS_ERROR("PinBus::assignPin: Could not find pin for assignment: %d\n",
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
		ROS_ERROR("PinBus::assignPins: Bad pin count for assignment: %d\n",
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
		ROS_ERROR(
			"PinBus::assignPinSet: assign pins: start value cannot be morethan end\n");
		return false;
	}
	uint8_t newPinCount = (endPin - startPin) + 1; // Get a new pin count
	// Pin count was not yet set
	if (pinCount == MAX_PINS + 1) {
		// Verify the new pin count
		if (newPinCount <= 0 || newPinCount > MAX_PINS) {
			ROS_ERROR(
				"PinBus::assignPinSet: assign pins: bad pin count computed from start/end values\n");
			return false;
		}
		pinCount = newPinCount;
		// return true;
	}	else if (pinCount != newPinCount) {
		ROS_ERROR(
			"PinBus::assignPinSet: assign pins: pin count computed from start / end values does not match preset pin count\n");
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

bool PinBus::setBusType(BusType type) {
	if (busType != BUS_INVALID) {
		ROS_ERROR("PinBus::setBusType: Bus type already assigned!\n");
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
		ROS_ERROR("PinBus::setPinCount: Pin count already assigned!\n");
		return false;
	}
	if (!(count < 0 || count > MAX_PINS)) {
		pinCount = count;
		return true;
	}
	printf("PinBus::setPinCount: Bad pin count: %d\n", count);
	return false;
} // setBusType

/**
 * @param busType TODO
 * @param pinCount TODO
 */

bool PinBus::createPinBus(BusType busType, uint8_t pinCount){
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

bool PinBus::createPinBus(BusType busType, uint8_t *pinAssignments,
                          uint8_t pinCount,
                          const PinMode *pinModes, const
                          PinState *pinStates){
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

bool PinBus::createPinBusFromSet(BusType busType, uint8_t start,
                                 uint8_t end,
                                 const PinMode *pinModes,
                                 const PinState *pinStates){
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

bool PinBus::createUniformPinBusFromSet(BusType busType, uint8_t start,
                                        uint8_t end,
                                        PinMode pinMode,
                                        PinState pinState){
	if (!createPinBus(busType, ((end - start) + 1)))
		return false;
	if (!(assignPinSet(start, end)))
		return false;
	if (!setAllPins(pinMode, pinState))
		return false;
	return true;
} // createPinBus

/**
 * Full reset of the object and pins
 */

void PinBus::resetAll(){
	for (uint8_t i = 0; i < MAX_PINS; i++) {
		pinStates[i] = STATE_INVALID;
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

bool PinBus::busEquals(BusType pinType) {
	return (pinType == busType);
} // busTypeIs

/**
 * @param pin TODO
 * @return TODO
 */

// char RED[10] = "\033[1;31m";
// char NO_COLOR[7] = "\033[0m";
char *PinBus::getModeString(uint8_t pin, bool colorizeBadOutputs) {
	switch (pinModes[pin]) {
	case MODE_INVALID:
		if (colorizeBadOutputs)
			return "\033[1;31mMODE_INVALID\033[0m";
		else return "MODE_INVALID";
		break;
	case MODE_GPIO_INPUT:
		return "MODE_GPIO_INPUT";
		break;
	case MODE_GPIO_INPUT_X:
		return "MODE_GPIO_INPUT_X";
		break;
	case MODE_GPIO_OUTPUT:
		return "MODE_GPIO_OUTPUT";
		break;
	case MODE_PWM_ON:
		return "MODE_PWM_ON";
		break;
	case MODE_PWM_OFF:
		return "MODE_PWM_OFF";
		break;
	default:
		if (colorizeBadOutputs)
			return "\033[1;31munknown_mode\033[0m";
		else return "unknown_mode";
		break;
	} // switch
} // busTypeIs

/**
 * @param pin TODO
 * @return TODO
 */

char *PinBus::getStateString(uint8_t pin, bool colorizeBadOutputs) {
	switch (pinStates[pin]) {
	case STATE_INVALID:
		if (colorizeBadOutputs)
			return "\033[1;31mSTATE_INVALID\033[0m";
		else return "STATE_INVALID";
		break;
	case STATE_ON:
		return "STATE_ON";
		break;
	case STATE_OFF:
		return "STATE_OFF";
		break;
	case STATE_NONE: // For inputs
		return "STATE_NONE";
		break;
	case STATE_VARIABLE:
		return "STATE_VARIABLE";
		break;
	default:
		if (colorizeBadOutputs)
			return "\033[1;31munknown_state\033[0m";
		else return "unknown_state";
		break;
	} // switch
} // busTypeIs

/**
 * @return TODO
 */

char *PinBus::getBusTypeString(bool colorizeBadOutputs){
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
		printf("\t Index: %d\tPin: %d\tMode: %s\tState: %s\n",
		       i, pinAssignments[i], getModeString(i, colorful),
		       getStateString(i, colorful));
	}
	printf("\n");
} // busTypeIs
