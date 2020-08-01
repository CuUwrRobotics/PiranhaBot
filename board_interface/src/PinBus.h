#ifndef PINBUS_H
#define PINBUS_H

#include <stdio.h>
#include <stdint.h>
#include "Logger.h"

enum BusType_t {BUS_INVALID, BUS_GPIO, BUS_ADC, BUS_PWM, BUS_OTHER};
// enum PinState {STATE_INVALID, STATE_ON, STATE_OFF, STATE_NONE, STATE_VARIABLE};
enum PinMode_t {MODE_INVALID, MODE_INPUT, MODE_OUTPUT}; // For handling data directions

/**
 * PinBus
 * @author
 */
class PinBus {
public:

bool setBusType(BusType_t type);

/**
 * @param type TODO
 * @return TODO
 */

inline BusType_t getBusType() {
	return busType;
} // getBusType

bool setPinCount(uint8_t count);

// inline bool setPin(uint8_t pin, PinMode_t mode);

bool setPinMode(uint8_t pin, PinMode_t mode);

/**
 * @param pin TODO
 * @return TODO
 */

inline PinMode_t getPinMode(uint8_t pin) {
	return pinModes[pin];
} // getPinMode

/**
 * @param modes TODO
 * @return TODO
 */

inline bool setPins(const PinMode_t *modes) {
	return (setPinModes(modes));
} // setPins

bool setAllPins(PinMode_t modes);

// bool setPinStates(const PinState *states);

bool setPinModes(const PinMode_t *modes);

// Sets a pin value. Can only happen once per pin.
bool assignPin(uint8_t index, uint8_t pinNumber);

// Sets a pin value. Can only happen once.
bool assignPins(uint8_t *pinNumbers, uint8_t pinNumbersLength);

/* Sets all pin values as a list from startPin to endPin. Can only happen once.
 * If not yet assigned, this will set pinCount.
 * endPin - startPin = pinCount.
 * Ex: endPin = 3, startPin = 8; pin assignments: {3, 4, 5, 6, 7, 8}
 */
bool assignPinSet(uint8_t startPin, uint8_t endPin);

// Returns the pin assignment for the pin at index
inline uint8_t getPin(uint8_t index) {
	return pinAssignments[index];
} // getPin

// Returns full array of pin assignemnts
inline uint8_t *getPins() {
	return pinAssignments;
} // getPins

// Returns full array of pin assignemnts
inline uint8_t getPinCount() {
	return pinCount;
} // getPins

bool createPinBus(BusType_t busType, uint8_t pinCount);

bool createPinBus(BusType_t busType, uint8_t *pinAssignments, uint8_t
                  pinCount,
                  const PinMode_t *pinModes);

bool createPinBusFromSet(BusType_t busType, uint8_t start, uint8_t end,
                         const PinMode_t *pinModes);

bool createUniformPinBusFromSet(BusType_t busType, uint8_t start,
                                uint8_t end,
                                const PinMode_t pinModes);

void resetAll();

bool busEquals(BusType_t pinType);

const char *getModeString(uint8_t pin, bool colorizeBadOutputs);

const char *getBusTypeString(bool colorizeBadOutputs);

void dumpInfo(bool colorful);

private:
// char ERROR_COLOR[10] = "\033[1;31m";
// char NO_COLOR[7] = "\033[0m";

const static uint8_t MAX_PINS = 16;

uint8_t pinAssignments[MAX_PINS] = {0xFF};
PinMode_t pinModes[MAX_PINS] = {MODE_INVALID};
// PinState pinStates[MAX_PINS] = {STATE_INVALID};
// Set up using invalid values so these need to be assigned before starting
uint8_t pinCount = MAX_PINS + 1;
BusType_t busType = BUS_INVALID;
}; // class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// Sketchy
// #include "PinBus.cpp"
#endif /* ifndef PINBUS_H */
