#ifndef PINBUS_H
#define PINBUS_H
#include <stdio.h>

enum BusType {BUS_INVALID, BUS_GPIO, BUS_ADC, BUS_PWM, BUS_OTHER};
// enum PinState {STATE_INVALID, STATE_ON, STATE_OFF, STATE_NONE, STATE_VARIABLE};
enum PinMode {MODE_INVALID, MODE_INPUT, MODE_OUTPUT}; // For handling data directions

/**
 * PinBus
 * @author
 */
class PinBus {
public:

bool setBusType(BusType type);

BusType getBusType();

bool setPinCount(uint8_t count);

inline bool setPin(uint8_t pin, PinMode mode);

bool setPinMode(uint8_t pin, PinMode mode);

// bool setPinState(uint8_t pin, PinState state);

PinMode getPinMode(uint8_t pin);

// PinState getPinState(uint8_t pin);

inline bool setPins(const PinMode *modes);

bool setAllPins(PinMode modes);

// bool setPinStates(const PinState *states);

bool setPinModes(const PinMode *modes);

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
inline uint8_t getPin(uint8_t index);

// Returns full array of pin assignemnts
inline uint8_t *getPins();

// Returns full array of pin assignemnts
inline uint8_t getPinCount();

bool createPinBus(BusType busType, uint8_t pinCount);

bool createPinBus(BusType busType, uint8_t *pinAssignments, uint8_t
                  pinCount,
                  const PinMode *pinModes);

bool createPinBusFromSet(BusType busType, uint8_t start, uint8_t end,
                         const PinMode *pinModes);

bool createUniformPinBusFromSet(BusType busType, uint8_t start,
                                uint8_t end,
                                const PinMode pinModes);

void resetAll();

bool busEquals(BusType pinType);

char *getModeString(uint8_t pin, bool colorizeBadOutputs);

// char *getStateString(uint8_t pin, bool colorizeBadOutputs);

char *getBusTypeString(bool colorizeBadOutputs);

void dumpInfo(bool colorful);

private:
// char ERROR_COLOR[10] = "\033[1;31m";
// char NO_COLOR[7] = "\033[0m";

const static uint8_t MAX_PINS = 16;

uint8_t pinAssignments[MAX_PINS] = {0xFF};
PinMode pinModes[MAX_PINS] = {MODE_INVALID};
// PinState pinStates[MAX_PINS] = {STATE_INVALID};
// Set up using invalid values so these need to be assigned before starting
uint8_t pinCount = MAX_PINS + 1;
BusType busType = BUS_INVALID;
}; // class PinBus

// class PinBus

// class PinBus

// class PinBus

// class PinBus

// Sketchy
#include "PinBus.cpp"
#endif /* ifndef PINBUS_H */
