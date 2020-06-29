// Handler for hardware devices.

#include "HwHeader.h"
#include "Devices_interface.h"
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

virtual bool deviceInit(uint8_t addr) = 0;
protected:
public:

// Descriptions for this device
// uint8_t *currentPinStates[MAX_PIN_COUNT]; // List of pin states from last update
// uint8_t *requestedPinStates[MAX_PIN_COUNT]; // List of pin states requested
// uint8_t *currentPinModes[MAX_PIN_COUNT]; // List of pin r/w states requested
// uint8_t *requestedPinModes[MAX_PIN_COUNT]; // List of pin r/w states requested
PinBus currentPinBus; // Current state of all pins
PinBus requestedPinBus; // State of pins not yet set
bool readDataPending = false;
bool writeDataPending = false;
bool pinModeChangePending = false;
uint8_t address; // Address for communications
bool deviceIsSetup = false;

// Hardware descirptor things
// const static uint8_t deviceTypeId = 0; // The type ID for this device based on HD class values
uint8_t deviceIndex; // Index of the device object

// For interacting with interfaces
uint8_t *reservedPins[MAX_PIN_COUNT]; // Stores what interfaces reserved what pins.

/*
 * @return If given mode is valied for this device.
 */
virtual bool pinModeIsValid(PinMode mode) = 0;

/*
 * @return If given state is valied for this device.
 */
virtual bool pinStateIsValid(PinState state) = 0;

/**
 * Init the variables and device for this object.
 * WARINGING: Do NOT overwrite this in subclasses! Instead, use the provided
 * deviceInit, whichi will be called by this method.
 * @return Whether init was successful
 */

virtual bool init(uint8_t index, uint8_t addr, BusType busType) = 0; /* getTypeId */

/**
 * @return TODO
 */

virtual uint8_t getTypeId() = 0; /* getTypeId */

/**
 * @return TODO
 */

virtual bool ready() = 0; /* ready */

/**
 * Hands control to pins over to an interface. Once control is granted, nothing
 * else can control these pins.
 * @param pinNumbers An array of pin numbers to assigne to the interface.
 * @param pinModes An array of pin modes to assign to the given pins
 * @param interfaceId HD to be checked against.
 * @return If handover was successful
 */

virtual bool attachInterface(PinBus pinBus, uint8_t interfaceId) = 0; // addInterface

/**
 * Drives a pin as requested from an interface. This will not actually change
 * the pin state yet, but will add it to the 'que'.
 * WARNING: SHOULD ONLY BE CALLED BY INTERFACES!
 *
 * @param pinNumber Number of the pin to drive
 * @param hd Full hardware descriptor
 * @return TODO
 */

virtual int setPinState(uint8_t pinNumber, PinState pinState, uint8_t
                        interfaceId) = 0;

virtual uint8_t getDeviceIndex() = 0;

/* Updates any data from the device. If this device is readable, this will always check any pins
 * assigned as inputs. If it is not, if there is new data to be written, that data will be sent to
 * the device.
 * @return True if reads and writes were successful or unneccesary.
 */
virtual bool updateData() = 0;

/* Changes the pin modes of the given pins for an interface.
 * @return True if successful.
 */
virtual bool setPinModes(PinBus pinBus, uint8_t interfaceId) = 0;

/* Changes the pin modes of the given pins for an interface.
 * @return True if successful.
 */
virtual bool setPinMode(uint8_t pinNumber, PinMode pinMode, uint8_t
                        interfaceId) = 0;

/*
 * @return Pin mode as int.
 */
virtual PinMode getPinMode(uint8_t pin) = 0;

/*
 * @return
 */
virtual PinState getPinState(uint8_t pin) = 0;

/*
 * @return
 */
virtual uint8_t getPinInterfaceOnPin(uint8_t pin) = 0;


/*
 * @return
 */
PinBus getPinBus() {
	return currentPinBus;
};
}

;
