// Handler for hardware devices.

#include "../HwHeader.h"
#include "Devices_interface.h"

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
uint8_t currentPinStates[MAX_PIN_COUNT]; // List of the current pin states, either last read or
                                         // written
uint8_t requestedPinStates[MAX_PIN_COUNT]; // List of pin states requested
uint8_t address; // Address for communications
bool deviceIsSetup = false;

// Hardware descirptor things
// const static uint8_t deviceTypeId = 0; // The type ID for this device based on HD class values
uint8_t deviceIndex; // Index of the device object

// For interacting with interfaces
uint8_t reservedPins[MAX_PIN_COUNT]; // Stores what interfaces reserved what pins.

/**
 * Init the variables for this object.
 * WARINGING: Do NOT overwrite this in subclasses! Instead, use the provided
 * deviceInit, whichi will be called by this method.
 * @return Whether init was successful
 */

virtual bool init(uint8_t index, uint8_t addr) = 0; /* getTypeId */

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
 * @return If handover was successful
 */

virtual bool attachInterface(uint8_t *pinNumbers, uint8_t interfaceId) = 0; // addInterface

/**
 * Drives a pin as requested from an interface. This will not actually change
 * the pin state yet, but will add it to the 'que'.
 * WARNING: SHOULD ONLY BE CALLED BY INTERFACES!
 *
 * @param pinNumber Number of the pin to drive
 * @param hd Full hardware descriptor
 * @return TODO
 */

virtual int drivePin(uint8_t pinState, uint8_t pinNumber, uint8_t
                     interfaceId) = 0;

virtual uint8_t getDeviceIndex() = 0;
}

;
