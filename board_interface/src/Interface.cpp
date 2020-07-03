/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "PinBus.h"

/**
 * Interface
 * @author
 */
class Interface {
protected:
private:
public:

/**
 * @param pinNumber TODO
 * @return TODO
 */

// virtual uint64_t getHardwareDescriptor(uint8_t pinNumber) = 0; // getHardwareDescriptor

/*
 * TODO: enumerate errors better
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
// virtual int drivePin(uint8_t pinState, uint8_t pinNumber, uint64_t hd) = 0; /*
// * verifyHardwareDescriptor
// * */

/*
 * uint8_t currentPinStates[MAX_PIN_COUNT]; // List of the current pin states, either last read or
 *                                       // written
 * uint8_t requestedPinStates[MAX_PIN_COUNT]; // List of pin states requested by other software
 */
// How this interface will communicate with real hardware
Device *commDevice; // The device to actually communicate using
// uint8_t *pinMapToDevice[MAX_PIN_COUNT]; // Contains a map to the devices pins. Resized by
// subclass.
// uint8_t *pinModes[MAX_PIN_COUNT]; // The type ID for this. Resized by subclass.
PinBus pinBus;

// Hardware descirption things
uint8_t interfaceTypeId = HardwareDescriptor::INTF_INVALID; // The type ID
bool initSuccessful = false;
bool commDeviceExists = false;
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The type ID
uint8_t interfaceIndex = 0xFF; // Index of the interface object
uint64_t baseHardwareDescriptor; // Stores all of the data for the hardware descriptor EXCEPT the

/**
 * @param device TODO
 * @param devicePins TODO
 * @param ifaceIndex TODO
 * @return TODO
 */

virtual bool start(Device *device, PinBus pb, uint8_t ifaceIndex) = 0;

virtual int drivePin(PinState pinState, uint8_t pinNumber, uint64_t hd) = 0;

virtual int pinMode(PinMode pinMode, uint8_t pinNumber, uint64_t hd) = 0;

virtual uint64_t getHardwareDescriptor(uint8_t pinNumber) = 0;

/*
 * @return Pin mode as int.
 */
virtual PinMode getPinMode(uint8_t pin) = 0;

/*
 * @return Pin state as int.
 */
virtual PinState getPinState(uint8_t pin) = 0;

/*
 * @return This interface's type ID.
 */
virtual uint8_t getInterfaceTypeId() = 0;

/*
 * @return The parent device's index.
 */
// virtual uint8_t getParentDeviceIndex() = 0;
inline uint8_t getParentDeviceIndex(){
	if (!commDeviceExists)
		return HardwareDescriptor::DEVICE_INVALID;
	return commDevice->getDeviceIndex();
} // getParentDeviceIndex

/*
 * @return The parent device's index.
 */
virtual void printPins() = 0;

virtual bool ready() = 0;

virtual uint8_t getPinCount() = 0;

virtual uint8_t getDevPin(uint8_t pin) = 0;

/**
 * @return TODO
 */

PinBus getPinBus() {
	return pinBus;
} // getPinBus
}

;
