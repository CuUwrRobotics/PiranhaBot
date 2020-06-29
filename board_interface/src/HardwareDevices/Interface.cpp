/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

 #include "../HwHeader.h"
 #include "Devices_interface.h"
// #include "Device.cpp"
// #include "HardwareDescriptor.cpp"
// #include <cstring>

class Interface {
protected:
private:
public:

/**
 * @param pinNumber TODO
 * @return TODO
 */

virtual uint64_t getHardwareDescriptor(uint8_t pinNumber) = 0; // getHardwareDescriptor

/*
 * TODO: enumerate errors better
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
virtual int drivePin(uint8_t pinState, uint8_t pinNumber, uint64_t hd) = 0; /*
	                                                                           * verifyHardwareDescriptor
	                                                                           * */

uint8_t currentPinStates[MAX_PIN_COUNT]; // List of the current pin states, either last read or
                                         // written
uint8_t requestedPinStates[MAX_PIN_COUNT]; // List of pin states requested by other software

// How this interface will communicate with real hardware
Device *commDevice; // The device to actually communicate with
uint8_t pinsInUse[MAX_PIN_COUNT]; // List of pin usage states.
uint8_t pinMapToDevice[MAX_PIN_COUNT]; // Contains a map to the devices pins.

// Hardware descirptor things
uint8_t interfaceTypeId = HardwareDescriptor::INTF_PWR_SWITCHING; // The type ID for this
// interface based on HD
// class values
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The type ID for this
// interface based on HD class
// values
uint8_t interfaceIndex; // Index of the interface object
uint64_t baseHardwareDescriptor; // Stores all of the data for the hardware descriptor EXCEPT the

/**
 * @param device TODO
 * @param devicePins TODO
 * @param ifaceIndex TODO
 * @return TODO
 */

virtual bool start(Device *device, uint8_t *devicePins, uint8_t
                   ifaceIndex) = 0; // start

// /*
//  * Read the current pin state on the device.
//  * @param pinNumber The pin nunmber to try to drive
//  * @param hd The full hardware descriptor.
//  * @return 1 if there are no errors.
//  */
// virtual inline int getPinState(uint8_t pinNumber, uint64_t hd){
// 	// Check that HD matches
// 	if (hd != HardwareDescriptor::addPinNumber(baseHardwareDescriptor, pinNumber))
// 		return 0;
// 	// Try to drive a pin in commDevice
// 	commDevice.drivePin(pinNumber, hd);
// } /* verifyHardwareDescriptor */
}

;
