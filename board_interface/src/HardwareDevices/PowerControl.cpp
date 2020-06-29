/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

 #include "../HwHeader.h"
 #include "Devices_interface.h"

/**
 * PowerControl
 * @author
 */
class PowerControl : public Interface {
private:
// Descriptions for this interface
uint8_t PIN_COUNT = 4; // Defined in subclasses; must be used to size all pin arrays.
uint8_t interfaceTypeId = HardwareDescriptor::INTF_PWR_SWITCHING; // The type ID for this
// interface based on HD
// class values
uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_GPIO; // The type ID for this
public:

/* Initialize the interface.
 * @param device Pointer for Device object that the interface communicates with. MUST already be
 * initialized
 * @param devicePins An array of the pin numbers to be used on the given device.
 * @param hd The full hardware descriptor.
 * @return true of sucessful
 */
bool start(Device *device, uint8_t *devicePins, uint8_t
           ifaceIndex) {
	// Verification code: don't put any init stuff here!
	// Check that device is the correct type
	if (parentDeviceTypeId != device->getTypeId()) {
		printf("ERROR: parentDeviceTypeId bad. Expeccted: 0x%.16X, Got: 0x%.16X\n",
		       parentDeviceTypeId, device->getTypeId());
		return false;
	}
	// Check that device is initialized
	if (!device->ready()) {
		printf("ERROR: parentDevice not ready\n");
		return false;
	}
	// Check that devicePins is not longer than PIN_COUNT
	if ((sizeof(devicePins) / sizeof(devicePins[0])) != PIN_COUNT) {
		printf("ERROR: device pins size bad, pin count = %d\n", PIN_COUNT);
		return false;
	}
	// Verification done. Add any code for initialization here.
	// ***************************************************************************
	// Some variable setups
	interfaceIndex = ifaceIndex;
	commDevice = device;
	// Create a base hardware descriptor that uses in invalid pin number
	baseHardwareDescriptor =
		HardwareDescriptor::createHardwareDescriptor(interfaceTypeId,
		                                             parentDeviceTypeId,
		                                             interfaceIndex,
		                                             commDevice->getDeviceIndex(),
		                                             HardwareDescriptor::
		                                             INVALID_PIN_NUMBER);
	for (uint8_t i = 0; i <= PIN_COUNT; i++) {
		pinMapToDevice[i] = devicePins[i];
		pinsInUse[i] = PIN_AVAILABLE; // Pins are not in use
	}
	// Grant this device control over the pins of the parent device
	// printf("Attaching interface...\n");
	return commDevice->attachInterface(pinMapToDevice, interfaceTypeId);

	// return true
} /* start */

/**
 * @param pinNumber TODO
 * @return TODO
 */

uint64_t getHardwareDescriptor(uint8_t pinNumber){
	return HardwareDescriptor::addPinNumber(baseHardwareDescriptor,
	                                        pinNumber);
} // getHardwareDescriptor

/*
 * TODO: enumerate errors better
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */
int drivePin(uint8_t pinState, uint8_t pinNumber, uint64_t hd){
	// Check that HD matches
	if (hd != HardwareDescriptor::addPinNumber(baseHardwareDescriptor,
	                                           pinNumber))
		return 0;
	// Try to drive a pin in commDevice
	return commDevice->drivePin(pinState, pinMapToDevice[pinNumber],
	                            HardwareDescriptor::getInterfaceId(hd));
} /* verifyHardwareDescriptor */
}

;
