 #include "Interface.h"

// Device *commDevice; // The device to actually communicate using
// PinBus pinBus;
//
// bool initerrorVal = false;
// bool commDeviceExists = false;
// uint8_t interfaceIndex = 0xFF; // Index of the interface object

bool Interface::start(Device *device, PinBus pb, uint8_t ifaceIndex) {
	initerrorVal = false;
	// Verification code: don't put any init stuff here!
	// Check that device is the correct type
	commDevice = device;
	commDeviceExists = true;
	if (getParentTypeId() != commDevice->getDeviceTypeId()) {
		printf("ERROR: getParentTypeId() bad. Expeccted: 0x%.16X, Got: 0x%.16X\n",
		       getParentTypeId(), commDevice->getDeviceTypeId());
		return false;
	}
	// Check that device is initialized
	if (!commDevice->ready()) {
		printf("ERROR: parentDevice not ready\n");
		return false;
	}
	// Check that devicePins is not longer than getPinCount()
	if (pb.getPinCount() != getPinCount()) {
		printf("ERROR: device pins size bad: %d, pin count = %d\n",
		       pb.getPinCount(),
		       getPinCount());
		return false;
	}
	// Some variable setups
	interfaceIndex = ifaceIndex;
	pinBus = pb;

	if (commDevice->attachInterface(pinBus, getInterfaceTypeId())) {
		prepareInterface(); // Set interface-specific default modes to pinBus
		initerrorVal = true;
	} else ROS_ERROR("Interface #%d (%s) could not attatch to the parent device.",
		               interfaceIndex, getInterfaceName());
	return initerrorVal;

	// return true
} /* start */
