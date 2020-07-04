/**
 * @param direction defines whether to make the smallest pin the smallest on-time
 * (true) or the largest (false)
 */

	#include "HwHeader.h"
	#include "BitTesting.h"

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPwm(Interface *intf, Device *dev, bool direction) {
	bool returnVal = true;
	uint64_t hd;
	uint16_t frequency;
	uint16_t data;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf("PWM test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
	       WHITE, intf->getInterfaceIndex(), NO_COLOR,
	       WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	frequency = 0xFFFF; // Set to maximum value; should get brought down to 3500
	// Writes frequency to a different pin than is chacked later, since this
	// should be set universally.
	hd = intf->getHardwareDescriptor(1);
	intf->writePin(1, &frequency, PACKET_PWM_FREQ, hd);
	// Start pin assignements
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		hd = intf->getHardwareDescriptor(pin);
		data = 256 * pin;
		if (direction) data = 4095 - data;
		intf->writePin(pin, &data, PACKET_PWM_ON_TICKS, hd);
	}
	dev->updateData(); // Push data out to the chip
	printf("Results:\n========\n");
	// All pins should have same freq
	frequency = intf->readPin(0, PACKET_PWM_FREQ);
	printf("Frequency: ");
	if (frequency != 3500) {
		returnVal = false;
		printf("%s", RED);
	} else {
		printf("%s", GREEN);
	}
	printf("%d%s\n", frequency, NO_COLOR);
	bool dataGood = false;
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		data = intf->readPin(pin, PACKET_PWM_ON_TICKS);
		if (direction) {
			// data = 4095 - data;
			dataGood = (data == 4095 - (256 * pin));
		} else {
			dataGood = (data == 256 * pin);
		}
		printf(" - Pin %s%d%s:\t%s%d%s\n", WHITE, pin, NO_COLOR,
		       (dataGood) ? GREEN : RED, data, NO_COLOR);
		if (!dataGood) returnVal = false;
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
	// }
} // testPwm

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testGpio(Interface *intf, Device *dev) {
	bool returnVal = true;
	uint64_t hd;
	uint16_t data;
	bool dataGood;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf("GPIO test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
	       WHITE, intf->getInterfaceIndex(), NO_COLOR,
	       WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! interface does not match parent device.%s\n",
		       RED, NO_COLOR);
		returnVal = false;
	}

	printf("Results:\n========\n");
	// Sets one pin per cycle to low (first) then high over 32 total cycles
	for (int highLow = 0; highLow < 2; highLow++) {
		printf("Incrementing %s test:\n", (highLow ? "ON" : "OFF"));
		for (int cycle = 0; cycle < intf->getPinCount(); cycle++) { // Run 16 times
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
				hd = intf->getHardwareDescriptor(pin);
				data = ((pin == cycle) == highLow); // Set one pin per cycle to HIGH
				intf->writePin(pin, &data, PACKET_GPIO_STATE, hd);
			}
		}
		dev->updateData(); // Push data out to the chip
		for (int cycle = 0; cycle < intf->getPinCount(); cycle++) { // Run 16 times
			printf("\tCycle %d Pins:\t|", cycle);
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
				data = intf->readPin(pin, PACKET_GPIO_STATE); // get pin data
				if (data == (((pin == cycle) == highLow) ? 1 : 0)) // Check pin data
					dataGood = true;
				else {
					dataGood = false;
					returnVal = false;
				}
				printf("%s%s", (dataGood ? GREEN : RED),
				       (((pin == cycle) == highLow) ? "." : "'"));
			}
			printf("%s|\n", NO_COLOR);
		}
	}

	// Sets pins to 0101 pattern, then 1010
	printf("Patterned test:\n");
	for (int highLow = 0; highLow < 2; highLow++) {
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
			hd = intf->getHardwareDescriptor(pin);
			data = ((pin % 2) == highLow); // Set one pin per cycle to HIGH
			intf->writePin(pin, &data, PACKET_GPIO_STATE, hd);
		}
		dev->updateData(); // Push data out to the chip
		printf("\tPins:\t\t|");
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
			data = intf->readPin(pin, PACKET_GPIO_STATE); // get pin data
			if (data == (((pin % 2) == highLow) ? 1 : 0)) // Check pin data
				dataGood = true;
			else {
				dataGood = false;
				returnVal = false;
			}
			printf("%s%s", (dataGood ? GREEN : RED),
			       (((pin % 2) == highLow) ? "." : "'"));
		}
		printf("%s|\n", NO_COLOR);
	}

	// dev->updateData(); // Push data out to the chip
	// printf("Results:\n========\n");
	// // All pins should have same freq
	// frequency = intf->readPin(0, PACKET_PWM_FREQ);
	// printf("Frequency: ");
	// if (frequency != 3500) {
	// 	returnVal = false;
	// 	printf("%s", RED);
	// } else {
	// 	printf("%s", GREEN);
	// }
	// printf("%d%s\n", frequency, NO_COLOR);
	// bool dataGood = false;
	// for (int pin = 0; pin < intf->getPinCount(); pin++) {
	// 	data = intf->readPin(pin, PACKET_PWM_ON_TICKS);
	// 	if (direction) {
	// 		// data = 4095 - data;
	// 		dataGood = (data == 4095 - (256 * pin));
	// 	} else {
	// 		dataGood = (data == 256 * pin);
	// 	}
	// 	printf(" - Pin %s%d%s:\t%s%d%s\n", WHITE, pin, NO_COLOR,
	// 	       (dataGood) ? GREEN : RED, data, NO_COLOR);
	// 	if (!dataGood) returnVal = false;
	// }

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
	// }
} // testPwm

/**
 * Fancy display of all devices, interfaces, and pins.
 * TODO: At end, cycle through and see if any interfaces are unconnected.
 */

void dumpConfiguration(bool shrinkRepeatedPins, Interface **interfaces,
                       Device **devices){
	char SP[4] = "  "; // Spacing for the tree view
	bool moreDevfLeft = true;
	bool moreIntfLeft = true;
	bool morePinsLeft = true;
	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf("\nCONFIGURATION DUMP:\n");
	for (uint8_t dev = 0; dev < TOTAL_DEVICES; dev++) {
		if (dev == TOTAL_DEVICES - 1) {
			printf("└─");
			moreDevfLeft = false;
		}	else {
			printf("├─"); // Formatting
			moreDevfLeft = true;
		}
		printf("Device #%s%d%s, Type: %s%s%s, Bus: %s\n", WHITE, dev, NO_COLOR,
		       devices[dev]->ready() ? GREEN :
		       RED,
		       HardwareDescriptor::deviceIdToCharArray(
						 devices[dev]->getDeviceTypeId()), NO_COLOR,
		       devices[dev]->getPinBus().getBusTypeString(true));
		for (uint8_t intf = 0; intf < TOTAL_INTERFACES; intf++) {
			// if (!interfaces[intf]->ready())
			// 	continue;
			if (interfaces[intf]->getParentDeviceIndex() == dev) {
				moreIntfLeft = false;
				for (uint8_t i = intf + 1; i < TOTAL_INTERFACES; i++)
					if (interfaces[i]->getParentDeviceIndex() == dev) {
						moreIntfLeft = true;
						break;
					}
				printf("%s%s", moreDevfLeft ? "│" : " ", SP); // Formatting
				printf("%s─", moreIntfLeft ? "├" : "└"); // Formatting
				printf("Interface #%s%d%s, Type: %s%s%s\n", WHITE, intf,
				       NO_COLOR, interfaces[intf]->ready() ? GREEN : RED,
				       HardwareDescriptor::interfaceIdToCharArray(
								 interfaces[intf]->getInterfaceTypeId()), NO_COLOR);
				printf("%s%s", moreDevfLeft ? "│" : " ", SP); // Formatting
				printf("%s%s", moreIntfLeft ? "│" : " ", SP); // Formatting
				printf("└─"); // Formatting
				if (!interfaces[intf]->ready()) {
					printf("Interface not ready, no pins to show.\n");
					continue;
				}
				printf("Pin count: %s%d%s\n", WHITE,
				       interfaces[intf]->getPinCount(), NO_COLOR);
				morePinsLeft = true;
				for (int pin = 0; pin < interfaces[intf]->getPinCount(); pin++) {
					if (shrinkRepeatedPins && pin != 0) { // Finds if pins are identical
						uint8_t firstPin = pin;
						// While the current pin matches previous, increment pin # up
						while (pin < interfaces[intf]->getPinCount() &&
						       interfaces[intf]->getPinBus().getPinMode(pin) ==
						       interfaces[intf]->getPinBus().getPinMode(pin - 1) &&
						       interfaces[intf]->getPinBus().getPinState(pin) ==
						       interfaces[intf]->getPinBus().getPinState(pin - 1)) {
							pin++;
						}
						// If incrementing actually happened
						if (pin == interfaces[intf]->getPinCount()) // Last pin?
							morePinsLeft = false;
						printf("%s%s%s%s%s%s─", moreDevfLeft ? "│" : " ", SP,
						       moreIntfLeft ? "│" : " ",  SP, SP, morePinsLeft ? "├" : "└"); // Formatting
						if (firstPin != pin && firstPin != pin - 1) {
							pin--;
							printf("%s[also %d ... %d]%s", D_GRAY,
							       interfaces[intf]->getDevPin(firstPin),
							       interfaces[intf]->getDevPin(pin),
							       NO_COLOR);
							printf("\n"); // Formatting
						} else { // Incrementing did NOT happen, so act normal.
							if (firstPin == pin - 1) // Incrementing did happen, but only once.
								pin--;
							printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getDevPin(pin),
							       NO_COLOR);
							printf("%s, ", interfaces[intf]->getPinBus().
							       getModeString(pin, true));
							printf("%s ", interfaces[intf]->getPinBus().
							       getStateString(pin, true));
							// printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getDevPin(pin),
							//        NO_COLOR);
							// printf("Count: %d ", interfaces[intf]->getPinBus().getPinCount());
							printf("\n"); // Formatting
						}
					} else {
						if (pin == interfaces[intf]->getPinCount() - 1) // Last pin?
							morePinsLeft = false;
						printf("%s%s%s%s%s%s─", moreDevfLeft ? "│" : " ", SP,
						       moreIntfLeft ? "│" : " ",  SP, SP, morePinsLeft ? "├" : "└"); // Formatting
						printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getDevPin(pin),
						       NO_COLOR);
						printf("%s, ", interfaces[intf]->getPinBus().
						       getModeString(pin, true));
						printf("%s ", interfaces[intf]->getPinBus().
						       getStateString(pin, true));
						// printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getDevPin(pin),
						//        NO_COLOR);
						// printf("Count: %d ", interfaces[intf]->getPinBus().getPinCount());
						printf("\n"); // Formatting
					}
				}
			}
		}
	}
	// List any devices ignored due to bad initializations
	for (uint8_t intf = 0; intf < TOTAL_INTERFACES; intf++) {
		if (interfaces[intf]->getParentDeviceIndex() ==
		    HardwareDescriptor::DEVICE_INVALID) {
			printf(
				"%sInterface #%d not listed due to a bad initialization.\nMost likely that start() was never called.%s\n",
				RED, intf, NO_COLOR);
		}
	}
	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
} // dumpConfigruation
