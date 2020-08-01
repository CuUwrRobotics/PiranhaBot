/**
 * @param direction defines whether to make the smallest pin the smallest on-time
 * (true) or the largest (false)
 */

#include "BitTesting.h"

namespace bit_testing {
/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPwm(Interface *intf, Device *dev, bool direction) {
	bool returnVal = true; // Set to false on error

	InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For pin values
	DataError_t errorVal; // For storing error codes from setting values and configs

	float frequency;
	float data;

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

	// Make sure pin modes can't be altred by the interface
	intf->setPinMode(0, MODE_INPUT);
	dev->updateData();
	if (intf->getPinMode(0) != MODE_OUTPUT) {
		printf(
			"%sPWM interface's pin modes was changed from output, which should not be possible.%s\n",
			RED, NO_COLOR);
		intf->setPinMode(0, MODE_INPUT); // Fix the mode
		dev->updateData();
		returnVal = false; // Return failure for test
	}

	frequency = 4000; // Set above maximum value; should get brought down to 3500
	// Writes frequency to a different pin than the one checked later, since this
	// should be set universally.
	val.fmt = VALUE_PWM_FREQ; // Set format
	val.pin = 1;
	val.data = &frequency;
	errorVal = intf->writePin(&val);
	if (!(errorVal == ERROR_SUCCESS)) {
		log_error("Could not write frequency to interface %d: %s",
		          intf->getInterfaceIndex(), errorCharArray(errorVal));
		returnVal = false; // Failure for test
	}
	// Start pin assignements
	val.fmt = VALUE_PWM_ON_TICKS; // Set format once
	val.data = &data;
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		// hd = intf->getHardwareDescriptor(pin);
		if (direction)
			data = 4095 - (256 * pin);
		else
			data = 256 * pin;
		val.pin = pin;
		intf->writePin(&val);
	}
	dev->updateData(); // Push data out to the chip
	printf("Results:\n========\n");
	// All pins should have same freq
	val.fmt = VALUE_PWM_FREQ; // Set format
	val.pin = 0;
	val.data = &frequency;
	errorVal = intf->readPin(&val);
	if (!(errorVal == ERROR_SUCCESS)) {
		log_error("Did not read same frequency after writing to interface %d: %s",
		          intf->getInterfaceIndex(), errorCharArray(errorVal));
		returnVal = false; // Failure for test
	}
	printf("Frequency: ");
	if (frequency != 3500) {
		returnVal = false;
		printf("%s", RED);
	} else {
		printf("%s", GREEN);
	}
	printf("%f %s\n", frequency, NO_COLOR);
	// Check all values
	bool dataGood = false;
	val.fmt = VALUE_PWM_ON_TICKS; // Set format once
	val.data = &data;
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		val.pin = pin;
		errorVal = intf->readPin(&val);
		if (direction) {
			dataGood = (data == 4095 - (256 * pin));
		} else {
			dataGood = (data == 256 * pin);
		}
		printf(" - Pin %s%d%s:\t%s%f %s\n", WHITE, pin, NO_COLOR,
		       (dataGood) ? GREEN : RED, data, NO_COLOR);
		if (!dataGood) returnVal = false;
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

// Turn everything back off
	val.fmt = VALUE_PWM_ON_TICKS; // Set format
	data = 0;
	val.data = &data;
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		val.pin = pin;
		intf->writePin(&val);
	}

	return returnVal;
} // testPwm

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testGpio(Interface *intf, Device *dev) {
	bool returnVal = true; // Set to false on error

	// InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For pin values
	DataError_t errorVal; // For storing error codes from setting values and configs

	float data;
	float correctData;
	float previousData;
	bool dataGood = true;

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

	// Tests pin modes
	// ***************************************************************************
	printf(
		"Pin Modes Test (setPinValue errors are normal and part of the test)\n");
	printf("Results:\n========\n");
	// Pin 0 (mode is OUTPUT, so this should work)
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.pin = 0;
	val.data = &data; // This will be read and written to multiple times in the test

	// Set pin mode first
	intf->setPinMode(0, MODE_OUTPUT);
	dev->updateData();
	if (intf->getPinMode(0) != MODE_OUTPUT) {
		printf("%sPin 0 could not be set as output.%s\n",
		       RED, NO_COLOR);
		returnVal = false; // Return failure for test
	}
	// Set pin 0 high/low, make sure it works
	data = 1; // Set one pin per cycle to HIGH
	errorVal = intf->writePin(&val);
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != 1) // Check pin data
		dataGood = false;
	data = 0; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != 0) // Check pin data
		dataGood = false;
	if (dataGood == false) {
		printf(
			"%sTried to set pin 0 was set as output, but output on pin 0 could not be written to.%s\n",
			RED, NO_COLOR);
		dataGood = true; // Reset for next step
		returnVal = false; // Return failure for test
	}

	// Pin 1 (mode is INPUT, so this should not work)
	val.pin = 1; // Format and data already set, so only pin needs a reassignment.
	// Set pin mode first
	intf->setPinMode(1, MODE_INPUT);
	dev->updateData();
	if (intf->getPinMode(1) != MODE_INPUT) {
		printf("%sCould not set pin 1 as input.%s\n",
		       RED, NO_COLOR);
		returnVal = false; // Return failure for test
	}
	// Set pin 1 high/low, make sure it does not change

	// Pin 1 on
	intf->readPin(&val);
	previousData = data;
	data = 1; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != previousData) // If data changed
		dataGood = false;
	// Pin 1 off
	intf->readPin(&val);
	previousData = data;
	data = 0; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != previousData) // If data changed
		dataGood = false;
	if (dataGood == false) {
		printf(
			"%sTried to set pin 1 as input, but the pin could still be written to.%s\n",
			RED, NO_COLOR);
		dataGood = true; // Reset for next step
		returnVal = false; // Return failure for test
	}

	// Set pin 0 high/low again, make sure it still works with pin 1 as input
	val.pin = 0;
	data = 1; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != 1) // Check pin data
		dataGood = false;
	data = 0; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != 0) // Check pin data
		dataGood = false;
	if (dataGood == false) {
		printf(
			"%sAfter trying to set pin 1 was set as input, pin 0 could not output data.%s\n",
			RED, NO_COLOR);
		dataGood = true; // Reset for next step
		returnVal = false; // Return failure for test
	}
	if (returnVal == true) { // Above test was errorVal
		printf("%sPin modes assigned correctly.%s\n", GREEN, NO_COLOR);
	} else {
		printf("%sFAILED: Pin mode assignment is non functional.%s\n", RED,
		       NO_COLOR);
	}
	// Set all pins as outputs for next test.
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		intf->setPinMode(pin, MODE_OUTPUT);
	}
	dev->updateData();

	// Sets one pin per cycle to low (first) then high over 32 total cycles
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times in the test
	for (int highLow = 0; highLow < 2; highLow++) {
		printf("Incrementing %s Test\n", (highLow ? "ON" : "OFF"));
		printf("Results:\n========\n");
		for (int cycle = 0; cycle < intf->getPinCount(); cycle++) { // Run 16 times
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
				val.pin = pin;
				data = ((pin == cycle) == highLow); // Set one pin per cycle to HIGH
				intf->writePin(&val);
			}

			dev->updateData(); // Push data out to the chip
			printf("\tCycle %d Pins:\t|", cycle);
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
				val.pin = pin;
				intf->readPin(&val); // Get actual pin data
				correctData = ((pin == cycle) == highLow); // Get the intended data
				if (data == correctData) // Check pin data
					dataGood = true;
				else {
					dataGood = false;
					returnVal = false;
				}
				printf("%s%s", (dataGood ? GREEN : RED), (data ? "'" : "."));
			}
			printf("%s|\n", NO_COLOR);
		}
	}

	// Sets pins to 0101 pattern, then 1010
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times in the test
	printf("Patterned Test\n");
	printf("Results:\n========\n");
	for (int highLow = 0; highLow < 2; highLow++) {
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
			val.pin = pin;
			data = ((pin % 2) == highLow); // Set one pin per cycle to HIGH
			intf->writePin(&val);
		}
		dev->updateData(); // Push data out to the chip
		printf("\tPins:\t\t|");
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
			val.pin = pin;
			intf->readPin(&val); // Get actual pin data
			correctData = ((pin % 2) == highLow); // Get the intended data
			if (data == correctData) // Check pin data
				dataGood = true;
			else {
				dataGood = false;
				returnVal = false;
			}
			printf("%s%s", (dataGood ? GREEN : RED), (data ? "'" : "."));
		}
		printf("%s|\n", NO_COLOR);
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	// Reset all pins
	data = 0;
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times
	for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
		val.pin = pin;
		intf->writePin(&val);
	}
	// Set all pins as inputs
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		intf->setPinMode(pin, MODE_INPUT);
	}
	dev->updateData();

	return returnVal;
} // testGpio

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPower(Interface *intf, Device *dev) {
	bool returnVal = true; // Set to false on error

	// InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For pin values
	// DataError_t errorVal; // For storing error codes from setting values and configs

	float data;
	PinMode_t pinMode;
	float correctData;
	bool dataGood;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf("POWER test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
	       WHITE, intf->getInterfaceIndex(), NO_COLOR,
	       WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! Interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	// Tests pin modes
	// ***************************************************************************
	printf("Pin Modes Test\n");
	printf("Results:\n========\n");
	printf("\tPin modes:\t|");
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		pinMode = intf->getPinMode(pin);
		if (pinMode == MODE_OUTPUT)
			dataGood = true;
		else {
			dataGood = false;
			returnVal = false;
		}
		printf("%s%s", dataGood ? GREEN : RED,
		       (pinMode == MODE_OUTPUT) ? "O" : "I" );
	}
	printf("%s|\n", NO_COLOR);

	// Sets one pin per cycle to low, then one per cycle high.
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times
	for (int highLow = 0; highLow < 2; highLow++) {
		printf("Incrementing %s Test\n", (highLow ? "ON" : "OFF"));
		printf("Results:\n========\n");
		for (int cycle = 0; cycle < intf->getPinCount(); cycle++) { // Run 16 times
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
				val.pin = pin;
				data = ((pin == cycle) == highLow); // Set one pin per cycle to HIGH
				intf->writePin(&val);
			}

			dev->updateData(); // Push data out to the chip
			printf("\tCycle %d Pins:\t|", cycle);
			for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
				val.pin = pin;
				intf->readPin(&val); // Get actual pin data
				correctData = ((pin == cycle) == highLow); // Get the intended data
				if (data == correctData) // Check pin data
					dataGood = true;
				else {
					dataGood = false;
					returnVal = false;
				}
				printf("%s%s", (dataGood ? GREEN : RED),
				       (data ? "'" : "."));
			}
			printf("%s|\n", NO_COLOR);
			// data = dev->getPinValue(0, VALUE_PWM_DUTY_100);
			// // printf("pin data: ");
		}
	}

	// Sets pins to 0101 pattern, then 1010
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times
	printf("Patterned Test\n");
	printf("Results:\n========\n");
	for (int highLow = 0; highLow < 2; highLow++) {
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
			val.pin = pin;
			data = ((pin % 2) == highLow); // Set one pin per cycle to HIGH
			intf->writePin(&val);
		}
		dev->updateData(); // Push data out to the chip
		printf("\tPins:\t\t|");
		for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
			val.pin = pin;
			intf->readPin(&val); // Get actual pin data
			correctData = ((pin % 2) == highLow); // Get the intended data
			if (data == correctData) // Check pin data
				dataGood = true;
			else {
				dataGood = false;
				returnVal = false;
			}
			printf("%s%s", (dataGood ? GREEN : RED),
			       (data ? "'" : "."));
		}
		printf("%s|\n", NO_COLOR);
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	// Reset all pins
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times
	for (int pin = 0; pin < intf->getPinCount(); pin++) { // Assign pins
		val.pin = pin;
		data = 0;
		intf->writePin(&val);
	}
	dev->updateData();

	return returnVal;
	// }
} // testPower

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testLeak(Interface *intf, Device *dev) {
	bool returnVal = true; // Set to false on error

	// InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For pin values
	// DataError_t errorVal; // For storing error codes from setting values and configs

	float data;
	float correctData;
	bool dataGood;
	bool hasLeak;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf("LEAK test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
	       WHITE, intf->getInterfaceIndex(), NO_COLOR,
	       WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! Interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	// Tests pin modes (need all to be input)
	// ***************************************************************************
	printf("Pin Modes Test\n");
	printf("Results:\n========\n");
	printf("\tPin modes:\t|");
	for (int pin = 0; pin < intf->getPinCount(); pin++) {
		// hd = intf->getHardwareDescriptor(pin);
		data = intf->getPinMode(pin);
		if (data == MODE_INPUT)
			dataGood = true;
		else {
			dataGood = false;
			returnVal = false;
		}
		printf("%s%s", dataGood ? GREEN : RED, (data == MODE_OUTPUT) ? "O" : "I" );
	}
	printf("%s|\n", NO_COLOR);

	// Make sure pin modes can't be altred by the interface
	// hd = intf->getHardwareDescriptor(0);
	intf->setPinMode(0, MODE_OUTPUT);
	dev->updateData();
	if (intf->getPinMode(0) != MODE_INPUT) {
		printf(
			"%sLEAK interface's pin modes was changed from input, which should not be possible.%s\n",
			RED, NO_COLOR);
		intf->setPinMode(0, MODE_INPUT); // Fix the mode
		dev->updateData();
		returnVal = false; // Return failure for test
	}

	// Tests pin states (should all be LOW)
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.data = &data; // This will be read and written to multiple times
	dev->updateData(); // Read leak data
	printf("Leak check test\n");
	printf("Results:\n========\n");
	printf("\tPin states:\t|");
	for (int pin = 0; pin < intf->getPinCount(); pin++) { // Check pins
		val.pin = pin; // Set format
		intf->readPin(&val); // Get actual pin data
		if (data == 0) // Check pin data
			dataGood = true;
		else {
			dataGood = false;
			returnVal = false;
			hasLeak = true;
		}
		printf("%s%s", (dataGood ? GREEN : RED),
		       (data ? "X" : "."));
	}
	printf("%s|\n", NO_COLOR);
	if (returnVal) {
		printf("%sLeak interface test passed.%s\n", GREEN, NO_COLOR);
	} else {
		printf("%sFAILED: Leak interface test has not passed.%s\n", RED, NO_COLOR);
		if (hasLeak)
			ROS_ERROR("LEAK INTERFACE HAS DETECTED A LEAK!");
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
} // testLeak

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testEmergencyIo(Interface *intf, Device *dev) {
	bool returnVal = true;
	// uint64_t hd;
	float data;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf(
		"EMERG_IO test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
		WHITE, intf->getInterfaceIndex(), NO_COLOR,
		WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! Interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	printf("No test for Emergency IO yet. This is a placeholder.\n");

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
} // testEmergencyIo

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testLed(Interface *intf, Device *dev) {
	bool returnVal = true; // Set to false on error

	// InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For pin values
	// DataError_t errorVal; // For storing error codes from setting values and configs

	float data;

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf(
		"LEAK_LED test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
		WHITE, intf->getInterfaceIndex(), NO_COLOR,
		WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! Interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	// Tests pin mode (should be output)
	// ***************************************************************************
	printf("Pin mode test\n");
	printf("Results:\n========\n");
	// hd = intf->getHardwareDescriptor(0);
	intf->setPinMode(0, MODE_INPUT);
	dev->updateData();
	if (intf->getPinMode(0) != MODE_OUTPUT) {
		printf(
			"%sLEAK_LED interface's pin modes was changed from output, which should not be possible.%s\n",
			RED, NO_COLOR);
		intf->setPinMode(0, MODE_OUTPUT); // Fix the mode
		dev->updateData();
		returnVal = false; // Return failure for test
	} else printf("%sPin mode test complete.%s\n", GREEN, NO_COLOR);

	// Tests pin states
	// ***************************************************************************
	val.fmt = VALUE_GPIO_STATE; // Set format
	val.pin = 0; // Only one pin per LED interface
	val.data = &data; // This will be read and written to multiple times
	printf("LED power test\n");
	printf("Results:\n========\n");
	// On
	// hd = intf->getHardwareDescriptor(0);
	data = 1; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (!data) {
		printf("%sCould not turn on LED%s\n", RED, NO_COLOR);
		returnVal = false;
	} else {
		printf("%sLED turned on.%s\n", GREEN, NO_COLOR);
	}

	// Off
	data = 0; // Set one pin per cycle to HIGH
	intf->writePin(&val);
	dev->updateData();
	intf->readPin(&val);
	if (data != 0) {
		printf("%sCould not turn off LED%s\n", RED, NO_COLOR);
		returnVal = false;
	} else {
		printf("%sLED turned off.%s\n", GREEN, NO_COLOR);
	}

	if (returnVal) {
		printf("%sLED test complete.%s\n", GREEN, NO_COLOR);
	} else {
		ROS_ERROR("%sLED test failed.%s\n", RED, NO_COLOR);
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
} // testLeakLed

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testAdc(Interface *intf, Device *dev) {
	bool returnVal = true; // Set to false on error

	InterfaceConfig_t cfg; // For configuring interfaces
	// PinValue_t val; // For pin values
	// DataError_t errorVal; // For storing error codes from setting values and configs

	float data;
	float calibrationDataIn[2];
	float calibrationDataOut[2];

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
	printf(
		"ADC test beginning.\n\tInterface:\t%s%d%s\n\tDevice:\t\t%s%d%s\n",
		WHITE, intf->getInterfaceIndex(), NO_COLOR,
		WHITE, dev->getDeviceIndex(), NO_COLOR);
	if (intf->getParentDeviceIndex() != dev->getDeviceIndex()) {
		printf("%sFailed test! Interface does not match parent device.%s",
		       RED, NO_COLOR);
		return false;
	}

	// Tests pin mode (should be input)
	// ***************************************************************************
	printf("Pin mode test\n");
	printf("Results:\n========\n");
	// hd = intf->getHardwareDescriptor(0);
	intf->setPinMode(0, MODE_OUTPUT);
	dev->updateData();
	if (intf->getPinMode(0) != MODE_INPUT) {
		printf(
			"%sADC interface pin modes were changed from input, which should not be possible.%s\n",
			RED, NO_COLOR);
		intf->setPinMode(0, MODE_INPUT); // Fix the mode
		dev->updateData();
		returnVal = false; // Return failure for test
	} else printf("%sPin mode test complete.%s\n", GREEN, NO_COLOR);

	// Tests calibration data (should be input)
	// ***************************************************************************
	printf("Checking calibration...\n");
	printf("Results:\n========\n");
	// Get ADC calibration values
	cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
	cfg.data = calibrationDataIn; // This will be read and written to multiple times
	intf->readConfig(&cfg);
	if (calibrationDataIn[1] == 1) { // If the tolerance is 100% (not yet assigned)
		printf(
			"%sADC Calibration has NOT been assigned. Default values have been assigned.%s\n",
			RED, NO_COLOR);
		returnVal = false; // Return failure for test
		calibrationDataOut[0] = 1; // Set default calibration values
		calibrationDataOut[1] = 1; // Makes tolerance = voltage to show there is no calibration
		cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
		cfg.data = calibrationDataOut; // This will be read and written to multiple times
		intf->writeConfig(&cfg); // Fix the mode
	} else printf("%sCalibration offset ratio = %5.2f.%s\n",
		            GREEN, calibrationDataIn[0], NO_COLOR);

	if (returnVal) {
		printf("%sADC test complete.%s\n", GREEN, NO_COLOR);
	} else {
		ROS_ERROR("ADC test failed.\n");
	}

	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);

	return returnVal;
} // testLeakLed

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
		       RED, deviceIdToCharArray(devices[dev]->getDeviceTypeId()), NO_COLOR,
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
				       interfaceIdToCharArray(interfaces[intf]->getInterfaceTypeId()),
				       NO_COLOR);
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
				// Temporarily removed to ease transition away from pinBus modes and states
				// for (int pin = 0; pin < interfaces[intf]->getPinCount(); pin++) {
				// 	if (shrinkRepeatedPins && pin != 0) { // Finds if pins are identical
				// 		uint8_t firstPin = pin;
				// 		// While the current pin matches previous, increment pin # up
				// 		while (pin < interfaces[intf]->getPinCount() &&
				// 		       interfaces[intf]->getPinBus().getPinMode(pin) ==
				// 		       interfaces[intf]->getPinBus().getPinMode(pin - 1) &&
				// 		       interfaces[intf]->getPinBus().getPinState(pin) ==
				// 		       interfaces[intf]->getPinBus().getPinState(pin - 1)) {
				// 			pin++;
				// 		}
				// 		// If incrementing actually happened
				// 		if (pin == interfaces[intf]->getPinCount()) // Last pin?
				// 			morePinsLeft = false;
				// 		printf("%s%s%s%s%s%s─", moreDevfLeft ? "│" : " ", SP,
				// 		       moreIntfLeft ? "│" : " ",  SP, SP, morePinsLeft ? "├" : "└"); // Formatting
				// 		if (firstPin != pin && firstPin != pin - 1) {
				// 			pin--;
				// 			printf("%s[also %d ... %d]%s", D_GRAY,
				// 			       interfaces[intf]->getMappedDevPinfirstPin),
				// 			       interfaces[intf]->getMappedDevPinpin),
				// 			       NO_COLOR);
				// 			printf("\n"); // Formatting
				// 		} else { // Incrementing did NOT happen, so act normal.
				// 			if (firstPin == pin - 1) // Incrementing did happen, but only once.
				// 				pin--;
				// 			printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getMappedDevPinpin),
				// 			       NO_COLOR);
				// 			printf("%s, ", interfaces[intf]->getPinBus().
				// 			       getModeString(pin, true));
				// 			printf("%s ", interfaces[intf]->getPinBus().
				// 			       getStateString(pin, true));
				// 			// printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getMappedDevPinpin),
				// 			//        NO_COLOR);
				// 			// printf("Count: %d ", interfaces[intf]->getPinBus().getPinCount());
				// 			printf("\n"); // Formatting
				// 		}
				// 	} else {
				// 		if (pin == interfaces[intf]->getPinCount() - 1) // Last pin?
				// 			morePinsLeft = false;
				// 		printf("%s%s%s%s%s%s─", moreDevfLeft ? "│" : " ", SP,
				// 		       moreIntfLeft ? "│" : " ",  SP, SP, morePinsLeft ? "├" : "└"); // Formatting
				// 		printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getMappedDevPinpin),
				// 		       NO_COLOR);
				// 		printf("%s, ", interfaces[intf]->getPinBus().
				// 		       getModeString(pin, true));
				// 		printf("%s ", interfaces[intf]->getPinBus().
				// 		       getStateString(pin, true));
				// 		// printf("Pin %s%d%s:\t", WHITE, interfaces[intf]->getMappedDevPinpin),
				// 		//        NO_COLOR);
				// 		// printf("Count: %d ", interfaces[intf]->getPinBus().getPinCount());
				// 		printf("\n"); // Formatting
				// 	}
				// }
			}
		}
	}
	// List any devices ignored due to bad initializations
	for (uint8_t intf = 0; intf < TOTAL_INTERFACES; intf++) {
		if (!interfaces[intf]->ready()) {
			printf("%sInterface #%d not listed due to a bad initialization.%s\n",
			       RED, intf, NO_COLOR);
		}
	}
	printf("%s", YELLOW);
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("%s\n", NO_COLOR);
} // dumpConfigruation
} // namespace bit_testing
