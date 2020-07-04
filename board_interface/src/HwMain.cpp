/* notes on how this will work:
 * Any device to be communicated with (eg, a GPIO chip) will be accessible via its class object. The
 * class object is initialized using the YAML config files. The sub-interfaces (interfaces which
 * aren't their own device, eg, PWR switching) will be defined using classes containing pointers to
 * the class given to their 'parent' interface. When new data is recieved, it is intrerpreted and
 * passed on to the parent class. To update data, each parent class is told to update. It will check
 * if data has been updated, and if it has, it will send the updated info over the communication
 * lines.
 *
 * Notes on this concept:
 * - All parent classes must have a communication method and address to communicate with (I2C addr
 * 			or SPI CS pin)
 * - All parent classes must track whether data was changed and update data as needed when called
 * 			from main program loop
 * - All parent classes must know the current and desired state of all their pins, so they can know
 * 			if an update is needed.
 * - Sub-classes must have a pointer to a parent class, and should assign values using them, while
 * 			also checking that the value is acceptable.
 * - Input data (eg, adc data) will always be chancked and output as a msg after that.
 *
 * An example psuedo-code following this logic, for a successful power switching output:
 * 	1. Request for Hardware Descriptor (HD) created to access power pin 3.
 * 		-> HD Request sent to the PWR class handler, which ensures that the PWR pin is not in use.
 * 			-> If ok, PWR class sends HD request to gpio, which also checks that pin is open to use.
 * 			-> HD Request is OK'd; return HD number from parent function.
 * 		-> Request was ok'd by parent function, return HD number to the node that made the request.
 * 	2. Control (CTRL) request sent to PWR pin 3, using correct HD number.
 *		-> PWR class checks its portion of HD, then relays to GPIO class to do the same
 *			-> HD is good:
 *				-> Add requested change to the new states array.
 *				-> Return that a request for change was set up (does not guarrentee that change will be
 *							made, which will fail if device is non-functional.)
 * 		-> After all requests processed, main loop moves on to sending data over logical lines
 */
// TODO: Convert all printf's to ROS_ERROR

#include "HwHeader.h"
#include "Devices_interfaces.h"
#include "PinBus.h"
#include "PinData.h"
#include "BitTesting.h"

#define DEVICE_ADDRESS 0x55

Interface *interfaces[TOTAL_INTERFACES];
Device *devices[TOTAL_DEVICES];

/**
 * Creates device objects, defining what type of device each object is.
 */

void createAndInitDevices(){
	uint8_t i = 0; // incremented to allow devices to keep track of their own index.

	// GPIO 0
	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	// GPIO 1
	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	// GPIO 2 (only accessible through otehr interfaces, like LEAK)
	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	// PWM 0
	devices[i] = new Device_Pwm_Pca9685();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_PWM);
	i++;

	// PWM 1
	devices[i] = new Device_Pwm_Pca9685();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_PWM);
	i++;

	// variable value must be changed if this is true
	if (i != TOTAL_DEVICES) {
		ROS_ERROR(
			"ERROR: TOTAL_DEVICES NOT SET CORRECTLY! DOUBLE CHECK DEVICE COUNT OR SETUP!\nSTOPPING.\n");
		while (true) {}
	}

	// ADD MORE DEVICES HERE
} // createAndInitDevices

/**
 * Creates interface objects, defining what type of interface each object is.
 */

void createAndInitInterfaces(){
	uint8_t i = 0; // Increment on each new interface
	uint8_t d = 0; // Increment on each new device
	PinBus pinBus; // Set on every new interface
	BusType busType = BUS_INVALID; // Set on every new device type

	// GPIO 0 (Device index 0)
	// ***************************************************************************
	busType = BUS_GPIO;
	// GPIO interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_INPUT);
	interfaces[i] = new Interface_Gpio();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// GPIO 1 (Device index 1)
	// ***************************************************************************
	busType = BUS_GPIO;
	d++;
	// GPIO interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_INPUT);
	interfaces[i] = new Interface_Gpio();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// GPIO 2 (device index 2)
	// ***************************************************************************
	busType = BUS_GPIO;
	d++;
	// LEAK interface: 8 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
	interfaces[i] = new Interface_Leak();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// PWR interface: 4 pins
	pinBus.createUniformPinBusFromSet(busType, 8, 11, MODE_OUTPUT);
	interfaces[i] = new Interface_Power();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// Emergeny I/O telemetry interface: 3 pins
	pinBus.createUniformPinBusFromSet(busType, 12, 14, MODE_OUTPUT);
	interfaces[i] = new Interface_EmergIO();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// Led which warns of leaks/other major issues. interface: 2 pin
	pinBus.createUniformPinBusFromSet(busType, 15, 15, MODE_OUTPUT);
	interfaces[i] = new Interface_LeakLed();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// PWM 0 (Device index 3)
	// ***************************************************************************
	busType = BUS_PWM;
	d++;
	// PWM interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_OUTPUT);
	interfaces[i] = new Interface_Pwm();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// PWM 1 (Device index 4)
	// ***************************************************************************
	busType = BUS_PWM;
	d++;
	// PWM interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_OUTPUT);
	interfaces[i] = new Interface_Pwm();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// variable value must be changed if this is true
	if (i != TOTAL_INTERFACES) {
		ROS_ERROR(
			"ERROR: TOTAL_INTERFACES NOT SET CORRECTLY! DOUBLE CHECK INTERACE COUNT OR SETUP!\nSTOPPING.\n");
		while (true) {}
	}
} // createAndInitInterfaces

/**
 *
 */

void runBitTest(){
	// run built in testing
	bool testsOk = true;
	// Run the tests andflag if one fails, since they dump a LOT of data.
	// PWM testing
	// ===========
	if (!testPwm(interfaces[6], devices[3], true)) testsOk = false;
	if (!testPwm(interfaces[6], devices[3], false)) testsOk = false;
	if (!testPwm(interfaces[7], devices[4], true)) testsOk = false;
	if (!testPwm(interfaces[7], devices[4], false)) testsOk = false;

	// GPIO testing
	// ===========
	if (!testGpio(interfaces[0], devices[0])) testsOk = false;
	if (!testGpio(interfaces[1], devices[1])) testsOk = false;
	// if (!testGpio(interfaces[7], devices[4])) testsOk = false;
	// if (!testGpio(interfaces[7], devices[4])) testsOk = false;
	if (!testsOk) {
		printf(
			"%sWARINING: AT LEAST ONE BIT TEST HAS FAILED. Read the data above to find the reason.%s\n",
			RED, NO_COLOR);
	}	else {
		printf("%sAll BIT tests good :)%s\n", GREEN, NO_COLOR);
	}
} // runBitTest

/**
 * @return TODO
 */

int main(){
	createAndInitDevices(); // Setup Devices
	createAndInitInterfaces(); // Setup Interfaces
	for (uint8_t i = 0; i < TOTAL_DEVICES; i++)
		devices[i]->updateData();
	// All set; dump data
	dumpConfiguration(true, interfaces, devices); // False for full pin listing
	runBitTest(); // Test interfaces
} // main
