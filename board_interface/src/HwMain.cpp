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
#include "Devices_interface.h"
#include "PinBus.h"

#define DEVICE_ADDRESS 0x55

const static uint8_t TOTAL_INTERFACES = 7;
const static uint8_t TOTAL_DEVICES = 4;

Interface *interfaces[TOTAL_INTERFACES];
Device *devices[TOTAL_DEVICES];

/**
 * Fancy display of all devices, interfaces, and pins.
 * TODO: At end, cycle through and see if any interfaces are unconnected.
 */

void dumpConfiguration(bool shrinkRepeatedPins){
	bool moreDevfLeft = true;
	bool moreIntfLeft = true;
	bool morePinsLeft = true;
	char D_GRAY[10] = "\033[1;30m";
	char RED[10] = "\033[1;31m";
	char SP[4] = "  "; // Spacing
	char YELLOW[10] = "\033[1;33m";
	char CYAN[10] = "\033[0;36m";
	char GREEN[10] = "\033[1;32m";
	char WHITE[10] = "\033[1;37m";
	char NO_COLOR[7] = "\033[0m";
	printf("\nCONFIGURATION DUMP:\n");
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("\n");
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
						 devices[dev]->getTypeId()), NO_COLOR,
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
								 interfaces[intf]->getTypeId()), NO_COLOR);
				printf("%s%s", moreDevfLeft ? "│" : " ", SP); // Formatting
				printf("%s%s", moreIntfLeft ? "│" : " ", SP); // Formatting
				printf("└─"); // Formatting
				if (!interfaces[intf]->ready()){
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
							printf("%s[pins #%d - %d are identical]%s", D_GRAY,
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
				// interfaces[intf]->printPins(); // prints pins to console
			}
		}
	}
	// List any devices ignored due to bad initializations
	for (uint8_t intf = 0; intf < TOTAL_INTERFACES; intf++) {
		if (interfaces[intf]->getParentDeviceIndex() ==
		    HardwareDescriptor::DEVICE_INVALID) {
			printf("%sInterface #%d not listed due to a bad initialization.\nMost likely that start() was never called.%s\n",
			       RED, intf, NO_COLOR);
		}
	}
	for (int i = 0; i < 80; i++)
		printf(":");
	printf("\n");
} // dumpConfigruation

/**
 * Creates device objects, defining what type of device each object is.
 */

void createAndInitDevices(){
	uint8_t i = 0; // incremented to allow devices to keep track of their own index.

	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	devices[i] = new Device_Gpio_Mcp23017();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_GPIO);
	i++;

	devices[i] = new Device_Pwm_Pca9685();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_PWM);
	i++;

	// variable value must be changed if this is true
	if (i != TOTAL_DEVICES) {
		printf(
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
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_GPIO_INPUT, STATE_NONE);
	interfaces[i] = new Interface_Gpio();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// GPIO 1 (Device index 1)
	// ***************************************************************************
	busType = BUS_GPIO;
	d++;
	// GPIO interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_GPIO_INPUT, STATE_NONE);
	interfaces[i] = new Interface_Gpio();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// GPIO 2 (device index 2)
	// ***************************************************************************
	busType = BUS_GPIO;
	d++;
	// LEAK interface: 8 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_GPIO_INPUT, STATE_NONE);
	interfaces[i] = new Interface_Leak();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// PWR interface: 4 pins
	pinBus.createUniformPinBusFromSet(busType, 8, 11, MODE_GPIO_OUTPUT,
	                                  STATE_OFF);
	interfaces[i] = new Interface_Power();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// Emergeny I/O telemetry interface: 3 pins
	pinBus.createUniformPinBusFromSet(busType, 12, 14, MODE_GPIO_OUTPUT,
	                                  STATE_OFF);
	interfaces[i] = new Interface_EmergIO();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// Led which warns of leaks/other major issues. interface: 2 pin
	pinBus.createUniformPinBusFromSet(busType, 15, 15, MODE_GPIO_OUTPUT,
	                                  STATE_OFF);
	interfaces[i] = new Interface_LeakLed();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// PWM 0 (Device index 3)
	// ***************************************************************************
	busType = BUS_PWM;
	d++;
	// GPIO interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_PWM_OFF, STATE_OFF);
	interfaces[i] = new Interface_Pwm();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// variable value must be changed if this is true
	if (i != TOTAL_INTERFACES) {
		printf(
			"ERROR: TOTAL_INTERFACES NOT SET CORRECTLY! DOUBLE CHECK INTERACE COUNT OR SETUP!\nSTOPPING.\n");
		while (true) {}
	}
} // createAndInitInterfaces

/**
 * @return TODO
 */

int main(){
	createAndInitDevices(); // Setup Devices
	createAndInitInterfaces(); // Setup Interfaces
	for (uint8_t i = 0; i < TOTAL_DEVICES; i++)
		devices[i]->updateData();
	dumpConfiguration(true); // All set; dump data. Enter false if you want ot see full pin lists
} // main
