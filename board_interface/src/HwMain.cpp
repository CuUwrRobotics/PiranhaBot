/**
 * Functionality:
 * Any device to be communicated with (eg, a GPIO chip) will be accessible via its class object. The
 * class object is initialized using the YAML config files. The sub-interfaces (interfaces which
 * aren't their own device, eg, PWR switching) will be defined using classes containing pointers to
 * the class given to their 'parent' interface. When new data is recieved, it is intrerpreted and
 * passed on to the parent class. To update data, each parent class is told to update. It will check
 * if data has been updated, and if it has, it will send the updated info over the communication
 * lines.
 */

#include "HwHeader.h"

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

	// ADC 0
	devices[i] = new Device_Adc_Mcp3008();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_ADC);
	i++;

	// ADC 1
	devices[i] = new Device_Adc_Mcp3008();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_ADC);
	i++;

	// ADC 2 (only accessible through other interfaces.)
	devices[i] = new Device_Adc_Mcp3008();
	devices[i]->init(i, DEVICE_ADDRESS + i, BUS_ADC);
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
	BusType_t busType = BUS_INVALID; // Set on every new device type

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

	// ADC 0 (Device index 5)
	// ***************************************************************************
	busType = BUS_ADC;
	d++;
	// PWM interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
	interfaces[i] = new Interface_Adc();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// ADC 1 (Device index 6)
	// ***************************************************************************
	busType = BUS_ADC;
	d++;
	// PWM interface: 16 pins
	pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
	interfaces[i] = new Interface_Adc();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// ADC 2 (Device index 6)
	// ***************************************************************************
	busType = BUS_ADC;
	d++;
// CURRENT interface: 1 pin
	pinBus.createUniformPinBusFromSet(busType, 0, 0, MODE_INPUT);
	interfaces[i] = new Interface_Current_Acs781();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// CURRENT interface: 1 pin
	pinBus.createUniformPinBusFromSet(busType, 1, 1, MODE_INPUT);
	interfaces[i] = new Interface_Current_Acs781();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// VOLTAGE REFRENCE interface: 1 pin
	pinBus.createUniformPinBusFromSet(busType, 2, 2, MODE_INPUT);
	interfaces[i] = new Interface_Voltage_Refrence();
	interfaces[i]->start(devices[d], pinBus, i);
	vrefIndex = i; // Save the index of the voltage refrence object.
	pinBus.resetAll();
	i++;

	// TEMP interface: 1 pin
	pinBus.createUniformPinBusFromSet(busType, 3, 3, MODE_INPUT);
	interfaces[i] = new Interface_Temp_Lm62();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// POWER_LINE interface: 1 pin (3.3v)
	pinBus.createUniformPinBusFromSet(busType, 4, 4, MODE_INPUT);
	interfaces[i] = new Interface_Voltage_Div();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// POWER_LINE interface: 1 pin (5v)
	pinBus.createUniformPinBusFromSet(busType, 5, 5, MODE_INPUT);
	interfaces[i] = new Interface_Voltage_Div();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// POWER_LINE interface: 1 pin (12v [?])
	pinBus.createUniformPinBusFromSet(busType, 6, 6, MODE_INPUT);
	interfaces[i] = new Interface_Voltage_Div();
	interfaces[i]->start(devices[d], pinBus, i);
	pinBus.resetAll();
	i++;

	// POWER_LINE interface: 1 pin (VIN, 48V)
	pinBus.createUniformPinBusFromSet(busType, 7, 7, MODE_INPUT);
	interfaces[i] = new Interface_Voltage_Div();
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
	if (!bit_testing::testPwm(interfaces[6], devices[3], true)) testsOk = false;
	if (!bit_testing::testPwm(interfaces[6], devices[3], false)) testsOk = false;
	if (!bit_testing::testPwm(interfaces[7], devices[4], true)) testsOk = false;
	if (!bit_testing::testPwm(interfaces[7], devices[4], false)) testsOk = false;

	// GPIO testing
	// ===========
	if (!bit_testing::testGpio(interfaces[0], devices[0])) testsOk = false;
	if (!bit_testing::testGpio(interfaces[1], devices[1])) testsOk = false;

	// POWER testing
	// ===========
	if (!bit_testing::testPower(interfaces[3], devices[2])) testsOk = false;

	// LEAK testing
	// ===========
	if (!bit_testing::testLeak(interfaces[2], devices[2])) testsOk = false;

	// EMERG_IO testing
	// ================
	if (!bit_testing::testEmergencyIo(interfaces[4], devices[2])) testsOk = false;

	// LEAK_LED testing
	// ================
	if (!bit_testing::testLed(interfaces[5], devices[2])) testsOk = false;

	// ADC testing
	// ================
	if (!bit_testing::testAdc(interfaces[8], devices[5])) testsOk = false;
	if (!bit_testing::testAdc(interfaces[9], devices[6])) testsOk = false;

	// All tests done
	if (!testsOk) {
		printf(
			"%sWARINING: AT LEAST ONE BIT TEST HAS FAILED. Read the data above to find the reason.%s\n",
			RED, NO_COLOR);
	}	else {
		printf("%sAll BIT tests good :)%s\n", GREEN, NO_COLOR);
	}
} // runBitTest

/**
 *
 */

void calibrateAdc() {
	InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For reading VREF

	float diodeVoltage = 3.1; // Known voltage.
	float measuredDiodeVoltage;
	float refReady;

	float refResults[2];
	float offsetRatio;
	float toleranceRatio; // Multiply by a voltage to get tolerance of estimate.
	float toleranceAtAvcc;
	float avccActual;
	// uint64_t hd;
	// First, set the truth values.
	float diodeData[2] = {actualDiodeVoltage, actualDiodeTolerance};
	// hd = interfaces[vrefIndex]->getHardwareDescriptor(0);
	cfg.fmt = ICFG_REF_KNOWN_VOLTS_WITH_TOLERANCE; // Set format
	cfg.data = diodeData;
	interfaces[vrefIndex]->writeConfig(&cfg);
	cfg.fmt = ICFG_REF_ADC_TOLERANCE; // Set format
	cfg.data = &adcTolerance;
	interfaces[vrefIndex]->writeConfig(&cfg);
	cfg.fmt = ICFG_REF_READY; // Set format
	cfg.data = &refReady;
	interfaces[vrefIndex]->readConfig(&cfg);
	if (!refReady) {
		log_error("%sVREF Interface did not give ready, cannot calibrate!%s\n");
		return;
	}
	cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
	cfg.data = refResults;
	interfaces[vrefIndex]->readConfig(&cfg);
	offsetRatio = refResults[0];
	toleranceRatio = refResults[1];
	val.fmt = VALUE_REF_VOLTAGE_NO_CORRECT; // Set format
	val.pin = 0;
	val.data = &measuredDiodeVoltage;
	interfaces[vrefIndex]->readPin(&val);

	printf("\n%sCalibrate ADC: Got calibration values:%s\n", YELLOW,
	       NO_COLOR);
	printf("\tActual diode voltage\t  = %s%5.2f%sV\t±%s%.2f%sV\n",
	       WHITE, actualDiodeVoltage, NO_COLOR,
	       WHITE, actualDiodeTolerance, NO_COLOR);
	printf("\tMeasured diode voltage\t  = %s%5.2f%sV\t±%s%.2f%sV\n",
	       WHITE, measuredDiodeVoltage, NO_COLOR,
	       WHITE, adcTolerance, NO_COLOR);
	// offsetRatio = actualDiodeVoltage / diodeVoltage;
	avccActual = offsetRatio * avccTheoretical;
	// Gets a tolerance ratio; multiply by the voltage to get tolerance.
	// toleranceRatio = ((actualDiodeTolerance / actualDiodeVoltage) +
	//                   (adcTolerance / diodeVoltage));
	toleranceAtAvcc = toleranceRatio * avccActual;
	printf("\tOffset ratio          \t  = %s%5.2f%s\t±%s%4.2f%s%\n",
	       WHITE, offsetRatio, NO_COLOR,
	       WHITE, toleranceRatio * 100, NO_COLOR);
	printf("\t%sAt measured %s5.00%sV, actual = %s%5.2f%sV\t±%s%.2f%sV\n",
	       NO_COLOR, // Text color
	       WHITE, NO_COLOR, // '5.00' color
	       WHITE, avccActual, NO_COLOR,
	       WHITE, toleranceAtAvcc, NO_COLOR);
	// Pack data into any device which identifies as an ADC.
	printf("%sCalibrate ADC: Storing values:%s\n", YELLOW,
	       NO_COLOR);
	float data[2] = {offsetRatio, toleranceRatio};
	for (uint8_t intf = 0; intf < TOTAL_INTERFACES; intf++) {
		// For any interfaces which use an ADC device
		if (interfaces[intf]->getParentTypeId() == DEVICE_ADC) {
			cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
			cfg.data = data;
			interfaces[intf]->writeConfig(&cfg);
			printf("\tData has been stored in %s%s%s interface, index %s%d%s.\n",
			       WHITE, interfaces[intf]->getInterfaceName(), NO_COLOR,
			       WHITE, intf, NO_COLOR);
		}
	}
} // calibrateAdc

/**
 * TODO: YAML
 */

void setupPowerLineReaders() {
	InterfaceConfig_t cfg; // For configuring interfaces
	PinValue_t val; // For checking interface outputs

	uint8_t lowestVoltageDividerIndex = 14;
	uint8_t highestVoltageDividerIndex = 18;

	float data;

	// These are all chosen based on the interface board REV A.
	printf("\n%sPower Line Startup: assigning resistor divider values.%s\n",
	       YELLOW,
	       NO_COLOR);
	// Technically in Ohms, but the ratio is all that matters.
	// To reduce error, measure actual resistors once installed, enter values, and
	// use a tolerance of zero.
	float highResistorValuesAndTolerances[4][2] = {
		{0, 0},
		{0, 0},
		{2.2, 0.05},
		{11,  0.05}};
	float lowResistorValuesAndTolerances[4][2] = {
		{0, 0},
		{0, 0},
		{1, 0.05},
		{1, 0.05}};
	for (uint8_t i = lowestVoltageDividerIndex; i < highestVoltageDividerIndex;
	     i++) {
		cfg.fmt = ICFG_PL_HIGH_RESISTOR_WITH_TOLERANCE; // Set format
		cfg.data = highResistorValuesAndTolerances[i - lowestVoltageDividerIndex];
		interfaces[i]->writeConfig(&cfg);

		cfg.fmt = ICFG_PL_LOW_RESISTOR_WITH_TOLERANCE; // Set format
		cfg.data = lowResistorValuesAndTolerances[i - lowestVoltageDividerIndex];
		interfaces[i]->writeConfig(&cfg);

		val.fmt = VALUE_ADC_VOLTAGE_WITH_TOLERANCE; // Set format
		val.pin = 0;
		val.data = &data;
		interfaces[i]->readPin(&val);
		printf("\tPL interface %s%d%s:\t%s%5.2f%sV\n",
		       WHITE, i, NO_COLOR,
		       WHITE, data, NO_COLOR);
	}
} // setupPowerLineReaders

void updateAllDevices() {
	for (uint8_t i = 0; i < TOTAL_DEVICES; i++)
		devices[i]->updateData();
} // updateDevices

void startupConfig(){
	printf("CONFIGURING BOARD INTERFACE");
	updateAllDevices();

	calibrateAdc(); // Calibrate the ADCs based on the known real AVCC value.
	setupPowerLineReaders(); // Assign resitor values for the power line interfaces

	updateAllDevices();
} // startupConfig

void hardwareExit() {
	ros::shutdown();
} // hardwareExit

/**
 * @return TODO
 */

int main(int argc, char *argv[]){
	atexit(hardwareExit);
	createAndInitDevices(); // Setup Devices
	createAndInitInterfaces(); // Setup Interfaces
	// YAML CONFIG GOES HERE
	// All set; dump data
	bit_testing::dumpConfiguration(true, interfaces, devices); // False for full pin listing
	startupConfig();
	runBitTest(); // Test interfaces
// #undefine DUMP_CURRENT_READS
#ifdef DUMP_CURRENT_READS
	float *currentIn;
	printf("CURRENT DUMP: CURRENT 0\n");
	printf("\tPin %d:\t%s%.0f%s", 0, WHITE,
	       *interfaces[10]->readPin(0, VALUE_ADC_DIRECT), NO_COLOR);
	currentIn = interfaces[10]->readPin(0, VALUE_CURRENT_AMPS_WITH_TOLERANCE);
	printf("\t%s%.2f%sA\t±%s%.2f%sA\n",
	       WHITE, currentIn[0], NO_COLOR,
	       WHITE, currentIn[1], NO_COLOR);

	printf("CURRENT DUMP: CURRENT 1\n");
	printf("\tPin %d:\t%s%.0f%s", 0, WHITE,
	       *interfaces[11]->readPin(0, VALUE_ADC_DIRECT), NO_COLOR);
	currentIn = interfaces[11]->readPin(0, VALUE_CURRENT_AMPS_WITH_TOLERANCE);
	printf("\t%s%.2f%sA\t±%s%.2f%sA\n",
	       WHITE, currentIn[0], NO_COLOR,
	       WHITE, currentIn[1], NO_COLOR);

	printf("TEMP DUMP: TEMP 0\n");
	printf("\tPin %d:\t%s%.0f%s\t%s%.2f%sV", 0,
	       WHITE, *interfaces[13]->readPin(0, VALUE_ADC_DIRECT), NO_COLOR,
	       WHITE, *interfaces[13]->readPin(0, VALUE_ADC_VOLTAGE), NO_COLOR);
	currentIn = interfaces[13]->readPin(0, VALUE_TEMP_C_WITH_TOLERANCE);
	printf("\t%s%5.2f%s'C\t±%s%.2f%s'C\n",
	       WHITE, currentIn[0], NO_COLOR,
	       WHITE, currentIn[1], NO_COLOR);

	printf("POWER LINE DUMP:\n");
	for (uint8_t i = 14; i < 18; i++) {
		currentIn = interfaces[i]->readPin(0, VALUE_ADC_VOLTAGE_WITH_TOLERANCE);
		printf("\tPL #%s%d%s:\t%s%5.2f%sV\t±%s%5.2f%sV\n",
		       WHITE, i, NO_COLOR,
		       WHITE,
		       currentIn[0],
		       NO_COLOR,
		       WHITE,
		       currentIn[1],
		       NO_COLOR);
	}

	printf("ADC DUMP: ADC 0\n");
	float *voltagesIn;
	// float *data;
	for (int pin = 0; pin < interfaces[8]->getPinCount(); pin++) {
		printf("\tPin %d:\t%s%.0f%s", pin, WHITE,
		       *interfaces[8]->readPin(pin, VALUE_ADC_DIRECT), NO_COLOR);
		voltagesIn = interfaces[8]->readPin(pin,
		                                    VALUE_ADC_VOLTAGE_WITH_TOLERANCE);
		printf("\t%s%.2f%sV\t±%s%.2f%sV\n",
		       WHITE, voltagesIn[0], NO_COLOR,
		       WHITE, voltagesIn[1], NO_COLOR);
	}
	printf("\n");
#endif // ifdef DUMP_CURRENT_READS
	// Connect ROS
	printf("Starting up ROS.\n");
	// Start ROS and get the node instance
	ros::init(argc, argv, "board_interface");
	ros::NodeHandle nd;
	// Get the node name ot send to the watchdog
	std::string nodeName = ros::this_node::getName();
	printf("Node name: %s\n", nodeName.c_str());
	// Set up the message publisher
	ros::Publisher wd_petter =
		nd.advertise <watchdog::pet_dog_msg> ("pet_dog_msg", 1000);
	// Allows for a 1 second delay between messages
	ros::Duration loop_wait(1);
	// For storing pets
	watchdog::pet_dog_msg msg;
	msg.petterName = nodeName; // Pack data
	while (ros::ok()) {
		// Pet the dog
		wd_petter.publish(msg);
		// Wait 1 second
		loop_wait.sleep();
	}
} // main
