// Generic headers
#include "HwHeader.h"
#include "Devices_interfaces.h"
#include "Hardware/Device_Adc_Mcp3008.h"

// Header file custom to this specific chip
// #include "Device_Gpio_Mcp23017.h"

class Device_Adc_Mcp3008 : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char HARDWARE_NAME[8] = "MCP3008";
// The name for the functionality of the device.
char HARDWARE_FUNCTION[4] = "ADC";
// What communication method is used for this device.
const static uint8_t COMM_TYPE = COMM_TYPE_SPI;

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 8;
const static uint8_t deviceTypeId = HardwareDescriptor::DEVICE_ADC;

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 1;
const PinMode validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT};

// Other Variables (Don't change these)
// ====================================
uint8_t reservedPins[PIN_COUNT];
// For storing tick rate (duty cycle)
float pinValues[PIN_COUNT];

// These are specific to the MCP3008 on the interfacing board REV A
const float ADC_STEPS = 1024;
const float AVCC_THEORETICAL_VALUE = 5.00;

/* These give the base Device class access to the above local variables. They
 * don't need any modification. See more info about each function in the Device
 * class.
 ******************************************************************************/

//
inline uint8_t getPinCount() {
	return PIN_COUNT;
} // getPinCount

//
inline uint8_t getCommType() {
	return COMM_TYPE;
} // getCommType

//
inline uint8_t getDeviceTypeId() {
	return deviceTypeId;
} // getDeviceTypeId

//
inline PinMode getValidPinModes(uint8_t i){
	return validPinModes[i];
} // getValidPinModes

//
inline uint8_t getValidPinModesCount(){
	return VALID_PIN_MODE_COUNT;
} // getValidPinModesCount

//
inline uint8_t &getReservedPins(uint8_t i){
	return reservedPins[i];
} // getReservedPins

//
inline char *getHardwareFunction(){
	return HARDWARE_FUNCTION;
} // getHardwareFunction

//
inline char *getHardwareName(){
	return HARDWARE_NAME;
} // getHardwareName

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool deviceInit(){
	// Init chip here

	// Default modes and states assigned here

	// Pin modes never change, and are not in update data function, so they are
	// set here and final
	for (uint8_t i = 0; i < getPinCount(); i++) {
		requestedPinBus.setPinMode(i, MODE_INPUT);
		currentPinBus.setPinMode(i, MODE_INPUT);
	}
	updateData();
	return true;
} /* deviceInit */

public:

/**
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

inline float getPinValue(uint8_t pin, DataType dataType){
	if (dataType == PACKET_ADC_DIRECT) // Data from a pin
		return pinValues[pin];
	if (dataType == PACKET_ADC_STEPS) // How many steps are in the ADC
		return ADC_STEPS;
	if (dataType == PACKET_ADC_AVCC_VOLTAGE) // Voltage of the ADC
		return AVCC_THEORETICAL_VALUE;
	ROS_ERROR("getPinValue for device index %d got bad dataType %d for pin %d.",
	          dataType, pin);
	return 0;
} // getPinValue

/**
 * Note that all data handling (min/max vals) happens in interfaces
 * @param pin TODO
 * @param struct PinDataPin data which MUST contain the PWM on time ticks.
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinValue(uint8_t pin, float *data, DataType dataType,
                 uint8_t interfaceId) {
	ROS_INFO("ADC index %d got set pin value, but cannot be written to.",
	         deviceIndex);
	return 0;
} // setPinValue

/**
 * Reads data
 */

bool updateData(){
	if (!ready())
		return false;
	// Check if any data is readable on any pins. If so, read it.
	printf("Reading data from pins (TODO).\n");
	// READ DATA HERE, THESE ARE FOR TESTING:
	for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
		pinValues[pin] = (0.625 * ADC_STEPS / AVCC_THEORETICAL_VALUE) * (pin + 1);
	}
	// Hacky way of getting correct telemetry data
	if (deviceIndex == 7) // CURRENT 0
		pinValues[0] = 389; // -10 amps
	if (deviceIndex == 7) // CURRENT 1
		pinValues[1] = 635; // +10 amps
	if (deviceIndex == 7) // VREF calibration, should be around 3V
		pinValues[2] = 614; // 3v (no adjustemnt)
	// pinValues[2] = 635;    // 3.1v (data is adjusted)
	if (deviceIndex == 7) // TEMP 0
		pinValues[3] = 178; // 25'C
	if (deviceIndex == 7) // PL 0
		pinValues[4] = 676; // 3.3v
	if (deviceIndex == 7) // PL 1
		pinValues[5] = 1024; // 5v
	if (deviceIndex == 7) // PL 2
		pinValues[6] = 768; // 12v ==> 3.75v with resitor divider
	if (deviceIndex == 7) // PL 3
		pinValues[7] = 819; // 48v ==> 4v with resitor divider
	return true;
} // updateData
}

;
