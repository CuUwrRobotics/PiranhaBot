/* This class is a tempalte for copy/pasting to actual class files. It is
 * intened tospeed up making new devices.
 */
// Generic headers
#include "HwHeader.h"
#include "Devices_interfaces.h"
#include "PinData.h"

// Header file custom to this specific chip
#include "Device_Gpio_Mcp23017.h"

/**
 * Template device subclass for creating any new devices.
 * @author
 */
class Device_Gpio_Mcp23017 : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char HARDWARE_NAME[9] = "MCP23017";
// The name for the functionality of the device.
char HARDWARE_FUNCTION[5] = "GPIO";
// What communication method is used for this device.
const static uint8_t COMM_TYPE = COMM_TYPE_I2C;

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 16;
const static uint8_t deviceTypeId = HardwareDescriptor::DEVICE_GPIO;
const static uint8_t readableData = true; // If this chip can read data

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 2;
const PinMode validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT,
	                                                   MODE_OUTPUT};

// Other Variables (Don't change these)
// ====================================
uint8_t reservedPins[PIN_COUNT];
// No pin values for GPIO, only HIGH/LOW, so each bit is one pin.
uint16_t currentPinValues;
uint16_t requestedPinValues;

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

	pinModeChangePending = true;
	writeDataPending = true;
	updateData();
	return true;
} /* deviceInit */

public:

/**
 * @param pin TODO
 * @param dataType TODO
 * @return TODO
 */

inline uint16_t getPinValue(uint8_t pin, DataType dataType){
	if (dataType == PACKET_GPIO_STATE) {
		// printf("reading: 0x%4x", currentPinValues);
		return (currentPinValues >> pin) & 0x0001; // Only need 1 bit.
	}
	ROS_ERROR("getPinValue for device index %d got bad dataType %d for pin %d.",
	          dataType, pin);
	return 0;
} // getPinValue

/**
 * @param pin TODO
 * @param value TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinValue(uint8_t pin, uint16_t *data, DataType dataType, uint8_t
                 interfaceId) {
	if (getReservedPins(pin) != interfaceId) {
		ROS_ERROR(
			"setPinValue for device index %d got bad interface ID 0x%.2x for pin %d.",
			deviceIndex, interfaceId, pin);
		return false;
	}
	// If pin is not going to be in output mode, don't set it.
	if (pinIsReadable(pin)) {
		// Error is silenceable for BIT testing.
		ROS_INFO(
			"setPinValue for device index %d was told to write to a pin in a reading mode.",
			deviceIndex);
		return false;
	}
	if (pin >= 0 && pin < PIN_COUNT) {
		if (dataType == PACKET_GPIO_STATE) {
			if (data[0]) // set bit
				requestedPinValues |= (0x0001 << pin);
			else // clear bit
				requestedPinValues &= ~(0x0001 << pin);
			writeDataPending = true;
			return true;
		}
		ROS_ERROR("setPinValue for device index %d got bad dataType %d.",
		          deviceIndex, dataType);
		return false;
	}
	ROS_ERROR("setPinValue for device index %d got bad pin %d.",
	          deviceIndex, pin);
	return false;
} // setPinValue

/**
 * Reads/writes any data related to the chip. This function will interface
 * directly with the chip's hardware interface, and is the ONLY way that any code
 * can read from or write to the chip at 'address'.
 * Note that the order this happens in is important.
 * @return if update succeeded
 */

bool updateData(){
	if (!ready())
		return false;
	// Check if any pins need their modes changed
	if (pinModeChangePending) {
		printf("Changing pin modes (TODO).\n");
		// HACK: For now, just push data over
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
			currentPinBus.setPinMode(pin, requestedPinBus.getPinMode(pin));
		}
		// ASSIGN PIN MODES HERE
		pinModeChangePending = false;
	}
	// Check if any data is readable on any pins. If so, read it.
	if (readableDataAvailable()) {
		printf("Reading data from pins (TODO).\n");
		// READ DATA HERE
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
			if (pinIsReadable(pin)) { // If pin should be read
				// currentPinValues[pin] = 0; // Just set to zero
			}
		}
	}
	// Check if any data needs to be written. If so, write it.
	if (writeDataPending) {
		printf("Writing data to pins (TODO).\n");
		currentPinValues = requestedPinValues; // Just pushes data over for now
		// NOTE: this process depends highly on the device being written to.
		// First, set any pin states
		// for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
		// }
		// No values for this interface
		// uint16_t pinValuesToSend[PIN_COUNT] = {0};
		// // For each pin, check if pin is in read mode. if not, write the value over.
		// for (uint8_t pin = 0; pin < PIN_COUNT; pin++) { // For each pin
		// 	if (pinIsReadable(pin)) {
		// 		pinValuesToSend[pin] = 0; // Set data to zero for non-writable pins
		// 	} else {
		// 		pinValuesToSend[pin] = requestedPinValues[pin]; // Pin is set up for write, so
		// 		currentPinValues[pin] = requestedPinValues[pin];
		// 	}
		// }
		// WRITE DATA HERE
		writeDataPending = false;
	}
	return true;
} // updateData
}

;
