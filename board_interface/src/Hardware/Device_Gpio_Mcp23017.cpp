/* This class is a tempalte for copy/pasting to actual class files. It is
 * intened tospeed up making new devices.
 */
// Generic headers
#include "HwHeader.h"
#include "AllDevicesInterfaces.h"

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

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 16;
const static Device_t deviceTypeId = DEVICE_GPIO;

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 2;
const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT,
	                                                     MODE_OUTPUT};

// Other Variables (Don't change these)
// ====================================
Interface_t reservedPins[PIN_COUNT];
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
inline Device_t getDeviceTypeId() {
	return deviceTypeId;
} // getDeviceTypeId

//
inline PinMode_t getValidPinModes(uint8_t i){
	return validPinModes[i];
} // getValidPinModes

//
inline uint8_t getValidPinModesCount(){
	return VALID_PIN_MODE_COUNT;
} // getValidPinModesCount

//
inline Interface_t &getReservedPins(uint8_t i){
	return reservedPins[i];
} // getReservedPins

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

DataError_t getPinValue(PinValue_t *value){
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;
	if (value->fmt == VALUE_GPIO_STATE) {
		// Gets the on/off bit and returns as a 1 or 0.
		// value->data[0] = ((uint16_t)((currentPinValues >> pin) & 0x01) !=
		//                   (uint16_t)0) ? 1 : 0;
		value->data[0] = ((currentPinValues >> value->pin) & 0x01);
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // getPinValue

DataError_t setPinValue(PinValue_t *value) {
	// If pin is not writable, don't set it.
	if (pinIsReadable(value->pin))
		return ERROR_WROTE_INPUT;
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;

	if (value->fmt == VALUE_GPIO_STATE) {
		if (value->data[0]) // set bit
			requestedPinValues |= (0x0001 << value->pin);
		else // clear bit
			requestedPinValues &= ~(0x0001 << value->pin);
		writeDataPending = true;
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

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
		// float pinValuesToSend[PIN_COUNT] = {0};
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
