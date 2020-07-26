// Generic headers
#include "HwHeader.h"
#include "AllDevicesInterfaces.h"

// Header file custom to this specific chip
// #include "Device_Gpio_Mcp23017.h"

class Device_Pwm_Pca9685 : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char HARDWARE_NAME[8] = "PCA9685";

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 16;
const static Device_t deviceTypeId = DEVICE_PWM;

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 1;
const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_OUTPUT};

// Other Variables (Don't change these)
// ====================================
Interface_t reservedPins[PIN_COUNT];
// For storing tick rate (duty cycle)
float currentPinTicks[PIN_COUNT];
float requestedPinTicks[PIN_COUNT];
// For storing device frequncy
float currentFrequencyValue;
float requestedFrequencyValue;

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

inline DataError_t getPinValue(PinValue_t *value){
	if (value->fmt == VALUE_PWM_FREQ) {
		value->data[0] = currentFrequencyValue;
		return ERROR_SUCCESS;
	}
	if (value->fmt == VALUE_PWM_ON_TICKS) {
		value->data[0] = currentPinTicks[value->pin];
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // getPinValue

/**
 * Note that all data handling (min/max vals) happens in interfaces
 * @param pin TODO
 * @param struct PinDataPin data which MUST contain the PWM on time ticks.
 * @param interfaceId TODO
 * @return TODO
 */

DataError_t setPinValue(PinValue_t *value) {
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;
	// Don't flag for a data write if no changes are made.
	if (value->fmt == VALUE_PWM_FREQ) {
		if (value->data[0] == currentFrequencyValue) {
			return ERROR_SUCCESS;
		}	else {
			requestedFrequencyValue = value->data[0];
			writeDataPending = true;
			return ERROR_SUCCESS;
		}
	} else if (value->fmt == VALUE_PWM_ON_TICKS) {
		if (value->data[0] == requestedPinTicks[value->pin]) {
			return ERROR_SUCCESS;
		}	else {
			requestedPinTicks[value->pin] = value->data[0];
			writeDataPending = true;
			return ERROR_SUCCESS;
		}
	}
	return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

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
		// printf("Reading data from pins (TODO).\n");
		// // READ DATA HERE
		// for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
		// 	if (pinIsReadable(pin)) { // If pin should be read
		// 		currentPinTicks[pin] = 0; // Just set to zero
		// 	}
		// }
	}
	// Check if any data needs to be written. If so, write it.
	if (writeDataPending) {
		printf("Writing data to pins (TODO). %d\n", requestedFrequencyValue);
		float pinValuesToSend[PIN_COUNT] = {0};
		// For each pin, write the value over.
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) { // For each pin
			// if (pinIsReadable(pin)) {
			// pinValuesToSend[pin] = 0; // Set data to zero for non-writable pins
			// } else {
			pinValuesToSend[pin] = requestedPinTicks[pin]; // Pin is set up for write, so
			currentPinTicks[pin] = requestedPinTicks[pin];
			currentFrequencyValue = requestedFrequencyValue;
			// }
		}
		// WRITE DATA HERE
		writeDataPending = false;
	}
	return true;
} // updateData
}

;
