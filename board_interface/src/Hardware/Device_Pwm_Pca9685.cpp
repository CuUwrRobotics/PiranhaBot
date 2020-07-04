/* This class is a tempalte for copy/pasting to actual class files. It is
 * intened tospeed up making new devices.
 */
// Generic headers
#include "HwHeader.h"
#include "Devices_interfaces.h"
#include "PinData.h"

// Header file custom to this specific chip
// #include "Device_Gpio_Mcp23017.h"

/**
 * Template device subclass for creating any new devices.
 * @author
 */
class Device_Pwm_Pca9685 : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char HARDWARE_NAME[8] = "PCA9685";
// The name for the functionality of the device.
char HARDWARE_FUNCTION[4] = "PWM";
// What communication method is used for this device.
const static uint8_t COMM_TYPE = COMM_TYPE_I2C;

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 16;
const static uint8_t deviceTypeId = HardwareDescriptor::DEVICE_PWM;
const static uint8_t readableData = false; // If this chip can read data

// Pin States and Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 2;
const PinMode validPinModes[VALID_PIN_MODE_COUNT] = {MODE_PWM_OFF,
	                                                   MODE_PWM_ON};
// Pin modes that this pin can read on. If none, use MODE_INVALID.
const static uint8_t VALID_READ_MODE_COUNT = 1;
const PinMode validReadModes[VALID_READ_MODE_COUNT] = {MODE_INVALID};
const static uint8_t VALID_PIN_STATE_COUNT = 3;
const PinState validPinStates[VALID_PIN_STATE_COUNT] = {STATE_ON,
	                                                      STATE_OFF};

// Other Variables (Don't change these)
// ====================================
uint8_t reservedPins[PIN_COUNT];
uint16_t currentPinValues[PIN_COUNT];
uint16_t requestedPinValues[PIN_COUNT];

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
inline PinMode getValidReadModes(uint8_t i){
	return validReadModes[i];
} // getValidReadModes

//
inline uint8_t getValidReadModesCount(){
	return VALID_READ_MODE_COUNT;
} // getValidReadModesCount

//
inline PinState getValidPinStates(uint8_t i){
	return validPinStates[i];
} // getValidPinStates

//
inline uint8_t getValidPinStatesCount(){
	return VALID_PIN_STATE_COUNT;
} // getValidPinStatesCount

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

// Simple retuning function which needs no modification.
inline uint16_t getPinValue(uint8_t pin, DataType dataType){
	return currentPinValues[pin];
} // getPinValue

/**
 * @param pin TODO
 * @param struct PinDataPin data which MUST contain the PWM on time ticks.
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinValue(uint8_t pin, uint16_t *data, DataType dataType,
                 uint8_t interfaceId) {
	if (getReservedPins(pin) != interfaceId) {
		ROS_ERROR(
			"setPinValue for device index %d got bad interface ID 0x%.2x for pin %d.",
			deviceIndex, interfaceId, pin);
		return false;
	}
	if (pin >= 0 && pin < PIN_COUNT) {
		requestedPinValues[pin] = data[0];
		writeDataPending = true;
		return true;
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
				currentPinValues[pin] = 0; // Just set to zero
			}
		}
	}
	// Check if any data needs to be written. If so, write it.
	if (writeDataPending) {
		printf("Writing data to pins (TODO).\n");
		// NOTE: this process depends highly on the device being written to.
		// First, set any pin states
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
			currentPinBus.setPinState(pin, requestedPinBus.getPinState(pin));
		}
		uint16_t pinValuesToSend[PIN_COUNT] = {0};
		// For each pin, check if pin is in read mode. if not, write the value over.
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) { // For each pin
			if (pinIsReadable(pin)) {
				pinValuesToSend[pin] = 0; // Set data to zero for non-writable pins
			} else {
				pinValuesToSend[pin] = requestedPinValues[pin]; // Pin is set up for write, so
				currentPinValues[pin] = requestedPinValues[pin];
			}
		}
		// WRITE DATA HERE
		writeDataPending = false;
	}
	return true;
} // updateData
}

;
