#ifndef DEVICE_PWM_PCA9685_H
#define DEVICE_PWM_PCA9685_H
// Generic headers
#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
#include "Device.h"
// #include "Interface.h"
#include "Logger.h"

// TODO: make this a config for interfaces to get value at their start
const float MAX_PWM_TICKS = 4095;
const float MIN_PWM_TICKS = 0;
const float MAX_VALUE_PWM_FREQUENCY_VALUE = 3500;
const float MIN_VALUE_PWM_FREQUENCY_VALUE = 0; // TODO: check this

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
}  // getPinCount

//
inline Device_t getDeviceTypeId() {
	return deviceTypeId;
}  // getDeviceTypeId

//
inline PinMode_t getValidPinModes(uint8_t i){
	return validPinModes[i];
}  // getValidPinModes

//
inline uint8_t getValidPinModesCount(){
	return VALID_PIN_MODE_COUNT;
}  // getValidPinModesCount

//
inline Interface_t &getReservedPins(uint8_t i){
	return reservedPins[i];
}  // getReservedPins

//
inline char *getHardwareName(){
	return HARDWARE_NAME;
}  // getHardwareName

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool deviceInit();

public:

DataError_t getPinValue(PinValue_t *value);

/**
 * Note that all data handling (min/max vals) happens in interfaces
 * @param pin TODO
 * @param struct PinDataPin data which MUST contain the PWM on time ticks.
 * @param interfaceId TODO
 * @return TODO
 */

DataError_t setPinValue(PinValue_t *value);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

/**
 * Reads/writes any data related to the chip. This function will interface
 * directly with the chip's hardware interface, and is the ONLY way that any code
 * can read from or write to the chip at 'address'.
 * Note that the order this happens in is important.
 * @return if update succeeded
 */

bool updateData();
}

;

#endif /* ifndef DEVICE_PWM_PCA9685_H */
