#ifndef DEVICE_ADC_MCP3008_H
#define DEVICE_ADC_MCP3008_H
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

// Total steps for the ADC = 2^(bitwidth)
const float ADC_STEPS = 1024;

// Theoretical AVCC voltage of 5v. For other devices, this may vary.
// Don't meaure and calibrate this value; calibration is handled in real time
// by onboard ADCs using voltage refrences. Just enter the theoretical,
// intended value.
const float AVCC_THEORETICAL_VALUE = 5.00;

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

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 8;
const static Device_t deviceTypeId = DEVICE_ADC;

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 1;
const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT};

// Other Variables (Don't change these)
// ====================================
Interface_t reservedPins[PIN_COUNT];
// For storing tick rate (duty cycle)
float pinValues[PIN_COUNT];

// These are specific to the MCP3008 on the interfacing board REV A
// const float ADC_STEPS = 1024;
// const float AVCC_THEORETICAL_VALUE = 5.00;

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

bool deviceInit();

public:

/**
 */

DataError_t getPinValue(PinValue_t *value);

/**
 * Can't set the pin value on an ADC
 */

DataError_t setPinValue(PinValue_t *value);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

/**
 * Reads data
 */

bool updateData();
}

;

#endif /* ifndef DEVICE_ADC_MCP3008_H */
