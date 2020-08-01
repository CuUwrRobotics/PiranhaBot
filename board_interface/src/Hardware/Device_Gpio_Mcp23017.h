// Header file exclusive to the the MCP23017 GPIO controller and it's communications.
#ifndef MCP23017_GPIODEVICE
#define MCP23017_GPIODEVICE

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

// GPIO specific pin states
const uint8_t MCP23017_PIN_ON = 0x01;
const uint8_t MCP23017_PIN_OFF = 0x00;
// TODO: fix these modes
// GPIO specific pin modes
const uint8_t MCP23017_PIN_INPUT = 0x10;
const uint8_t MCP23017_PIN_INPUT_HIGH_X = 0x11;
const uint8_t MCP23017_PIN_OUTPUT = 0x12;

// Defenitions, from Adafruit libarary:
#define MCP23017_DEFAULT_ADDRESS 0x20 // !< MCP23017 Address

// registers
#define MCP23017_IODIRA 0x00 // !< I/O direction register A
#define MCP23017_IPOLA 0x02 // !< Input polarity port register A
#define MCP23017_GPINTENA 0x04 // !< Interrupt-on-change pins A
#define MCP23017_DEFVALA 0x06 // !< Default value register A
#define MCP23017_INTCONA 0x08 // !< Interrupt-on-change control register A
#define MCP23017_IOCONA 0x0A // !< I/O expander configuration register A
#define MCP23017_GPPUA 0x0C // !< GPIO pull-up resistor register A
#define MCP23017_INTFA 0x0E // !< Interrupt flag register A
#define MCP23017_INTCAPA 0x10 // !< Interrupt captured value for port register A
#define MCP23017_GPIOA 0x12 // !< General purpose I/O port register A
#define MCP23017_OLATA 0x14 // !< Output latch register 0 A

#define MCP23017_IODIRB 0x01 // !< I/O direction register B
#define MCP23017_IPOLB 0x03 // !< Input polarity port register B
#define MCP23017_GPINTENB 0x05 // !< Interrupt-on-change pins B
#define MCP23017_DEFVALB 0x07 // !< Default value register B
#define MCP23017_INTCONB 0x09 // !< Interrupt-on-change control register B
#define MCP23017_IOCONB 0x0B // !< I/O expander configuration register B
#define MCP23017_GPPUB 0x0D // !< GPIO pull-up resistor register B
#define MCP23017_INTFB 0x0F // !< Interrupt flag register B
#define MCP23017_INTCAPB 0x11 // !< Interrupt captured value for port register B
#define MCP23017_GPIOB 0x13 // !< General purpose I/O port register B
#define MCP23017_OLATB 0x15 // !< Output latch register 0 B

#define MCP23017_INT_ERR 255 // !< Interrupt error

/**
 * Template device subclass for creating any new devices.
 * @author
 */
class Device_Gpio_Mcp23017:
public Device {
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

	bool deviceInit();

public:

	DataError_t getPinValue(PinValue_t *value);

	DataError_t setPinValue(PinValue_t *value);

	DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

	DataError_t readDeviceConfig(DeviceConfig_t *cfg);

	bool updateData();
}

;

#endif /* ifndef MCP23017_GPIODEVICE */
