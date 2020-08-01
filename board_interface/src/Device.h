#ifndef DEVICE_H
#define DEVICE_H

// Generic headers
#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "HardwareData.h"
#include "PinBus.h"
#include "Logger.h"

/**
 * Device
 * @author
 */
class Device {
private:

/**
 * @param addr TODO
 * @return TODO
 */

virtual bool deviceInit() = 0;

/**
 * Used to give base class access to local variables.
 */

virtual uint8_t getPinCount()  = 0;

/**
 * Used to give base class access to local variables.
 * @return The valid pin modes array declared at top of file.
 */

virtual PinMode_t getValidPinModes(uint8_t i) = 0;

/**
 * Used to give base class access to local variables.
 * @return Size of the valid pin modes array, which cannot be extrapolated from
 * the pointer returned by getValidPinModes()
 */

virtual uint8_t getValidPinModesCount() = 0;

/**
 * Used to give base class access to local variables.
 * @return reservedPins array of size getPinCount()
 */

virtual Interface_t &getReservedPins(uint8_t i) = 0;

/**
 * Used to give base class access to local variables.
 * @return String of the name of the chip used
 */

virtual char *getHardwareName() = 0;

public:

PinBus currentPinBus; // Current state of all pins
PinBus requestedPinBus; // State of pins not yet set
bool writeDataPending = false;
bool pinModeChangePending = false;
uint8_t address; // Address for communications
bool deviceIsSetup = false;

uint8_t deviceIndex; // Index of the device object

/**
 * Used to give base class access to local variables.
 * @return TODO
 */

virtual Device_t getDeviceTypeId() = 0;

/**
 * Used to get the value of a pin. If pin is readable, value will be the last read data.
 * @return DataError_t error usable with errorCharArray.
 */

virtual DataError_t getPinValue(PinValue_t *value) = 0;

/**
 * Used to set the value of a pin
 * If pin is readable, this will fail.
 * @return DataError_t error usable with errorCharArray.
 */

virtual DataError_t setPinValue(PinValue_t *value) = 0;

/**
 * Used to write a configuration for the device object
 * @return DataError_t error usable with errorCharArray.
 */

virtual DataError_t writeDeviceConfig(DeviceConfig_t *cfg) = 0;

/**
 * Used to get a configuration from the device object
 * @return DataError_t error usable with errorCharArray.
 */

virtual DataError_t readDeviceConfig(DeviceConfig_t *cfg) = 0;

/**
 * @return If given mode is valied for this device.
 */
bool pinModeIsValid(PinMode_t mode); // pinModeIsValid

/**
 * Init the variables and device for this object.
 * WARINGING: Do NOT overwrite this in subclasses! Instead, use the provided
 * deviceInit, which will be called by this method before finishing.
 * @return Whether init was errorVal
 */

// virtual bool init(uint8_t index, uint8_t addr, BusType_t busType) = 0; /* getTypeId */
bool init(uint8_t index, uint8_t addr, BusType_t busType); /* init */

/**
 * Hands control to pins over to an interface. Once control is granted, nothing
 * else can control these pins.
 * @param pinNumbers An array of pin numbers to assigne to the interface.
 * @return If handover was errorVal
 */

bool attachInterface(PinBus pinBus, Interface_t interfaceId); // attachInterface

/**
 * @return This device's index.
 */

uint8_t getDeviceIndex();

/* Updates any data from the device. If this device is readable, this will always check any pins
 * assigned as inputs. If it is not, if there is new data to be written, that data will be sent to
 * the device.
 * @return True if reads and writes were errorVal or unneccesary.
 */
virtual bool updateData() = 0;

/**
 * @param pinNumbers TODO
 * @param pinModes TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinModes(PinBus pinBus);

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param interfaceId TODO
 * @return TODO
 */

bool setPinMode(uint8_t pinNumber, PinMode_t pinMode);

/**
 * @return TODO
 */

bool verifyPins(PinBus pinBus);

/**
 * @return TODO
 */

bool readableDataAvailable();

/**
 * @return TODO
 */

inline bool pinIsReadable(uint8_t pin) {
	if (currentPinBus.getPinMode(pin) == MODE_INPUT) // If the pin is set to input
		return true; // Then there is some readable data
	return false; // There was no readable data
} // readableDataAvailable

/**
 * @return TODO
 */

inline PinMode_t getPinMode(uint8_t pin){
	return currentPinBus.getPinMode(pin);
} // getPinModes

/**
 * @param pin TODO
 * @return TODO
 */

Interface_t getPinInterface(uint8_t pin);

/*
 * @return
 */
PinBus getPinBus();

/**
 * @return If the device has been initialized yet.
 */

inline bool ready(){
	return deviceIsSetup;
} /* ready */
}

;

#endif /* end of include guard: DEVICE_H */
