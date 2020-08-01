#ifndef INTERFACE_VOLTAGE_REFRENCE_H
#define INTERFACE_VOLTAGE_REFRENCE_H

// Generic headers
#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
// #include "Device.h"
#include "Interface.h"
#include "Logger.h"

/**
 * Interface_Adc
 * @author
 */
class Interface_Voltage_Refrence : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 1;
// IDs which indicate what this is and what it should be connected to
const static Interface_t interfaceTypeId = INTF_VREF;
const static Device_t parentDeviceTypeId = DEVICE_ADC;

// For data conversions
float adcSteps = -99; // number of steps the ADC uses to save data (ie reading at AVCC)
float avccTheoretical = -99; // Theoretical AVCC
// Values for voltage offset ratio calcs
float knownDiodeVoltage = -99; // This should be measured for accuracy
float knownDiodeTolerance = -99; // 2%
float knownAdcTolerance = -99; // (5v/2^9) for a 10-bit ADC after removing LSB
float measuredDiodeVoltage = -99; // Measured voltage.
float offsetRatio = -99;
float toleranceRatio = -99; // Multiply by a voltage to get tolerance of estimate.

// How many times to measure the refrence valtage before calculating average
uint8_t measureCycles = 10;
float refMeasurementAverage = 0;

/**
 * @return If all data needed to run calcs, true.
 */

bool calculateValues();

public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline Interface_t getInterfaceTypeId(){
	return interfaceTypeId;
} // getInterfaceTypeId

//
inline uint8_t getParentTypeId(){
	return parentDeviceTypeId;
} // getParentTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void prepareInterface();

DataError_t readPin(PinValue_t *valueIn);

DataError_t writePin(PinValue_t *valueIn);

DataError_t writeConfig(InterfaceConfig_t *cfg);

DataError_t readConfig(InterfaceConfig_t *cfg);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

uint8_t setPinMode(uint8_t pinNumber, PinMode_t pinMode);
}

;

#endif /* end of include guard: INTERFACE_VOLTAGE_REFRENCE_H */
