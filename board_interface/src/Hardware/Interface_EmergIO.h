#ifndef INTERFACE_EMERG_IO_H
#define INTERFACE_EMERG_IO_H

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
 * Interface_EmergIO
 * @author Nicholas Steele
 */
class Interface_EmergIO : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 3; // number of pins
const static Interface_t interfaceTypeId = INTF_EMERGENCY_IO;
const static Device_t parentDeviceTypeId = DEVICE_GPIO;
// ----------------------------------------------------------------------------

public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline Interface_t getInterfaceTypeId(){
	return interfaceTypeId;
}  // getInterfaceTypeId

//
inline uint8_t getParentTypeId(){
	return parentDeviceTypeId;
}  // getParentTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
}  // getPinCount

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
}

;
#endif /* end of include guard: INTERFACE_EMERG_IO_H */
