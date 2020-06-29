#ifndef DEVICES_INTERFACES
#define DEVICES_INTERFACES

// Devices
#include "Device.cpp"
#include "HardwareDevices/Device_Gpio_Mcp23017.cpp"
#include "HardwareDevices/Device_Pwm_Pca9685.cpp"

// Interfaces for the devices
#include "Interface.cpp"
#include "HardwareDevices/Interface_Power.cpp"
#include "HardwareDevices/Interface_Leak.cpp"
#include "HardwareDevices/Interface_EmergIO.cpp"
#include "HardwareDevices/Interface_LeakLed.cpp"
#include "HardwareDevices/Interface_Gpio.cpp"
#include "HardwareDevices/Interface_Pwm.cpp"

#endif /* ifndef DEVICES_INTERFACES */
