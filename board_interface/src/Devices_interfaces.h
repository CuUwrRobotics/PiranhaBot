#ifndef DEVICES_INTERFACES
#define DEVICES_INTERFACES

// Devices
#include "Device.cpp"
#include "Hardware/Device_Gpio_Mcp23017.cpp"
#include "Hardware/Device_Pwm_Pca9685.cpp"
#include "Hardware/Device_Adc_Mcp3008.cpp"

// Interfaces for the devices
#include "Interface.cpp"
#include "Hardware/Interface_Power.cpp"
#include "Hardware/Interface_Leak.cpp"
#include "Hardware/Interface_EmergIO.cpp"
#include "Hardware/Interface_LeakLed.cpp"
#include "Hardware/Interface_Gpio.cpp"
#include "Hardware/Interface_Pwm.cpp"
#include "Hardware/Interface_Adc.cpp"
#include "Hardware/Interface_Current_Acs781.cpp"
#include "Hardware/Interface_Voltage_Refrence.cpp"
#include "Hardware/Interface_Temp_Lm62.cpp"
#include "Hardware/Interface_Power_Line.cpp"

#endif /* ifndef DEVICES_INTERFACES */
