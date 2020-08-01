#ifndef DEVICES_INTERFACES_H
#define DEVICES_INTERFACES_H

// Devices
#include "Device.h"
#include "Hardware/Device_Gpio_Mcp23017.h"
#include "Hardware/Device_Pwm_Pca9685.h"
#include "Hardware/Device_Adc_Mcp3008.h"

// Interfaces for the devices
#include "Interface.h"
#include "Hardware/Interface_Power.h"
#include "Hardware/Interface_Leak.h"
#include "Hardware/Interface_EmergIO.h"
#include "Hardware/Interface_LeakLed.h"
#include "Hardware/Interface_Gpio.h"
#include "Hardware/Interface_Pwm.h"
#include "Hardware/Interface_Adc.h"
#include "Hardware/Interface_Current_Acs781.h"
#include "Hardware/Interface_Voltage_Refrence.h"
#include "Hardware/Interface_Temp_Lm62.h"
#include "Hardware/Interface_Voltage_Div.h"

#endif /* ifndef DEVICES_INTERFACES_H */
