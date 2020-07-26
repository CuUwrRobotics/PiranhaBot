/* HardwareDescription.h
 * Holds data types and stringizers for all interface and devcice types.
 */
#ifndef HARDWARE_DESCRIPTION_H
#define HARDWARE_DESCRIPTION_H
 #include "HwHeader.h"

// Interface names
enum Interface_t {INTF_INVALID,
	                INTF_PWM,
	                INTF_GPIO,
	                INTF_ADC,
	                INTF_PWR_SWITCHING,
	                INTF_ARDUINO,
	                INTF_TEMP,
	                INTF_PRESSURE,
	                INTF_CURRENT,
	                INTF_VOLTAGE_DIVIDER,
	                INTF_SWITCHES,
	                INTF_LEAK,
	                INTF_EMERGENCY_IO,
	                INTF_LED, // Single LED
	                INTF_ARDUINO_GPIO_PWM,
	                INTF_ARDUINO_ADC,
	                INTF_VREF};

/**
 * @return Char array describing the interface
 */

const static char *interfaceIdToCharArray(Interface_t intf){
	switch (intf) {
	case INTF_INVALID:
		return "INTF_INVALID";
		break;
	case INTF_PWM:
		return "INTF_PWM";
		break;
	case INTF_GPIO:
		return "INTF_GPIO";
		break;
	case INTF_ADC:
		return "INTF_ADC";
		break;
	case INTF_PWR_SWITCHING:
		return "INTF_PWR_SWITCHING";
		break;
	case INTF_ARDUINO:
		return "INTF_ARDUINO";
		break;
	case INTF_TEMP:
		return "INTF_TEMP";
		break;
	case INTF_PRESSURE:
		return "INTF_PRESSURE";
		break;
	case INTF_CURRENT:
		return "INTF_CURRENT";
		break;
	case INTF_VOLTAGE_DIVIDER:
		return "INTF_VOLTAGE_DIVIDER";
		break;
	case INTF_SWITCHES:
		return "INTF_SWITCHES";
		break;
	case INTF_LEAK:
		return "INTF_LEAK";
		break;
	case INTF_EMERGENCY_IO:
		return "INTF_EMERGENCY_IO";
		break;
	case INTF_LED:
		return "INTF_LED";
		break;
	case INTF_ARDUINO_GPIO_PWM:
		return "INTF_ARDUINO_GPIO_PWM";
		break;
	case INTF_ARDUINO_ADC:
		return "INTF_ARDUINO_ADC";
	case INTF_VREF:
		return "INTF_VREF";
		break;
	default:
		return "(bad Interface_t)";
	} // switch
} // interfaceIdToCharArray

const static char *interfaceName(Interface_t intf){
	switch (intf) {
	case INTF_PWM:
		return "PWM";
		break;
	case INTF_GPIO:
		return "GPIO";
		break;
	case INTF_ADC:
		return "ADC";
		break;
	case INTF_PWR_SWITCHING:
		return "PWR SWITCHING";
		break;
	case INTF_ARDUINO:
		return "ARDUINO";
		break;
	case INTF_TEMP:
		return "TEMP";
		break;
	case INTF_PRESSURE:
		return "PRESSURE";
		break;
	case INTF_CURRENT:
		return "CURRENT";
		break;
	case INTF_VOLTAGE_DIVIDER:
		return "VOLTAGE DIVIDER";
		break;
	case INTF_SWITCHES:
		return "SWITCHES";
		break;
	case INTF_LEAK:
		return "LEAK";
		break;
	case INTF_EMERGENCY_IO:
		return "EMERGENCY IO";
		break;
	case INTF_LED:
		return "LED";
		break;
	case INTF_ARDUINO_GPIO_PWM:
		return "ARDUINO PWM";
		break;
	case INTF_ARDUINO_ADC:
		return "ARDUINO ADC";
	case INTF_VREF:
		return "VREF";
		break;
	default:
		return "(bad Interface_t)";
	} // switch
} /* interfaceName */

// Device names
enum Device_t {DEVICE_INVALID,
	             DEVICE_PWM,
	             DEVICE_GPIO,
	             DEVICE_ADC,
	             DEVICE_ARDUINO,
	             DEVICE_PRESSURE};

/**
 * @return Char array describing the device.
 */

const static char *deviceIdToCharArray(Device_t dev){
	switch (dev) {
	case DEVICE_INVALID:
		return "DEVICE_INVALID";
		break;
	case DEVICE_PWM:
		return "DEVICE_PWM";
		break;
	case DEVICE_GPIO:
		return "DEVICE_GPIO";
		break;
	case DEVICE_ADC:
		return "DEVICE_ADC";
		break;
	case DEVICE_ARDUINO:
		return "DEVICE_ARDUINO";
		break;
	case DEVICE_PRESSURE:
		return "DEVICE_PRESSURE";
		break;
	default:
		return "(bad Device_t)";
	} // switch
} // deviceIdToCharArray

const static char *deviceHardwareFunction(Device_t dev) {
	switch (dev) {
	case DEVICE_PWM:
		return "PWM";
		break;
	case DEVICE_GPIO:
		return "GPIO";
		break;
	case DEVICE_ADC:
		return "ADC";
		break;
	case DEVICE_ARDUINO:
		return "ARDUINO";
		break;
	case DEVICE_PRESSURE:
		return "PRESSURE";
		break;
	default:
		return "(bad Device_t)";
	} // switch
} /* deviceHardwareFunction */

const static char *devicePartNumber(Device_t dev){
	switch (dev) {
	case DEVICE_PWM:
		return "PCA9685";
		break;
	case DEVICE_GPIO:
		return "MCP23017";
		break;
	case DEVICE_ADC:
		return "MCP3008";
		break;
	case DEVICE_ARDUINO:
		return "ATMEGA328";
		break;
	case DEVICE_PRESSURE:
		return "MS5637";
		break;
	default:
		return "(bad Device_t)";
	} // switch
} /* devicePartNumber */

#endif // ifndef HARDWARE_DESCRIPTION_H
