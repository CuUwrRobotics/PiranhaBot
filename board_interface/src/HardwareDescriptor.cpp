/* HardwareDescriptor.c
 * Handles Harware Descriptors that are used to give out access to interfaces.
 * Hardware descriptor is an identifier which describes the device being connected to. The intent is
 * that another peice of software will request an HD, then after recieving one, can write to an
 * interface. Note that pin numbers and interface indexes start from 0 once they are no longer in
 * the HD.
 * WARNING: DO NOT PREDEFINE HARDWARE DESCRIPTORS ANYWHERE ELSE IN CODE! Use the given IDs and
 * functions.
 *
 * This is the outline for the 8-byte uint64_t which makes up the hardware descriptor:
 * WORDS 15, 14:			Interface
 * WORDS 13, 12:			Digital 'Parent' Device to communicate with over digital lines.
 * WORDS 11, 10:			Interface index.
 * WORDS 9, 8:				Digital Device index.
 * WORDS 7, 6:				Pin number of interface.
 * WORDS 5 - 0:				UNUSED (reserved for future development).
 */
#ifndef HARDWARE_DESCRIPTOR
#define HARDWARE_DESCRIPTOR
 #include "HwHeader.h"
// #include <stdint.h>

/**
 * HardwareDescriptor
 * @author
 */
class HardwareDescriptor {
private:
// Masks for getting data from the HD
// ****************************************************************************
const static uint64_t INTERFACE_TYPE_MASK						= 0xFF00000000000000;
const static uint64_t DIGITAL_DEVICE_MASK						= 0x00FF000000000000;
const static uint64_t INTERFACE_INDEX_MASK					= 0x0000FF0000000000;
const static uint64_t DIGITAL_DEVICE_INDEX_MASK			= 0x000000FF00000000;
const static uint64_t PIN_NUMBER_MASK								= 0x00000000FF000000;
const static uint64_t RESERVED_MASK									= 0x0000000000FFFFFF; // Change as needed

// Bit shifts for converting some data
// For using 8-bit numbers with the 64-bit HD
// ****************************************************************************
const static uint8_t INTERFACE_TYPE_SHIFT = 56;
const static uint8_t DIGITAL_DEVICE_SHIFT = 48;
const static uint8_t INTERFACE_INDEX_SHIFT = 40;
const static uint8_t DIGITAL_DEVICE_INDEX_SHIFT = 32;
const static uint8_t PIN_NUMBER_SHIFT = 24;
// without a specific pin.

public:
// For assigning an HD without yet having a pin number.
const static uint8_t INVALID_PIN_NUMBER = 0xFF; // An invalid pin number for a HD constructed

// Interface names
// The interface being controlled or read.
// ****************************************************************************
const static uint8_t INTF_INVALID						= 0xFF;
const static uint8_t INTF_PWM								= 0x01;
const static uint8_t INTF_GPIO							= 0x02;
const static uint8_t INTF_ADC								= 0x03;
const static uint8_t INTF_PWR_SWITCHING			= 0x04;
const static uint8_t INTF_ARDUINO						= 0x05;
const static uint8_t INTF_TEMP							= 0x06;
const static uint8_t INTF_TEL_PRESSURE			= 0x07;
const static uint8_t INTF_CURRENT						= 0x08;
const static uint8_t INTF_POWER_LINE				= 0x09;
const static uint8_t INTF_TEL_SWITCHES			= 0x0D;
const static uint8_t INTF_TEL_LEAK					= 0x0E;
const static uint8_t INTF_TEL_EMERGENCY_IO	= 0x0F;
const static uint8_t INTF_LEAK_WARN					= 0x10; // Leak warn LED
const static uint8_t INTF_ARDUINO_GPIO_PWM	= 0x11;
const static uint8_t INTF_ARDUINO_ADC				= 0x12;
const static uint8_t INTF_VREF							= 0x13;
// Digital Device names
// The device used to control or read an interface; note that these can overlap inteface names
// ****************************************************************************
const static uint8_t DEVICE_INVALID					= 0xFF;
const static uint8_t DEVICE_PWM							= 0x01;
const static uint8_t DEVICE_GPIO						= 0x02;
const static uint8_t DEVICE_ADC							= 0x03;
const static uint8_t DEVICE_ARDUINO					= 0x04;
const static uint8_t DEVICE_PRESSURE_I2C		= 0x05;

/**
 * @param hd Harware descriptor to be interpreted
 * @return Interface ID, which can be compared to internal enums.
 */

static inline uint8_t getInterfaceId(uint64_t hd) {
	return (uint8_t)((hd & INTERFACE_TYPE_MASK) >> INTERFACE_TYPE_SHIFT);
} // getInterface

/**
 * @param hd Harware descriptor to be interpreted
 * @return Device ID for comparison to internal enums
 */

static inline uint8_t getDeviceId(uint64_t hd) {
	return (uint8_t)((hd & DIGITAL_DEVICE_MASK) >> DIGITAL_DEVICE_SHIFT);
} // getInterface

/**
 * @param hd Harware descriptor to be interpreted
 * @return The index of the hardware interface, starting at 0.
 */

static inline uint8_t getInterfaceIndex(uint64_t hd) {
	return (uint8_t)((hd & INTERFACE_INDEX_MASK) >> INTERFACE_INDEX_SHIFT);
} // getInterface

/**
 * @param hd Harware descriptor to be interpreted
 * @return The index of the digital device, starting at 0.
 */

static inline uint8_t getDeviceIndex(uint64_t hd) {
	return (uint8_t)((hd & DIGITAL_DEVICE_INDEX_MASK) >>
	                 DIGITAL_DEVICE_INDEX_SHIFT);
} // getInterface

/**
 * @param hd Harware descriptor to be interpreted
 * @return The pin number of the interface, starting at 0.
 */

static inline uint8_t getInterfacePin(uint64_t hd) {
	return (uint8_t)((hd & PIN_NUMBER_MASK) >> PIN_NUMBER_SHIFT);
} // getInterface

/**
 * Create a hardware descriptor using the given data.
 * @param iface Interface
 * @param device Digital Device to communicate with
 * @param ifaceIndex Interface index, starting at 0
 * @param ifacePin Index referring to pin for interface, starting at 0
 * @return The full hardware descriptor.
 */

static inline uint64_t createHardwareDescriptor(uint8_t iface,
                                                uint8_t device,
                                                uint8_t ifaceIndex,
                                                uint8_t deviceIndex,
                                                uint8_t ifacePin) {
	// uint64_t inttoshift = (uint64_t) 0;
	// inttoshift = ((uint64_t)0xFF) << INTERFACE_TYPE_SHIFT;
	// printf("shifted: \t\t\t\t\t\t\t0x%.16llX\n", inttoshift);
	return (uint64_t)
	       (((uint64_t)iface) << INTERFACE_TYPE_SHIFT) |
	       ((((uint64_t)device)) << DIGITAL_DEVICE_SHIFT) |
	       (((uint64_t)ifaceIndex) << INTERFACE_INDEX_SHIFT) |
	       (((uint64_t)deviceIndex) << DIGITAL_DEVICE_INDEX_SHIFT) |
	       (((uint64_t)ifacePin) << PIN_NUMBER_SHIFT);
} // getInterface

/**
 * @param hd Already created harware descriptor that does not have a pin number yet
 * @param pinNumber Pin number to add.
 * @return Hardware descriptor with a pin number.
 */

static inline uint64_t addPinNumber(uint64_t hd, uint8_t pinNumber) {
	// Zero out pin number portion of hd
	// Shifts pin number into posistion
	// Mask pin number to ensure it only contains pin number
	// OR's pin number into given hd
	return (uint64_t)(hd & ~PIN_NUMBER_MASK) | ((((uint64_t)pinNumber) <<
	                                             PIN_NUMBER_SHIFT));
} // getInterface

/**
 * @return TODO
 */

const static char *interfaceIdToCharArray(uint8_t intf){
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
		return "INTF_TEL_TEMP";
		break;
	case INTF_TEL_PRESSURE:
		return "INTF_TEL_PRESSURE";
		break;
	case INTF_CURRENT:
		return "INTF_CURRENT";
		break;
	case INTF_POWER_LINE:
		return "INTF_POWER_LINE";
		break;
	case INTF_TEL_SWITCHES:
		return "INTF_TEL_SWITCHES";
		break;
	case INTF_TEL_LEAK:
		return "INTF_TEL_LEAK";
		break;
	case INTF_TEL_EMERGENCY_IO:
		return "INTF_TEL_EMERGENCY_IO";
		break;
	case INTF_LEAK_WARN:
		return "INTF_LEAK_WARN";
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
		return "no_valid_interface_found";
	} // switch
} // toCharArray

/**
 * @return TODO
 */

const static char *deviceIdToCharArray(uint8_t dev){
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
	case DEVICE_PRESSURE_I2C:
		return "DEVICE_PRESSURE_I2C";
		break;
	default:
		return "no_valid_device_found";
	} // switch
} // toCharArray
} // class HardwareDescriptor

;

#endif // ifndef HARDWARE_DESCRIPTOR
