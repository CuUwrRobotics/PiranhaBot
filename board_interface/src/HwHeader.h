#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"

// If incompatible enums are compared, an error will be thrown instead of a warning.
#pragma GCC diagnostic error "-Wenum-compare"

const static uint8_t TOTAL_INTERFACES		= 18;
const static uint8_t TOTAL_DEVICES			= 8;

const uint8_t MAX_PIN_COUNT							= 16;

// Digital communication types (TODO: GET RID OF THESE!)
const float actualDiodeVoltage = 3; // This should be measured for accuracy
const float actualDiodeTolerance = .06; // 2%
float adcTolerance = .01; // (5v/2^9) for a 10-bit ADC after removing LSB
float avccTheoretical = 5;
uint8_t vrefIndex; // Assigned at creation of VREF object.

/**
 * Defines the formats for assigning or reading pin values.
 * IMPORTANT: If data being transferred to/from an interface/device is not for
 * actually reading/writing data on a device's pin, it is a configuration.
 */
enum DataValueFormat_t {VALUE_INVALID, /*< Default if data never gets set. */
	                      // PWM
	                      // ===
	                      VALUE_PWM_DUTY_100, /*< Duty cycle */
	                      VALUE_PWM_ON_TICKS, /*< On time, in ticks, out of 4096 */
	                      VALUE_PWM_FREQ, /*< Frequency setting */
	                      // GPIO
	                      // ====
	                      VALUE_GPIO_STATE,
	                      // ADC
	                      // ===
	                      VALUE_ADC_DIRECT, /*< Direct data from the ADC. Supported by any ADC based
	                                         * interface */
	                      VALUE_ADC_VOLTAGE, /*< Calculated voltage */

	                      /** Two values:
	                       * [0] = Measured voltage
	                       * [1] = Tolerance in volts */
	                      VALUE_ADC_VOLTAGE_WITH_TOLERANCE,
	                      // CURRENT
	                      // =======
	                      VALUE_CURRENT_AMPS, /*< Curernt measured */

	                      /** Two values:
	                       * [0] = Measured current
	                       * [1] = Tolerance in amps */
	                      VALUE_CURRENT_AMPS_WITH_TOLERANCE,

	                      // VOLTAGE REFRENCE
	                      // ================
	                      VALUE_REF_VOLTAGE_NO_CORRECT, /*< Uncorrected meaured diode voltage. */
	                      // TEMP
	                      // ====

	                      /** Two values:
	                       * [0] = Measured temperature in degrees C
	                       * [1] = Tolerance in degrees C */
	                      VALUE_TEMP_C_WITH_TOLERANCE};

/**
 * Defines the formats for assigning or reading interface configurations.
 */
enum InterfaceConfigFormat_t {ICFG_INTERFACE_CONFIG_INVALID, /*< Default if data never gets set. */
	                            // ADC
	                            // ===

	                            /** When the actual level of the 5V rail is known,
	                             * these can be used to assign it for more accurate ADC
	                             * measurements. Meant to be set in all ADC interfaces after
	                             * reading this value from \ref Interface_Voltage_Refrence.
	                             * Two values:
	                             * [0] = Ratio of (intended AVCC):(actual AVCC)
	                             * [1] = Multiply this value by actual voltage */
	                            ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS,

	                            // VOLTAGE REFRENCE
	                            // ================
	                            ICFG_ADC_STEPS, /*< The number of ADC steps. TODO: MOVE TO ADC DEVICE
	                                             * */
	                            ICFG_ADC_AVCC_VOLTAGE, /*< The intended ADC device AVCC. TODO: MOVE TO
	                                                    * ADC DEVICE */

	                            /** Two values:
	                             * [0] = Known voltage of the refrence (3V refrence ==> 3V)
	                             * [1] = Tolerance in volts for the known value. */
	                            ICFG_REF_KNOWN_VOLTS_WITH_TOLERANCE,
	                            ICFG_REF_ADC_TOLERANCE, /*< Tolerance of ADC measurements
	                                                     * TODO: MOVE TO DEVICE */
	                            ICFG_REF_NUM_CYCLES, /*< Number of cycles to measure the
	                                                  * refrence for an average value. Should
	                                                  * be >= 10. */
	                            ICFG_REF_READY, /*< Checks that voltage refrence is
	                                             * ready. Check this prior to assigning values
	                                             * from it, and  before any very critical
	                                             * measurements. */

	                            // POWER LINE
	                            // ==========

	                            /** Two values:
	                             * [0] = Known value of the high resistor
	                             * [1] = Tolerance of the above value in %. If resistance
	                             * was actually measured, this is zero. */
	                            ICFG_PL_HIGH_RESISTOR_WITH_TOLERANCE,

	                            /** Two values:
	                             * [0] = Known value of the low resistor
	                             * [1] = Tolerance of the above value in %. If resistance
	                             * was actually measured, this is zero. */
	                            ICFG_PL_LOW_RESISTOR_WITH_TOLERANCE};

/**
 * Defines the formats for assigning or reading device configurations.
 *
 * TODO
 */

enum DeviceConfigFormat_t {DCFG_DEVICE_CONFIG_INVALID, /*< Default if data never gets set. */
	                         // ADC
	                         // ===
	                         DCFG_ADC_STEPS, /*< The number of ADC steps.
	                                          * */
	                         DCFG_ADC_AVCC_VOLTAGE /*< The intended ADC device AVCC. */
};

/**
 * For transferring data to interfaces. Values will be interpreted and sent to
 * the device.
 *
 * When writing a value, assign the format and use a pointer to the data. This
 * may need to be an array.
 *
 * When reading a value, assign a format, and the called function will return
 * a pointer to a static variable, which should have the value taken from it. The
 * variable may be overwritten by other reads, so don't rely on the pointer to hold
 * your data!
 */

struct PinValue_t {
	DataValueFormat_t fmt; /*< The data format. */
	uint8_t pin; /*< Where to assign the data */
	float *data; /*< Data to pass along. While writing, this may be changed. */
};

/**
 * For transferring configurations to interfaces. Configruations will be
 * interpreted and stored.
 *
 * Data read/writes are as in \ref PinValue.
 */

struct InterfaceConfig_t {
	InterfaceConfigFormat_t fmt; /*< The data format. */
	float *data; /*< Data to pass along. While writing, this may be changed. */
};

/**
 * For transferring configurations to interfaces. Configruations will be
 * interpreted and sent to the parent device.
 * These should be pretty limited! Don't use this for assigning constant data like
 * how many steps the ADC has, as each device subclass is meant to contain that
 * data anyway.
 *
 * Data read/writes are as in \ref PinValue.
 */

struct DeviceConfig_t {
	DeviceConfigFormat_t fmt; /*< The data format. */
	float *data; /*< Data to pass along. While writing, this may be changed. */
};

/**
 * Errors returned by any pin value or configruation read/write.
 * TIP: Check if there was an error using if(!errorValue) {}, since
 * ERROR_SUCCESS == 0.
 */

enum DataError_t {
	ERROR_SUCCESS, /*< Nothing wrong. This MUST be first error for easier error checking */
	ERROR_GENERAL, /*< For use as placeholder */
	ERROR_NOT_AVAIL, /*< The requested read/write format is not available for the interface */
	ERROR_WROTE_INPUT, /*< Tried to write to a pin in input mode. */
	ERROR_DEV_PIN_INVALID, /*< Pin was out of device pins range. */
	ERROR_INTF_PIN_INVALID, /*< Pin was out of interface pins range. */
	ERROR_DEV_N_READY, /*< Device was not ready yet. */
	ERROR_INTF_N_READY, /*< Interface was not ready yet. */
	ERROR_INVALID_DATA /*< Data for write was invalid. */
};

const char *errorCharArray(DataError_t err) {
	switch (err) {
	case ERROR_SUCCESS:
		return "No error";
		break;
	case ERROR_GENERAL:
		return "General error";
		break;
	case ERROR_NOT_AVAIL:
		return "Data format not accepted by interface";
		break;
	case ERROR_WROTE_INPUT:
		return "Tried to write to a pin in input mode";
		break;
	case ERROR_DEV_PIN_INVALID:
		return "Pin was out of device pins range";
		break;
	case ERROR_INTF_PIN_INVALID:
		return "Pin was out of interface pins range";
		break;
	case ERROR_DEV_N_READY:
		return "Device not ready";
		break;
	case ERROR_INTF_N_READY:
		return "Interface not ready";
		break;
	case ERROR_INVALID_DATA:
		return "Data for write was invalid";
		break;
	default:
		return "Unknown error";
		break;
	} /* switch */
} /* errorCharArray */

// For colorful console output
char D_GRAY[10] = "\033[1;30m";
char RED[10] = "\033[1;31m";
char YELLOW[10] = "\033[1;33m";
char CYAN[10] = "\033[0;36m";
char GREEN[10] = "\033[1;32m";
char WHITE[10] = "\033[1;37m";
char NO_COLOR[7] = "\033[0m";

#define log_info(a, ...) \
	ROS_INFO(a__VA_OPT__(, ) __VA_ARGS__)
#define log_warn(a, ...) \
	ROS_WARN("Internal Warning: " a " [from %s (in %s:%d)]\n" \
	         __VA_OPT__(, ) __VA_ARGS__,  \
	         __FUNCTION__, __FILE__, __LINE__)
#define log_error(a, ...) \
	ROS_ERROR("Internal Error: " a " [from %s (in %s:%d)]\n" \
	          __VA_OPT__(, ) __VA_ARGS__,  \
	          __FUNCTION__, __FILE__, __LINE__)

#endif /* ifndef HW_HEADER */
