#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include "HardwareDescriptor.cpp"
#include <ros/ros.h>

const static uint8_t TOTAL_INTERFACES		= 18;
const static uint8_t TOTAL_DEVICES			= 8;

const uint8_t MAX_PIN_COUNT							= 16;

// Digital communication types
const uint8_t COMM_TYPE_I2C							= 0x20;
const uint8_t COMM_TYPE_SPI							= 0x21;

// Digital communication types
const float actualDiodeVoltage = 3; // This should be measured for accuracy
const float actualDiodeTolerance = .06; // 2%
float adcTolerance = .01; // (5v/2^9) for a 10-bit ADC after removing LSB
float avccTheoretical = 5;
const uint8_t vrefIndex = 12;

// Data packet types for ROS data transfers. Some have multiple peices of data
// stored in an array. Some are intended for settigns, not for directly
// writing/redaing data.
enum DataType {PACKET_INVALID,
	             // PWM
	             // ===
	             PACKET_PWM_DUTY_100,
	             PACKET_PWM_ON_TICKS,
	             PACKET_PWM_FREQ, // frequency setting
	             // GPIO
	             // ====
	             PACKET_GPIO_STATE,
	             // ADC
	             // ===
	             // Some ADC types are settings for calibration
	             PACKET_ADC_DIRECT, // Direct data from the ADC. Supported by any ADC based interface
	             PACKET_ADC_VOLTAGE,
	             PACKET_ADC_STEPS, // Only for interface <-> device data
	             PACKET_ADC_AVCC_VOLTAGE, // Only for interface <-> device data
	             // TWO VALUES! [0] = actual voltage; [1] = tolerance in volts
	             PACKET_ADC_VOLTAGE_WITH_TOLERANCE,
	             // TWO VALUES! [0] * measured = actual voltage; [1] * actual = tolerance
	             PACKET_ADC_OFFSET_AND_TOLERANCE_RATIOS,
	             // CURRENT
	             // =======
	             // Current will also use all ADC settings/calibration data.
	             PACKET_CURRENT_AMPS, // Curernt from ADC
	             // TWO VALUES! [0] = actual voltage; [1] = tolerance in volts
	             PACKET_CURRENT_AMPS_WITH_TOLERANCE,
	             // VOLTAGE REFRENCE
	             // ================
	             // TWO VALUES! [0] = known voltage of the refrence; [1] tolerance in volts
	             PACKET_REF_KNOWN_VOLTS_WITH_TOLERANCE,
	             PACKET_REF_ADC_TOLERANCE, // Tolerance of ADC measurements
	             PACKET_REF_NUM_CYCLES, // Number of cycles to spend measuring for an average.
	             PACKET_REF_READY, // Checks that voltage refrence is ready.
	             PACKET_REF_VOLTAGE_NO_CORRECT, // Uncorrected meaured diode voltage
	             // TEMP
	             // ====
	             PACKET_TEMP_C_WITH_TOLERANCE, //
	             // POWER LINE
	             // ==========
	             PACKET_PL_HIGH_RESISTOR_WITH_TOLERANCE, //
	             PACKET_PL_LOW_RESISTOR_WITH_TOLERANCE //
};

// For colorful console output
char D_GRAY[10] = "\033[1;30m";
char RED[10] = "\033[1;31m";
char YELLOW[10] = "\033[1;33m";
char CYAN[10] = "\033[0;36m";
char GREEN[10] = "\033[1;32m";
char WHITE[10] = "\033[1;37m";
char NO_COLOR[7] = "\033[0m";

#endif /* ifndef HW_HEADER */
