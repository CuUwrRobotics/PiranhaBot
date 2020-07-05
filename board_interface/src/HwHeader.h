#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include "HardwareDescriptor.cpp"
#include <ros/ros.h>

const static uint8_t TOTAL_INTERFACES		= 10;
const static uint8_t TOTAL_DEVICES			= 7;

const uint8_t MAX_PIN_COUNT							= 16;

// Digital communication types
const uint8_t COMM_TYPE_I2C							= 0x20;
const uint8_t COMM_TYPE_SPI							= 0x21;

// Data packet types for ROS data transfers. Some have multiple peices of data
// stored in an array. Some are intended for settigns, not for directly
// writing/redaing data.
enum DataType {PACKET_INVALID,
	             // PWM data tyeps
	             PACKET_PWM_DUTY_100,
	             PACKET_PWM_ON_TICKS,
	             PACKET_PWM_FREQ, // frequency setting
	             // Only one for GPIO
	             PACKET_GPIO_STATE,
	             // Some ADC types are settings for calibration
	             PACKET_ADC_DIRECT_10BIT, // 10-bit range, direct from the ADC
	             PACKET_ADC_VOLTAGE,
	             // TWO VALUES! [0] = actual voltage; [1] = tolerance in volts
	             PACKET_ADC_VOLTAGE_WITH_TOLERANCE,
	             // TWO VALUES! [0] * measured = actual voltage; [1] * actual = tolerance
	             PACKET_ADC_AVCC_OFFSET_AND_TOLERANCE_RATIOS};

// For colorful console output
char D_GRAY[10] = "\033[1;30m";
char RED[10] = "\033[1;31m";
char YELLOW[10] = "\033[1;33m";
char CYAN[10] = "\033[0;36m";
char GREEN[10] = "\033[1;32m";
char WHITE[10] = "\033[1;37m";
char NO_COLOR[7] = "\033[0m";

#endif /* ifndef HW_HEADER */
