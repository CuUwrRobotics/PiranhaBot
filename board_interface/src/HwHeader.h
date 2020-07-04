#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include "HardwareDescriptor.cpp"
#include <ros/ros.h>

const static uint8_t TOTAL_INTERFACES = 8;
const static uint8_t TOTAL_DEVICES = 5;

const uint8_t MAX_PIN_COUNT							= 16;

// Digital communication types
const uint8_t COMM_TYPE_I2C							= 0x20;
const uint8_t COMM_TYPE_SPI							= 0x21;

// Data packet types for ROS data transfers. (This just says what the values
// passed are supposed to mean, ie, packets.)
enum DataType {PACKET_INVALID,
	             PACKET_PWM_DUTY_100,
	             PACKET_PWM_ON_TICKS,
	             PACKET_PWM_FREQ,
	             PACKET_GPIO_STATE};

// For colorful console output
char D_GRAY[10] = "\033[1;30m";
char RED[10] = "\033[1;31m";
char YELLOW[10] = "\033[1;33m";
char CYAN[10] = "\033[0;36m";
char GREEN[10] = "\033[1;32m";
char WHITE[10] = "\033[1;37m";
char NO_COLOR[7] = "\033[0m";

#endif /* ifndef HW_HEADER */
