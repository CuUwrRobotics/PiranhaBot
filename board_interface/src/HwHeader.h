#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include "HardwareDescriptor.cpp"
#include <ros/ros.h>
// #include <cstring.h>
// Don't change this as it's linked to intentional PinBus limitations
const uint8_t MAX_PIN_COUNT							= 16;

// Digital communication types
const uint8_t COMM_TYPE_I2C							= 0x20;
const uint8_t COMM_TYPE_SPI							= 0x21;

// Data packet types for ROS data transfers. (This just says what the values
// passed are supposed to mean, ie, packets.)
enum DataType {PACKET_INVALID,
	             PACKET_PWM_DUTY_100,
	             PACKET_PWM_ON_TICKS,
	             PACKET_PWM_FREQ};
// const uint8_t PACKET_INVALID						= 0x00;
// const uint8_t PACKET_PWM_DUTY_100				= 0x01; // PWM ON duty cycle as % of 100
// const uint8_t PACKET_PWM_ON_TICKS				= 0x02; // PWM ticks on
// const uint8_t PACKET_PWM_FREQ						= 0x03; // PWM frquency setter

#endif /* ifndef HW_HEADER */
