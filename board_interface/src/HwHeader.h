#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include "HardwareDescriptor.cpp"
#include <ros/ros.h>
// #include <cstring.h>
// Don't change this as it's linked to intentional PinBus limitations
const uint8_t MAX_PIN_COUNT = 16;

// Datatype used for pin values. Included here to allow for easy expansion.
// Current datatype is a 16-bit unsigned integer.
#define PIN_VALUE_DATA_TYPE uint16_t

// Digital communication types
const uint8_t COMM_TYPE_I2C = 0x20;
const uint8_t COMM_TYPE_SPI = 0x21;

#endif /* ifndef HW_HEADER */
