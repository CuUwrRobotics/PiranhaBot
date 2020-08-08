#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "HardwareData.h"
#include "Logger.h"
#include "ConsoleColors.h"
#include "AllDevicesInterfaces.h"
#include "PinBus.h"
#include "BitTesting.h"
#include "watchdog/pet_dog_msg.h"

// If incompatible enums are compared, an error will be thrown instead of a warning.
#pragma GCC diagnostic error "-Wenum-compare"

// Digital communication types (TODO: GET RID OF THESE!)
const float actualDiodeVoltage = 3; // This should be measured for accuracy
const float actualDiodeTolerance = .06; // 2%
float adcTolerance = .01; // (5v/2^9) for a 10-bit ADC after removing LSB
float avccTheoretical = 5;
uint8_t vrefIndex; // Assigned at creation of VREF object.

#endif /* ifndef HW_HEADER */
