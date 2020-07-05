#ifndef DEVICE_ADC_MCP3008_H
#define DEVICE_ADC_MCP3008_H
#include <stdint.h>

// Scale for avcc voltages when stored in integer format. Depends on ADC precision.
// 100 --> 5.00v == 500
// 1000 --> 5.000v == 5000
// Due to noise, a minimum tolerance of +- 1 LSB must be assumed
// For MCP3008, a 10-bit ADC:
// :: 5.0v / (2^9) = 0.01v
// Smallest precision ==> .001v
// ==> 100:1v = scale
const float VOLTAGE_SCALE = 1;

// Voltage that the LSB represents, provided with its inverse for simpler math
// For MCP3008, a 10-bit ADC:
// :: 5.0v / (2^10) = 0.000976V ~= .005
const float ADC_BITWIDTH = 1024;
const float VOLTS_PER_LSB = 0.00488;
const float LSB_PER_VOLT = 204.8;


const float AVCC_THEORETICAL_VALUE = 5.00;

#endif /* ifndef DEVICE_ADC_MCP3008_H */
