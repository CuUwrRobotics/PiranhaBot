#ifndef DEVICE_ADC_MCP3008_H
#define DEVICE_ADC_MCP3008_H
#include <stdint.h>

// Total steps for the ADC = 2^(bitwidth)
const float ADC_STEPS = 1024;

// Theoretical AVCC voltage of 5v. For other devices, this may vary.
// Don't meaure and calibrate this value; calibration is handled in real time
// by onboard ADCs using voltage refrences. Just enter the theoretical,
// intended value.
const float AVCC_THEORETICAL_VALUE = 5.00;

#endif /* ifndef DEVICE_ADC_MCP3008_H */
