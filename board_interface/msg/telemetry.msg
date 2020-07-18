# Array indexes for the voltage and current
uint8 VOLTAGE_3V3 = 0
uint8 VOLTAGE_5V = 1
uint8 VOLTAGE_12V = 2
uint8 VOLTAGE_VIN = 3
uint8 CURRENT_0 = 0
uint8 CURRENT_1 = 1


# Telemetry information.
int8 temperature
int8 humidity
float current[2]
float power_line_voltages[4]
uint8 switches # Binary output;  0000dcba, where a=1, b=2, etc.
uint8 leak # Binary output;  hgfedcba, where a=1, b=2, etc.
uint8 em_io # Returns binary result: 00000cba, where a=1, b=2, etc.
