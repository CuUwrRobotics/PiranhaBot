// =============== INTERFACES ===============
// The numbers that refer to different types of interfaces.
#define INTF_PWM 0
#define INTF_GPIO 1
#define INTF_ADC 2
#define INTF_PWR_SWITCHING 3
#define INTF_TEL_TEMP 4
#define INTF_ARDUINO 5 // Other interfaces use arduino, and are listed below
#define INTF_TEL_CURRENT 6
#define INTF_TEL_3V3 7
#define INTF_TEL_5V 8
#define INTF_TEL_12V 9
#define INTF_TEL_VIN 10
#define INTF_TEL_SWITCHES 11
#define INTF_LEAK 12
#define INTF_EMERGENCY_IO 13
#define INTF_LED 14 // Leak warn LED. Not controlled by external nodes.
#define INTF_ARDUINO_GPIO_PWM 15
#define INTF_ARDUINO_ADC 16

// =============== DEVICE TYPES ===============

/* Only one device can request control of an interface's pin at a time. The request
 * types are used to define how the pin will be controlled. In a request, a node
 * must use the request service and include both the request type and what pin
 * to use. These must agree with the YAML configuration file. Example for a
 * servo motor:
 * ************************
 * Type: PWM_MTR_SERVO
 * Pin: 21
 * Command: 0
 * ************************
 * If a device is using this already, -1 will be returned. Otherwise, a unique
 * ID number will be returned. This ID must be used in any attempt to change
 * the status of the motor at this pin. These precautions exist to avoid sending
 * a signal to a wrong device while protyping, which could be very dangerous.
 *
 * This restriction does not exist for inputs, which can be read at any point. A
 * requst for an input will return the requested value in place of an id #.
 *
 * ANY request that results in an error will return negatvie values indicating the error.
 */
// PWM types
#define PWM_DIRECT              // cmd1 = duty cycle, cmd2 = period.
#define PWM_MTR_ESC             // 50 Hz ESC
#define PWM_MTR_ESC_60
#define PWM_MTR_ESC_490
#define PWM_MTR_SERVO           // 50 Hz servo
#define PWM_MTR_SERVO_60
#define PWM_LIGHT               // cmd1 = duty cycle, cmd2 = period.

// GPIO Types
#define IO_OUT
// *********************** MOVE TO MESSAGE TYPE
// #define IO_IN                   // Input with pullup
// #define IO_ALL
// #define IO_IN_NO_PULLUP

// *********************** MOVE TO MESSAGE TYPE
// ADC Types
// #define ADC_IN                  // Voltage is real, not estimated. (Uses diode refrence.)
// #define VALUE_ADC_DIRECT              // Reads 10-bit number directly recieved by ADC.
// #define VALUE_ADC_DIRECT_REF_3V       // Reads 10-bit reading of the vref diode.

// Power Control
#define POWER_SWITCHING_OUT
// #define POWER_SWITCHING_PWM     // Software PWM. cmd1 = duty cycle, cmd2 = period.

// On-board Arduino interfacing
#define ARDUINO_ADC
#define ARDUINO_IO_IN           // Arduino inputs with pullup
#define ARDUINO_LED             // LED indicator on arduino pin 13.
#define ARDUINO_IO_OUT
#define ARDUINO_IO_IN_NO_PULLUP

// Info request
#define INFO_REQUEST            // See info requests below.

// =============== ERROR TYPES ===============
// WARNING: OUTDATED
/* Any request that does not succeed will recieve a negative number in place of
 * its regular return value. This table refers to the error types.
 * Some errors are given only when a request is first made, some are given when
 * an attempt is made to control a pin that was already succesfully requested.
 * Also:
 * Input read errors: A problem reading the input of a device. Data was recieved, but something is
 *wrong with it.
 * Hardware errors: These are related to communicating with the hardware device, and indicate that
 *somthing is wrong with the hardware node or the board.
//
*/
// // First request errors
// #define ERR_PIN_IN_USE
// #define ERR_REQ_DOES_NOT_MATCH_CFG
// // Control errors
// #define ERR_BAD_ID_NUMBER       // ID number given does not match the ID for this interface/pin
// #define ERR_BAD_DUTY            // Duty cycle not between 0 and 1
// #define ERR_BAD_PERIOD          // Duty cycle period could not be applied
// #define ERR_BAD_ID_NUMBER       // ID number given does not match the ID for this interface/pin
// // Input read errors
// #define ERR_DATA_OUT_OF_RANGE   // Data is out of the range given by a config file.
// // Hardware errors
// #define ERR_COULD_NOT_FIND_IF   // Could not connect to the interface.
// #define ERR_IF_COMM             // Communication with interface failed.
// // Generic errors
// #define ERR_STOPPED             // Emergency stop was called; the request will be processed, but
// 	                              // will not be used yet.
// #define ERR_GENERAL             // Hardware control node is running, but something's wrong with it
// 	                              // (maybe config).

// =============== INFO REQUESTS ===============

/* A request to get information about the hardware configuration. Pin # defines
 * request type. Example request:
 * ************************
 * Type: INFO_REQUEST
 * Pin: PWM_1_FREQUENCY
 * Command: 0
 * ************************
 */
 #define PWM_1_FREQUENCY        // Returns the theoretical (intended) PWM frequency
 #define PWM_1_FREQUENCY_ACTUAL // Returns the actual (calculated) PWM frequency
