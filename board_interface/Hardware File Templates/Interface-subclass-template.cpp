/* This class is a tempalte for copy/pasting to actual class files. It is
 * intened tospeed up making new devices.
 */

// Generic headers
 #include "HwHeader.h"
 #include "Devices_interfaces.h"
 #include "pinData.h"

/**
 * Interface_Template
 */
class Interface_Template : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char INTERFACE_NAME[9] = "TEMPLATE"; // Length MUST = (# of chars) + 1
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 16;
// IDs which indicate what this is and what it should be connected to
const static uint8_t interfaceTypeId = HardwareDescriptor::INTF_PWM;
const static uint8_t parentDeviceTypeId = HardwareDescriptor::DEVICE_PWM;

public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline uint8_t getInterfaceTypeId(){
	return interfaceTypeId;
} // getInterfaceTypeId

//
inline uint8_t getParentTypeId(){
	return parentDeviceTypeId;
} // getParentTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

//
inline char *getInterfaceName(){
	return INTERFACE_NAME;
} // getPinCount

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

 /** Called at init. Should assign default modes to the pinBus object.
  * updateData() will be called after this, so there's no needto call it here.
  */

 void setDefaultModes(){
 	// PinMode modes[4] = {MODE_OUTPUT, MODE_INPUT, MODE_OUTPUT, MODE_OUTPUT};
 	pinBus.setAllPins(MODE_OUTPUT);
 	commDevice->setPinModes(pinBus, interfaceTypeId);
 } // setDefaultModes

 /**
  * @param pin TODO
  * @param dataType TODO
  * @return TODO
  */

 uint16_t readPin(uint8_t pin, DataType dataType) {
 	// Requested format is available directly from device
 	if (dataType == PACKET_PWM_FREQ || dataType == PACKET_PWM_ON_TICKS)
 		return commDevice->getPinValue(pinBus.getPin(pin),
 		                               dataType);
 	// If program got here, data reformatting is needed from any other possible formats
 	if (dataType == PACKET_INVALID) {
 		ROS_ERROR("writePin: Recieved invalid dataType.\n");
 		return 0;
 	}
 	if (dataType == PACKET_PWM_DUTY_100) {
 		uint16_t pinValue = commDevice->getPinValue(pinBus.getPin(pin),
 		                                            PACKET_PWM_ON_TICKS);

 		// printf("Read: %d", pinValue);
 		return (pinValue / (MAX_PWM_TICKS / 100));
 	} else {
 		ROS_ERROR("writePin: Recieved invalid dataType: %d\n", dataType);
 		return 0;
 	}
 } // readPin

 /**
  * writePin interprets a value to be assigned to a pin, then tells the parent
  * device commDevice to setPinValue using the commDevice's data conventions.
  * TODO: enumerate errors better
  * @param pinNumber The pin nunmber to try to drive
  * @param hd The full hardware descriptor.
  * @return 1 if there are no errors.
  */
 uint8_t writePin(uint8_t pinNumber, uint16_t *data, DataType dataType,
                  uint64_t hd){
 	// Check that HD matches
 	if (hd != getHardwareDescriptor(pinNumber)) {
 		ROS_ERROR("writePin: Bad HD, not driving pin.\n");
 		return 0;
 	}
 	if (!(commDevice->ready())) {
 		ROS_ERROR("writePin: Device for interface not ready.\n");
 		return 0;
 	}
 	if (dataType == PACKET_INVALID) {
 		ROS_ERROR("writePin: Recieved invalid dataType.\n");
 		return 0;
 	}
 	uint16_t dataForDevice;
 	DataType dataTypeForDevice;
 	if (dataType == PACKET_PWM_FREQ) {
 		dataTypeForDevice = PACKET_PWM_FREQ;
 		// check/set frequency
 		if (data[0] > MAX_PWM_FREQUENCY_VALUE) // Max value for frequency
 			dataForDevice = MAX_PWM_FREQUENCY_VALUE;
 		else if (data[0] < 1) // Min value for frequency
 			dataForDevice = 1;
 		else dataForDevice = data[0];
 	} else if (dataType == PACKET_PWM_DUTY_100) {
 		dataTypeForDevice = PACKET_PWM_ON_TICKS;
 		if (data[0] > 100) // Max
 			dataForDevice = MAX_PWM_TICKS;
 		else
 			dataForDevice = (data[0] * MAX_PWM_TICKS) / 100;
 		// set PWM on/off ticks based on duty cycle
 	} else if (dataType == PACKET_PWM_ON_TICKS) {
 		dataTypeForDevice = PACKET_PWM_ON_TICKS;
 		if (data[0] > MAX_PWM_TICKS) // Max
 			dataForDevice = MAX_PWM_TICKS;
 		else
 			dataForDevice = data[0];
 		// check/set PWM on/off ticks based on on ticks
 	} else {
 		ROS_ERROR("writePin: Recieved invalid dataType: %d\n", dataType);
 		return 0;
 	} // Parent device expects data format PACKET_PWM_ON_TICKS
 	return commDevice->setPinValue(pinBus.getPin(pinNumber), &dataForDevice,
 	                               dataTypeForDevice, interfaceTypeId);
 } /* writePin */

 // **** OVERRIDE OVER PARENT CLASS ****
 uint8_t setPinMode(uint8_t pinNumber, PinMode pinMode, uint64_t hd){
 	ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
 	         INTERFACE_NAME);
 	return 0;
 } /* setPinMode */
 }

 ;
