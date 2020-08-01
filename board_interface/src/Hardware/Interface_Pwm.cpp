#include "Interface_Pwm.h"
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 16; // number of pins
// const static Interface_t interfaceTypeId = INTF_PWM; // The ID for this intf
// const static Device_t parentDeviceTypeId = DEVICE_PWM; // The IF for the device
// ----------------------------------------------------------------------------


/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Pwm::prepareInterface(){
	pinBus.setAllPins(MODE_OUTPUT);
	commDevice->setPinModes(pinBus);
} // prepareInterface

DataError_t Interface_Pwm::readPin(PinValue_t *valueIn) {
	if (!(valueIn->pin >= 0 && valueIn->pin < PIN_COUNT))
		return ERROR_INTF_PIN_INVALID;

	PinValue_t val;
	DataError_t errorVal;
	// Reads the pin on the device. Formatting/scaling/other data changes happen below.
	// val.fmt = VALUE_ADC_DIRECT; // Set format
	val.pin = pinBus.getPin(valueIn->pin); // Go from local pin to the device pin
	val.data = valueIn->data; // Uses input data to store data
	// errorVal = commDevice->getPinValue(&val); // Get the data

	// Format data and return with the error/success code from device
	switch (valueIn->fmt) {
	case VALUE_PWM_FREQ:
	case VALUE_PWM_ON_TICKS:
		// These can both be readdirectly from PWM devices.
		val.fmt = valueIn->fmt;
		return commDevice->getPinValue(&val);
		break;
	case VALUE_PWM_DUTY_100:
		val.fmt = VALUE_PWM_ON_TICKS;
		errorVal = commDevice->getPinValue(&val);
		valueIn->data[0] = (valueIn->data[0] / (MAX_PWM_TICKS / 100)); // TODO: MAX_PWM_TICKS
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // readPin

DataError_t Interface_Pwm::writePin(PinValue_t *valueIn) {
	if (!(valueIn->pin >= 0 && valueIn->pin < PIN_COUNT))
		return ERROR_INTF_PIN_INVALID;

	PinValue_t val;
	DataError_t errorVal; // not used
	// Reads the pin on the device. Formatting/scaling/other data changes happen below.
	// val.fmt = VALUE_GPIO_STATE; // Set format
	val.pin = pinBus.getPin(valueIn->pin); // Go from local pin to the device pin
	val.data = valueIn->data;
	// errorVal = commDevice->setPinValue(&val); // Get the data

	if (!(commDevice->ready()))
		return ERROR_DEV_N_READY;

	switch (valueIn->fmt) {
	case VALUE_PWM_FREQ:
		if (val.data[0] > MAX_VALUE_PWM_FREQUENCY_VALUE)
			val.data[0] = MAX_VALUE_PWM_FREQUENCY_VALUE;
		else if (val.data[0] < MIN_VALUE_PWM_FREQUENCY_VALUE)
			val.data[0] = MIN_VALUE_PWM_FREQUENCY_VALUE;
		val.fmt = VALUE_PWM_FREQ; // Set format
		return commDevice->setPinValue(&val);
		break;
	case VALUE_PWM_DUTY_100:
		if (val.data[0] > 100) // Max
			val.data[0] = MAX_PWM_TICKS;
		else if (val.data[0] < 0)
			val.data[0] = MIN_PWM_TICKS;
		else
			val.data[0] = (val.data[0] * MAX_PWM_TICKS) / 100;
		val.fmt = VALUE_PWM_ON_TICKS;
		return commDevice->setPinValue(&val);
		break;
	case VALUE_PWM_ON_TICKS:
		if (val.data[0] > MAX_PWM_TICKS)
			val.data[0] = MAX_PWM_TICKS;
		if (val.data[0] < MIN_PWM_TICKS)
			val.data[0] = MIN_PWM_TICKS;
		else
			val.data[0] = val.data[0];
		val.fmt = VALUE_PWM_ON_TICKS;
		return commDevice->setPinValue(&val);
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // writePin

DataError_t Interface_Pwm::writeConfig(InterfaceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeConfig

DataError_t Interface_Pwm::readConfig(InterfaceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readConfig

DataError_t Interface_Pwm::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Pwm::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

// **** OVERRIDES BASE CLASS ****
uint8_t Interface_Pwm::setPinMode(uint8_t pinNumber, PinMode_t pinMode){ // TODO: DataError_t
	ROS_INFO("setPinMode: Pin modes cannot be written to the %s interface!",
	         interfaceIdToCharArray(interfaceTypeId));
	return 0;
} /* setPinMode */
