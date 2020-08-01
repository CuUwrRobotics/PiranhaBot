#ifndef BIT_TESTING_H
#define BIT_TESTING_H

#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
#include "Device.h"
#include "Interface.h"
#include "ConsoleColors.h"

namespace bit_testing {
/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPwm(Interface *intf, Device *dev, bool direction);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testGpio(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPower(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testLeak(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testEmergencyIo(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testLed(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testAdc(Interface *intf, Device *dev);

/**
 * Fancy display of all devices, interfaces, and pins.
 * TODO: At end, cycle through and see if any interfaces are unconnected.
 */

void dumpConfiguration(bool shrinkRepeatedPins, Interface **interfaces,
                       Device **devices);
} // namespace bit_testing

#endif /* ifndef BIT_TESTING_H */
