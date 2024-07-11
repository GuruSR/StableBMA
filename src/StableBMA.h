#pragma once

/* Forked from bma.h by GuruSR (https://www.github.com/GuruSR/StableBMA) 
 * This fork is to improve Watchy functionality based on board version (via RTCType).
 * Version 1.0, February  6, 2022 - Initial changes for Watchy usage.
 * Version 1.1, February  8, 2022 - Fixed readTemperatureF to show F properly.
 * Version 1.2, July     19, 2022 - Fixed readTemperatureF to include errors.  License Update.
 * Version 1.3, December 31, 2023 - Added orientation for V3.0 and cleaned up temperature code.
 * Version 1.4, February 24, 2024 - Added Low Power mode to the defaultConfig().
 * Version 1.5, July      9, 2024 - Fixed wrong getAccel assignments and added conditionBMA.
 *
 * MIT License
 *
 * Copyright (c) 2020 Lewis He
 * Copyright (c) 2022 for StableBMA GuruSR
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * StableBMA is a fork of:
 * bma.h- Arduino library for Bosch BMA423 accelerometer software library.
 * Created by Lewis He on July 27, 2020.
 * github:https://github.com/lewisxhe/BMA423_Library
*/

#ifndef STABLEBMA_H_INCLUDED
#define STABLEBMA_H_INCLUDED

#ifdef  ARDUINO
#include <Arduino.h>
#else
#include <stdlib.h>
#endif

#include "bma423.h"

#ifndef WATCHY_H
enum {
    DIRECTION_TOP_EDGE        = 0,
    DIRECTION_BOTTOM_EDGE     = 1,
    DIRECTION_LEFT_EDGE       = 2,
    DIRECTION_RIGHT_EDGE      = 3,
    DIRECTION_DISP_UP         = 4,
    DIRECTION_DISP_DOWN       = 5
} ;
#endif

typedef struct bma4_accel Accel;
typedef struct bma4_accel_config Acfg;

#endif

class StableBMA
{

public:
    StableBMA();
    ~StableBMA();

    bool begin(bma4_com_fptr_t readCallBlack, bma4_com_fptr_t writeCallBlack, bma4_delay_fptr_t delayCallBlack, uint8_t RTCType,
               uint8_t address = BMA4_I2C_ADDR_PRIMARY, uint8_t BMA423_INT1_PIN = 14, uint8_t BMA423_INT2_PIN = 12);  // Same as original but requires an RTCType and INT PINS from WatchyRTC or SmallRTC.

    void softReset();  // Same as original.
    void shutDown();   // Same as original.
    void wakeUp();     // Same as original.
    bool selfTest();   // Same as original.

    uint8_t getDirection();  // Same as original except it is orientated to show the proper higher edge on your Watchy.
    bool IsUp();             // Returns True if your Watchy is in the Tilt position (with flexible room).

    bool setAccelConfig(Acfg &cfg);    // Same as original.
    bool getAccelConfig(Acfg &cfg);    // Same as original.
    bool getAccel(Accel &acc);         // Same as original with the exception that it inverts the x and y axes on the necessary RTCType.
    bool getAccelEnable();             // Same as original.
    bool disableAccel();               // Same as original.
    bool enableAccel(bool en = true);  // Same as original.

    bool setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap);  // Same as original.
    bool getINT();  // Same as original.
    uint8_t getIRQMASK();  // Same as original.
    bool disableIRQ(uint16_t int_map = BMA423_STEP_CNTR_INT);  // Same as original.
    bool enableIRQ(uint16_t int_map = BMA423_STEP_CNTR_INT);   // Same as original.
    bool isStepCounter();  // Same as original.
    bool isDoubleClick(); // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isTilt();        // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isActivity();    // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isAnyNoMotion(); // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool didBMAWakeUp(uint64_t hwWakeup); // Allows you to tell via wakeupBit, if the BMA woke the Watchy, if it did, it reads the reason so you can use the above 4 functions.

    bool resetStepCounter();  // Same as original.
    uint32_t getCounter();    // Same as original.

    float readTemperature(bool Metric = true);  // Changed to support both temperatures.
    float readTemperatureF(); // Left for compatibility with older code.

    uint16_t getErrorCode();  // Same as original.
    uint16_t getStatus();     // Same as original.
    uint32_t getSensorTime(); // Same as original.

    const char *getActivity(); // Same as original.
    bool setRemapAxes(struct bma423_axes_remap *remap_data); // Same as original.

    bool enableFeature(uint8_t feature, uint8_t enable ); // Same as original.
    bool enableStepCountInterrupt(bool en = true);        // Same as original.
    bool enableTiltInterrupt(bool en = true);             // Same as original.
    bool enableWakeupInterrupt(bool en = true);           // Same as original.
    bool enableAnyNoMotionInterrupt(bool en = true);      // Same as original.
    bool enableActivityInterrupt(bool en = true);         // Same as original.
    uint32_t WakeMask();   // Returns the necessary value to OR in the esp_sleep_enable_ext1_wakeup function to request BMA wakeups to work.
    bool defaultConfig(bool LowPower = true);   // This is the default Configuration settings removed from Watchy::_bmaConfig(), corrected based on needs of RTCType.  _bmaConfig() should only consist of the begin() call and after that, the defaultConfig().
    bool enableDoubleClickWake(bool en = true); // Enables/Disables DoubleClick and the Wake Interrupt
    bool enableTiltWake(bool en = true);        // Enables/Disables Tilt and the Wake Interrupt

private:
    bma4_com_fptr_t __readRegisterFptr;
    bma4_com_fptr_t __writeRegisterFptr;
    bma4_delay_fptr_t __delayCallBlackFptr;

    uint8_t __address;
    uint8_t __RTCTYPE;
    uint16_t __IRQ_MASK;
    bool __init;
    struct bma4_dev __devFptr;
};
