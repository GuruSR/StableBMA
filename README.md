# StableBMA [![Arduino Lint](https://github.com/GuruSR/StableBMA/actions/workflows/main.yml/badge.svg)](https://github.com/GuruSR/StableBMA/actions/workflows/main.yml)
A fork of the original (https://github.com/lewisxhe/BMA423_Library) bma.h/.cpp reworked and upgraded for the Watchy.

This was done to provide a working model of the bma.h for Watchy, that works with either version (and future ones that require alteration).

The functionality of this is identical (with 1 change) to the original bma.h/bma.cpp.  The one change is it requires the RTC.rtcType included in the "begin(" function.

Other functions that were added/altered are:

```
bool IsUp();                // Returns True if your Watchy is in the Tilt position (with flexible room).
bool getAccel(Accel &acc);  // Same as original with the exception that it inverts the x and y axes on the necessary RTCType.
bool isDoubleClick();       // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
bool isTilt();              // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
bool isActivity();          // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
bool isAnyNoMotion();       // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
bool didBMAWakeUp(uint64_t hwWakeup); // Allows you to tell via wakeupBit, if the BMA woke the Watchy, if it did, 
                                         it reads the reason so you can use the above 4 functions.

uint32_t WakeMask();   // Returns the necessary value to OR in the esp_sleep_enable_ext1_wakeup function to request BMA wakeups.
bool defaultConfig();  // This is the default Configuration settings removed from Watchy::
                          _bmaConfig() corrected based on needs of RTCType.
                          _bmaConfig() should only consist of the begin() call and after that, the defaultConfig().
                          See example Usage.md for information on usage.

bool enableDoubleClickWake(bool en = true); // Enables/Disables DoubleClick and the Wake Interrupt
bool enableTiltWake(bool en = true);        // Enables/Disables Tilt and the Wake Interrupt
float readTemperature(bool Metric = true);  // Diferent from the original, 0C bug fixed and offers Imperial temperature.
float readTemperatureF();                   // Fixes 0 Celcius bug and also will return 0 if an error.
```
