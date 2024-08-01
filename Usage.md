Download StableBMA.h & .cpp into a library folder of the same name.

**NOTE:  Starting with StableBMA 1.6, the begin function has changed, see example below.**

To use this instead of bma.h, EDIT your Watchy.h:

Find:
```#include "bma.h"```

it should read:
```#include <StableBMA.h>```

Find:
```BMA423 sensor```

it should read:
```StableBMA sensor```

Save the file, now edit Watchy.cpp.

Find:
```BMA423 sensor```

it should read:
```StableBMA sensor```

Find:
```void Watchy::_bmaConfig()```

REPLACE the entire function with:  (Items in [] are optional.)
```
void Watchy::_bmaConfig() {

  if (sensor.begin(_readRegister, _writeRegister, delay, RTC.getType(),BMA4_I2C_ADDR_PRIMARY,[high/low interrupt],[interrupt1 pin]) == false) {
    //fail to init BMA
    return;
  }

  if (!sensor.defaultConfig()) return;  // Failed.
  // Enable BMA423 isStepCounter feature
  sensor.enableFeature(BMA423_STEP_CNTR, true);
  // Enable isTilt feature
  //sensor.enableTiltWake();
  // Enable isDoubleClick feature
  //sensor.enableDoubleClickWake();

  // Reset steps
  //sensor.resetStepCounter();

  // Turn on feature interrupt
  //sensor.enableStepCountInterrupt();
}
```

To receive Tilt or Double Tap (Double Click) events:

In the ```switch (wakeup_reason)``` under ```ESP_SLEEP_WAKEUP_EXT1```:

Add the following:
```
if (sensor.didBMAWakeUp(wakeupBit))
{
    if (sensor.isTilt())
    {
       // Do tilt stuff here.
    } else if (sensor.isDoubleClick())
    {
       // Do Double Tap stuff here.
    }
}
```

In your ```deepSleep()``` function:

```
sensor.enableTiltWake([{true}/false]);         // true by default, optional [true/false].
sensor.enableDoubleClickWake([{true}/false]);
```

And alter your ```esp_sleep_enable_ext1_wakeup``` function, place ```sensor.WakeMask() |``` in front of ```BTN_PIN_MASK``` if you want the ESP32 to wake when either of the two interupts happen.

IE:

```
esp_sleep_enable_ext1_wakeup(sensor.WakeMask() | BTN_PIN_MASK, ESP_EXT1_WAKEUP_ANY_HIGH);
```

Only include the sensor.WakeMask() if you want either of the Tilt or DoubleClick wakes to happen.
