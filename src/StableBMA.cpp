#include "StableBMA.h"

/* Forked from bma.h by GuruSR (https://www.github.com/GuruSR/StableBMA) 
 * This fork is to improve Watchy functionality based on board version (via RTCType).
 * Version 1.0, February  6, 2022
 *
 *
 * MIT License
 *
 * Copyright (c) 2022 GuruSR
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
*/

#define DEBUGPORT Serial
#ifdef DEBUGPORT
#define DEBUG(...)      DEBUGPORT.printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

StableBMA::StableBMA()
{
    __readRegisterFptr = nullptr;
    __writeRegisterFptr = nullptr;
    __delayCallBlackFptr = nullptr;
    __init = false;
}

StableBMA::~StableBMA()
{

}

bool StableBMA::begin(bma4_com_fptr_t readCallBlack,
                   bma4_com_fptr_t writeCallBlack,
                   bma4_delay_fptr_t delayCallBlack,
                   uint8_t RTCType,
                   uint8_t address)
{

    if (__init ||
            readCallBlack == nullptr ||
            writeCallBlack == nullptr ||
            delayCallBlack == nullptr ||
            RTCType == 0) {
        return true;
    }

    __readRegisterFptr = readCallBlack;
    __writeRegisterFptr = writeCallBlack;
    __delayCallBlackFptr = delayCallBlack;
	__RTCTYPE = RTCType;

    __devFptr.dev_addr        = address;
    __devFptr.interface       = BMA4_I2C_INTERFACE;
    __devFptr.bus_read        = readCallBlack;
    __devFptr.bus_write       = writeCallBlack;
    __devFptr.delay           = delayCallBlack;
    __devFptr.read_write_len  = 8;
    __devFptr.resolution      = 12;
    __devFptr.feature_len     = BMA423_FEATURE_SIZE;

    softReset();

    __delayCallBlackFptr(20);

    if (bma423_init(&__devFptr) != BMA4_OK) {
        DEBUG("BMA423 FAIL\n");
        return false;
    }

    if (bma423_write_config_file(&__devFptr) != BMA4_OK) {
        DEBUG("BMA423 Write Config FAIL\n");
        return false;
    }

    __init = true;

    return __init;
}

void StableBMA::softReset()
{
    uint8_t reg = BMA4_RESET_ADDR;
    __writeRegisterFptr(BMA4_I2C_ADDR_PRIMARY, BMA4_RESET_SET_MASK, &reg, 1);
}

void StableBMA::shutDown()
{
    bma4_set_advance_power_save(BMA4_DISABLE,  &__devFptr);
}

void StableBMA::wakeUp()
{
    bma4_set_advance_power_save(BMA4_ENABLE, &__devFptr);
}

uint16_t StableBMA::getErrorCode()
{
    struct bma4_err_reg err;
    uint16_t rslt = bma4_get_error_status(&err, &__devFptr);
    return rslt;
}

uint16_t StableBMA::getStatus()
{
    uint8_t status;
    bma4_get_status(&status, &__devFptr);
    return status;
}

uint32_t StableBMA::getSensorTime()
{
    uint32_t ms;
    bma4_get_sensor_time(&ms, &__devFptr);
    return ms;
}

bool StableBMA::selfTest()
{
    return (BMA4_OK == bma4_selftest_config(BMA4_ACCEL_SELFTEST_ENABLE_MSK, &__devFptr));
}

uint8_t StableBMA::getDirection()
{
    Accel acc;
    if (!getAccel(acc)) return 0;
    uint16_t absX = abs(acc.x);
    uint16_t absY = abs(acc.y);
    uint16_t absZ = abs(acc.z);

    if ((absZ > absX) && (absZ > absY)) {
        return ((acc.z > 0) ? DIRECTION_DISP_DOWN : DIRECTION_DISP_UP);
    } else if ((absY > absX) && (absY > absZ)) {
        return ((acc.y > 0) ? DIRECTION_LEFT_EDGE : DIRECTION_RIGHT_EDGE);
    } else {
        return ((acc.x < 0) ? DIRECTION_TOP_EDGE : DIRECTION_BOTTOM_EDGE);
    }
}

bool StableBMA::IsUp() {
    Accel acc;
    if (!getAccel(acc)) return false;
    return (acc.x <= 0 && acc.x >= -700 && acc.y >= -300 && acc.y <= 300 && acc.z <= -750 && acc.z >= -1070);
}

float StableBMA::readTemperature()
{
    int32_t data = 0;
    bma4_get_temperature(&data, BMA4_DEG, &__devFptr);
    float res = (float)data / (float)BMA4_SCALE_TEMP;
    /* 0x80 - temp read from the register and 23 is the ambient temp added.
     * If the temp read from register is 0x80, it means no valid
     * information is available */
    if (((data - 23) / BMA4_SCALE_TEMP) == 0x80) {
        res = 0;
    }
    return res;
}


float StableBMA::readTemperatureF()
{
    float temp = readTemperature();
    if (temp != 0) return (temp * 1.8 + 32.0);
    return 0;
}

bool StableBMA::getAccel(Accel &acc)
{
    memset(&acc, 0, sizeof(acc));
    if (bma4_read_accel_xyz(&acc, &__devFptr) != BMA4_OK) {
        return false;
    }
	if (__RTCTYPE != 1) { acc.x = -acc.x; acc.y = -acc.y; }
    return true;
}

bool StableBMA::getAccelEnable()
{
    uint8_t en;
    bma4_get_accel_enable(&en, &__devFptr);
    return (en & BMA4_ACCEL_ENABLE_POS) == BMA4_ACCEL_ENABLE_POS;
}

bool StableBMA::disableAccel()
{
    return enableAccel(false);
}

bool StableBMA::enableAccel(bool en)
{
    return (BMA4_OK == bma4_set_accel_enable(en ? BMA4_ENABLE : BMA4_DISABLE, &__devFptr));
}

bool StableBMA::setAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_set_accel_config(&cfg, &__devFptr));
}

bool StableBMA::getAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_get_accel_config(&cfg, &__devFptr));
}

bool StableBMA::setRemapAxes(struct bma423_axes_remap *remap_data)
{
    return (BMA4_OK == bma423_set_remap_axes(remap_data, &__devFptr));
}

bool StableBMA::resetStepCounter()
{
    return  BMA4_OK == bma423_reset_step_counter(&__devFptr) ;
}

uint32_t StableBMA::getCounter()
{
    uint32_t stepCount;
    if (bma423_step_counter_output(&stepCount, &__devFptr) == BMA4_OK) {
        return stepCount;
    }
    return 0;
}

bool StableBMA::setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap)
{
    return BMA4_OK == bma4_set_int_pin_config(&config, pinMap, &__devFptr);
}

bool StableBMA::getINT()
{
    return bma423_read_int_status(&__IRQ_MASK, &__devFptr) == BMA4_OK;
}

uint8_t StableBMA::getIRQMASK()
{
    return __IRQ_MASK;
}

bool StableBMA::disableIRQ(uint16_t int_map)
{
    return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &__devFptr));
}

bool StableBMA::enableIRQ(uint16_t int_map)
{
    return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &__devFptr));
}

bool StableBMA::enableFeature(uint8_t feature, uint8_t enable)
{
    if ((feature & BMA423_STEP_CNTR) == BMA423_STEP_CNTR) {
        bma423_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &__devFptr);
    }
    return (BMA4_OK == bma423_feature_enable(feature, enable, &__devFptr));
}

bool StableBMA::isStepCounter()
{
    return (bool)(BMA423_STEP_CNTR_INT & __IRQ_MASK);
}

bool StableBMA::isDoubleClick()
{
    return (bool)(BMA423_WAKEUP_INT & __IRQ_MASK);
}

bool StableBMA::isTilt()
{
    return (bool)(BMA423_TILT_INT & __IRQ_MASK);
}

bool StableBMA::isActivity()
{
    return (bool)(BMA423_ACTIVITY_INT & __IRQ_MASK);
}

bool StableBMA::isAnyNoMotion()
{
    return (bool)(BMA423_ANY_NO_MOTION_INT & __IRQ_MASK);
}

bool StableBMA::didBMAWakeUp(uint64_t hwWakeup)
{
    bool B =((hwWakeup & BMA423x_INT1_MASK) || (hwWakeup & BMA423x_INT2_MASK));
	if (!B) return B;
    if (getINT()) return B;
	return false;
}


bool StableBMA::enableStepCountInterrupt(bool en)
{
    return  (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP,  BMA423_STEP_CNTR_INT, en, &__devFptr));
}

bool StableBMA::enableTiltInterrupt(bool en)
{
    return  (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_TILT_INT, en, &__devFptr));
}

bool StableBMA::enableWakeupInterrupt(bool en)
{
    return  (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_WAKEUP_INT, en, &__devFptr));
}

bool StableBMA::enableAnyNoMotionInterrupt(bool en)
{
    return  (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ANY_NO_MOTION_INT, en, &__devFptr));
}

bool StableBMA::enableActivityInterrupt(bool en)
{
    return  (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ACTIVITY_INT, en, &__devFptr));
}

const char *StableBMA::getActivity()
{
    uint8_t activity;
    bma423_activity_output(&activity, &__devFptr);
    if (activity & BMA423_USER_STATIONARY) {
        return "BMA423_USER_STATIONARY";
    } else if (activity & BMA423_USER_WALKING) {
        return "BMA423_USER_WALKING";
    } else if (activity & BMA423_USER_RUNNING) {
        return "BMA423_USER_RUNNING";
    } else if (activity & BMA423_STATE_INVALID) {
        return "BMA423_STATE_INVALID";
    }
    return "None";
}

uint32_t StableBMA::WakeMask()
{
    return (BMA423x_INT1_MASK | BMA423x_INT2_MASK);
}

bool StableBMA::defaultConfig()
{
    struct bma4_int_pin_config config ;
    config.edge_ctrl = BMA4_LEVEL_TRIGGER;
    config.lvl = BMA4_ACTIVE_HIGH;
    config.od = BMA4_PUSH_PULL;
    config.output_en = BMA4_OUTPUT_ENABLE;
    config.input_en = BMA4_INPUT_DISABLE;


    if (bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &__devFptr) != BMA4_OK) {
        DEBUG("BMA423 DEF CFG FAIL\n");
        return false;
    }
    Acfg cfg;
    cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    cfg.range = BMA4_ACCEL_RANGE_2G;
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    cfg.perf_mode = BMA4_CONTINUOUS_MODE;
    if (setAccelConfig(cfg)){
        if (enableAccel()){
            setINTPinConfig(config, BMA4_INTR1_MAP);

            struct bma423_axes_remap remap_data;
            remap_data.x_axis = 1;
            remap_data.x_axis_sign = (__RTCTYPE == 1 ? 1 : 0);
            remap_data.y_axis = 0;
            remap_data.y_axis_sign = (__RTCTYPE == 1 ? 1 : 0);
            remap_data.z_axis = 2;
            remap_data.z_axis_sign = 1;
            return setRemapAxes(&remap_data);
        }
    }
    return false;
}

bool StableBMA::enableDoubleClickWake(bool en)
{
	if (enableFeature(BMA423_WAKEUP,en)) return enableWakeupInterrupt(en);
	return false;
}

bool StableBMA::enableTiltWake(bool en)
{
    if (enableFeature(BMA423_TILT,en)) return enableTiltInterrupt(en);
	return false;
}
