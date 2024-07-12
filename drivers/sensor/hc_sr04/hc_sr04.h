/*
 * Copyright (c) 2024 Pierre COURBIN
 * Based on https://github.com/inductivekickback/hc-sr04, Copyright (c) 2020 Daniel Veilleux
 */

#ifndef ZEPHYR_DRIVERS_HC_SR04_H_
#define ZEPHYR_DRIVERS_HC_SR04_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/* Timings defined by spec */
#define T_TRIG_PULSE_US 11
#define T_INVALID_PULSE_US 25000
#define T_MAX_WAIT_MS 130
#define T_SPURIOS_WAIT_US 145
#define METERS_PER_SEC 340

    enum hc_sr04_state
    {
        HC_SR04_STATE_IDLE,
        HC_SR04_STATE_RISING_EDGE,
        HC_SR04_STATE_FALLING_EDGE,
        HC_SR04_STATE_FINISHED,
        HC_SR04_STATE_ERROR,
        HC_SR04_STATE_COUNT
    };

    struct hc_sr04_data
    {
        const struct device *dev;
        const struct device *trig_gpio;
        const struct device *echo_gpio;
        struct gpio_callback echo_gpio_cb;

        struct k_sem echo_sem;
        struct k_mutex mutex;

        enum hc_sr04_state state;
        bool ready; /* The module has been initialized */

        uint32_t start_time;
        uint32_t end_time;

        struct sensor_value distance;
    };

    struct hc_sr04_config
    {
        gpio_pin_t trig_pin;
        const struct device *trig_ctrl;
        gpio_dt_flags_t trig_flags;

        gpio_pin_t echo_pin;
        const struct device *echo_ctrl;
        gpio_dt_flags_t echo_flags;
    };

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_HC_SR04_H_ */
