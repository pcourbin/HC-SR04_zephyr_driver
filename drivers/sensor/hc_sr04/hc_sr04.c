/*
 * Copyright (c) 2024 Pierre COURBIN
 * Based on https://github.com/inductivekickback/hc-sr04, Copyright (c) 2020 Daniel Veilleux
 */

/*
 * NOTE: Invalid measurements manifest as a 128600us pulse followed by a second pulse of ~6us
 *       about 145us later. This pulse can't be truncated so it effectively reduces the sensor's
 *       working rate.
 */

#define DT_DRV_COMPAT zephyr_hc_sr04

#include "hc_sr04.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(HC_SR04, CONFIG_SENSOR_LOG_LEVEL);

static void hc_sr04_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct hc_sr04_data *p_data = CONTAINER_OF(cb, struct hc_sr04_data, echo_gpio_cb);

    switch (p_data->state)
    {
    case HC_SR04_STATE_RISING_EDGE:
        p_data->start_time = k_cycle_get_32();
        p_data->state = HC_SR04_STATE_FALLING_EDGE;
        break;
    case HC_SR04_STATE_FALLING_EDGE:
        p_data->end_time = k_cycle_get_32();
        (void)gpio_remove_callback(dev, cb);
        p_data->state = HC_SR04_STATE_FINISHED;
        k_sem_give(&p_data->echo_sem);
        break;
    default:
        (void)gpio_remove_callback(dev, cb);
        p_data->state = HC_SR04_STATE_ERROR;
        break;
    }
}

static int hc_sr04_init(const struct device *dev)
{
    LOG_DBG("Initialising HC_SR04\n");

    int err, ret = 0;

    struct hc_sr04_data *p_data = dev->data;
    const struct hc_sr04_config *p_cfg = dev->config;

    p_data->distance.val1 = 0;
    p_data->distance.val2 = 0;

    /* Configure TRIG as output, LOW */
    p_data->trig_gpio = p_cfg->trig_ctrl;
    LOG_DBG("TRIG pin controller is %p, name is %s\n", p_data->trig_gpio, p_data->trig_gpio->name);

    ret = gpio_pin_configure(p_data->trig_gpio, p_cfg->trig_pin, GPIO_OUTPUT_INACTIVE | p_cfg->trig_flags);
    if (ret != 0)
    {
        return ret;
    }

    /* Configure ECHO as input */
    p_data->echo_gpio = p_cfg->echo_ctrl;
    LOG_DBG("ECHO pin controller is %p, name is %s\n", p_data->echo_gpio, p_data->echo_gpio->name);

    ret = gpio_pin_configure(p_data->echo_gpio, p_cfg->echo_pin, GPIO_INPUT | p_cfg->echo_flags);
    if (ret != 0)
    {
        return ret;
    }
    err = gpio_pin_interrupt_configure(p_data->echo_gpio,
                                       p_cfg->echo_pin,
                                       GPIO_INT_EDGE_BOTH);
    if (err != 0)
    {
        return err;
    }
    gpio_init_callback(&p_data->echo_gpio_cb, hc_sr04_gpio_callback, BIT(p_cfg->echo_pin));

    if (p_data->ready)
    {
        /* Already initialized */
        return 0;
    }

    err = k_sem_init(&p_data->echo_sem, 0, 1);
    if (0 != err)
    {
        return err;
    }
    err = k_mutex_init(&p_data->mutex);
    if (0 != err)
    {
        return err;
    }

    p_data->state = HC_SR04_STATE_IDLE;
    p_data->ready = true;
    return 0;
}

static int hc_sr04_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int err;
    uint32_t count;

    struct hc_sr04_data *p_data = dev->data;
    const struct hc_sr04_config *p_cfg = dev->config;

    if (unlikely((SENSOR_CHAN_ALL != chan) && (SENSOR_CHAN_DISTANCE != chan)))
    {
        return -ENOTSUP;
    }

    if (unlikely(!p_data->ready))
    {
        LOG_ERR("Driver is not initialized yet");
        return -EBUSY;
    }

    err = k_mutex_lock(&p_data->mutex, K_FOREVER);
    if (0 != err)
    {
        return err;
    }

    err = gpio_add_callback(p_data->echo_gpio, &p_data->echo_gpio_cb);
    if (0 != err)
    {
        LOG_DBG("Failed to add HC-SR04 echo callback");
        (void)k_mutex_unlock(&p_data->mutex);
        return -EIO;
    }

    p_data->state = HC_SR04_STATE_RISING_EDGE;
    gpio_pin_set(p_data->trig_gpio, p_cfg->trig_pin, 1);
    k_busy_wait(T_TRIG_PULSE_US);
    gpio_pin_set(p_data->trig_gpio, p_cfg->trig_pin, 0);

    if (0 != k_sem_take(&p_data->echo_sem, K_MSEC(T_MAX_WAIT_MS)))
    {
        LOG_DBG("No response from HC-SR04");
        (void)k_mutex_unlock(&p_data->mutex);
        err = gpio_remove_callback(p_data->echo_gpio, &p_data->echo_gpio_cb);
        if (0 != err)
        {
            return err;
        }
        return -EIO;
    }

    __ASSERT_NO_MSG(HC_SR04_STATE_FINISHED == p_data->state);

    if (p_data->start_time <= p_data->end_time)
    {
        count = (p_data->end_time - p_data->start_time);
    }
    else
    {
        count = (0xFFFFFFFF - p_data->start_time);
        count += p_data->end_time;
    }
    /* Convert from ticks to nanoseconds and then to microseconds */
    count = k_cyc_to_us_near32(count);
    if ((T_INVALID_PULSE_US > count) && (T_TRIG_PULSE_US < count))
    {
        /* Convert to centimeters and divide round-trip distance by two */
        count = (count * METERS_PER_SEC / 2);
        p_data->distance.val2 = (count % 10000);
        p_data->distance.val1 = (count / 10000);
    }
    else
    {
        LOG_INF("Invalid measurement");
        p_data->distance.val1 = 0;
        p_data->distance.val2 = 0;
        k_usleep(T_SPURIOS_WAIT_US);
    }

    err = k_mutex_unlock(&p_data->mutex);
    if (0 != err)
    {
        return err;
    }
    return 0;
}

static int hc_sr04_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct hc_sr04_data *p_data = dev->data;

    if (unlikely(!p_data->ready))
    {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    switch (chan)
    {
    case SENSOR_CHAN_DISTANCE:
        val->val2 = p_data->distance.val2;
        val->val1 = p_data->distance.val1;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api hc_sr04_api = {
    .sample_fetch = hc_sr04_sample_fetch,
    .channel_get = hc_sr04_channel_get,
};

#define INST(num) DT_INST(num, zephyr_hc_sr04)

#define HC_SR04_DEVICE(n)                                                                          \
    static const struct hc_sr04_config hc_sr04_config_##n = {                                      \
        .trig_pin = DT_INST_GPIO_PIN(n, trig_gpios),                                               \
        .trig_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(n), trig_gpios)),                      \
        .trig_flags = DT_INST_GPIO_FLAGS(n, trig_gpios),                                           \
        .echo_pin = DT_INST_GPIO_PIN(n, echo_gpios),                                               \
        .echo_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(n), echo_gpios)),                      \
        .echo_flags = DT_INST_GPIO_FLAGS(n, echo_gpios),                                           \
    };                                                                                             \
    static struct hc_sr04_data hc_sr04_data_##n = {                                                \
        .state = HC_SR04_STATE_IDLE,                                                               \
        .distance = {                                                                              \
            .val1 = 0,                                                                             \
            .val2 = 0,                                                                             \
        },                                                                                         \
        .start_time = 0,                                                                           \
        .end_time = 0,                                                                             \
        .ready = false,                                                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n,                                                                       \
                          hc_sr04_init, NULL, &hc_sr04_data_##n, &hc_sr04_config_##n, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &hc_sr04_api);

DT_INST_FOREACH_STATUS_OKAY(HC_SR04_DEVICE)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "HC_SR04 driver enabled without any devices"
#endif