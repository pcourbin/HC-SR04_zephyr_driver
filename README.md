# Out of tree HC-SR04 sensor driver module for Zephyr RTOS
*This is a Zephyr RTOS out of tree driver for the [HC-SR04 Ultrasonic ranging module](https://www.sparkfun.com/products/24049)*

This work is based on:
- The great work of [inductivekickback](https://github.com/inductivekickback) on a driver of the HC-SR04 to a previous version of Zephyr RTOS. (see [here](https://github.com/inductivekickback/hc-sr04)).
- The perfect example of out of tree module/driver for Zephyr RTOS from [nobodyguy](https://github.com/nobodyguy) with the [HX711 driver](https://github.com/nobodyguy/HX711_zephyr_driver)
- [Useful documentation](https://docs.nordicsemi.com/bundle/ncs-2.1.3/page/zephyr/build/dts/howtos.html#option_1_create_devices_using_instance_numbers) on DTS to read multiple sensors of the same type.

## Supported and tested Zephyr versions
* 3.6.0 (July 2024)

## Usage
### Module installation for projects not using west modules directly
1) Create directory named `modules` inside project root directory.
2) Clone this repository into `modules` as `HC_SR04` directory
3) Edit `CMakeLists.txt` in your project's root directory:
```CMake
set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/modules/HC_SR04")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

To replace steps 1 and 2, e.g. to use in a `Platformio`project, you can create a `.gitmodules` file with:
```ini
[submodule "HC-SR04-Driver"]
	path = zephyr/modules/HC_SR04
	url = https://github.com/pcourbin/HC-SR04_zephyr_driver
```

### Driver configuration
Enable sensor driver subsystem and HC_SR04 driver by adding these entries to your `prj.conf`:
```ini
CONFIG_SENSOR=y
CONFIG_HC_SR04=y
```

Define HC_SR04 in your board `.overlay` like this example with two sensors:
```dts
/ {
    us0: hc-sr04_0 {
        compatible = "zephyr,hc-sr04";
        trig-gpios = <&gpiog 5 GPIO_ACTIVE_HIGH>;
        echo-gpios = <&gpiog 4 GPIO_ACTIVE_HIGH>;
        status = "okay";
    };

    us1: hc-sr04_1 {
        compatible = "zephyr,hc-sr04";
        trig-gpios = <&gpiog 2 GPIO_ACTIVE_HIGH>;
        echo-gpios = <&gpiog 3 GPIO_ACTIVE_HIGH>;
        status = "okay";
    };
};

```

### Driver usage
```c
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#include <sensor/hc_sr04/hc_sr04.h>

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

struct sensor_value measure(const struct device *dev)
{
	struct sensor_value distance = {-1, 0};
	int ret;

	ret = sensor_sample_fetch(dev);
	if (ret != 0)
	{
		LOG_ERR("%s - Cannot take measurement: %d", dev->name, ret);
	}
	else
	{
		sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
		LOG_INF("%s - Distance: %d.%06d cm", dev->name, distance.val1, distance.val2);
	}
	return distance;
}

int main(void)
{
	char text[50] = {0};

	const struct device *hc_sr04_dev0 = DEVICE_DT_GET(DT_NODELABEL(us0));
	__ASSERT(hc_sr04_dev0 == NULL, "Failed to get device binding");
	struct sensor_value distance0 = {0, 0};

	const struct device *hc_sr04_dev1 = DEVICE_DT_GET(DT_NODELABEL(us1));
	__ASSERT(hc_sr04_dev1 == NULL, "Failed to get device binding");
	struct sensor_value distance1 = {0, 0};

	while (1)
	{
		distance0 = measure(hc_sr04_dev0);
		distance1 = measure(hc_sr04_dev1);

		sprintf(text, "US0:%d.%02dcm \t US1:%d.%02dcm",
				distance0.val1, distance0.val2,
				distance1.val1, distance1.val2);
		printf("%s\n", text);
		k_msleep(1000);
	}
}
```
Relevant `prj.conf`:
```ini
CONFIG_SENSOR=y
CONFIG_HC_SR04=y
CONFIG_LOG=y
```