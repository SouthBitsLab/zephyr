/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "icg20660l.h"
#include "icg20660l_reg.h"

LOG_MODULE_DECLARE(ICG20660L, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Forward declaration of the interrupt handler dispatch function.
 * The actual ISR/callback only signals the processing context.
 */
static void icg20660l_handle_data_ready(const struct device *dev);

#if defined(CONFIG_ICG20660L_TRIGGER_OWN_THREAD)
/*
 * Dedicated driver thread: waits on a semaphore posted by the GPIO callback,
 * then invokes the user-registered handler.
 */
static void icg20660l_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct icg20660l_data *drv_data = dev->data;

	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		icg20660l_handle_data_ready(dev);
	}
}

#elif defined(CONFIG_ICG20660L_TRIGGER_GLOBAL_THREAD)
/*
 * Global work queue handler: runs in the shared system work queue after the
 * GPIO callback submits it.
 */
static void icg20660l_work_handler(struct k_work *work)
{
	struct icg20660l_data *drv_data =
		CONTAINER_OF(work, struct icg20660l_data, work);

	icg20660l_handle_data_ready(drv_data->dev);
}
#endif

/*
 * GPIO interrupt callback: the interrupt line toggled, so schedule the
 * deferred handler. The GPIO interrupt stays enabled; the semaphore count
 * absorbs bursts so no edge is lost while a previous sample is processed.
 */
static void icg20660l_gpio_callback(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	struct icg20660l_data *drv_data =
		CONTAINER_OF(cb, struct icg20660l_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

#if defined(CONFIG_ICG20660L_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ICG20660L_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

/*
 * Dispatch the registered data-ready handler.
 */
static void icg20660l_handle_data_ready(const struct device *dev)
{
	struct icg20660l_data *drv_data = dev->data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev, drv_data->data_ready_trigger);
	}
}

/*
 * Register or unregister the data-ready trigger handler.
 * Only SENSOR_TRIG_DATA_READY is supported.
 */
int icg20660l_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler)
{
	struct icg20660l_data *drv_data = dev->data;
	const struct icg20660l_config *cfg = dev->config;
	int ret;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	/* Disable the GPIO interrupt while updating the handler pointer. */
	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}

	drv_data->data_ready_handler = handler;
	drv_data->data_ready_trigger = trig;

	/* Re-enable the interrupt if a handler was registered. */
	if (handler != NULL) {
		ret = gpio_pin_interrupt_configure_dt(
			&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Initialize the data-ready interrupt path:
 *  - Configure the GPIO pin as input.
 *  - Enable the data-ready interrupt source inside the sensor.
 *  - Install and enable the GPIO callback.
 *  - Create the driver's own thread or initialize the work item.
 */
int icg20660l_init_interrupt(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;
	struct icg20660l_data *drv_data = dev->data;
	int ret;

	drv_data->dev = dev;

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("GPIO interrupt pin is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO interrupt pin: %d", ret);
		return ret;
	}

	/* Enable the data-ready interrupt output from the sensor. */
	ret = icg20660l_bus_write(dev, ICG20660L_REG_INT_ENABLE,
				ICG20660L_INT_ENABLE_DATA_RDY);
	if (ret < 0) {
		LOG_ERR("Failed to enable data-ready interrupt: %d", ret);
		return ret;
	}

	gpio_init_callback(&drv_data->gpio_cb, icg20660l_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &drv_data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

#if defined(CONFIG_ICG20660L_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			K_KERNEL_STACK_SIZEOF(drv_data->thread_stack),
			icg20660l_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICG20660L_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ICG20660L_TRIGGER_GLOBAL_THREAD)
	k_work_init(&drv_data->work, icg20660l_work_handler);
#endif

	return 0;
}
