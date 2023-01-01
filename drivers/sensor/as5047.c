/*
 * Copyright (c) 2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/spi.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util_macro.h>
#include <sys/__assert.h>

LOG_MODULE_REGISTER(as5047, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT ams_as5047

#define ADDR_NOP 0x0000
#define ADDR_ERRFL 0x0001
#define ADDR_PROG 0x0003
#define ADDR_DIAAGC 0x3FFC
#define ADDR_MAG 0x3FFD
#define ADDR_ANGLEUNC 0x3FFE
#define ADDR_ANGLECOM 0x3FFF

#define RW_WRITE 0
#define RW_READ 1

struct as5047_data {
	const struct device *dev;
	uint16_t val;
	int32_t delta;

	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trigger;

	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_AS5047_THREAD_STACK_SIZE);
	struct k_thread thread;
};

struct as5047_config {
	struct spi_dt_spec bus;
	uint32_t init_delay_us;
	uint32_t poll_interval_us;
	uint8_t precision_shift;
};

static inline uint8_t as5047_calc_parc(uint16_t command)
{
	uint8_t parc = 0;
	for (uint8_t i = 0; i < 16; i++) {
		if (command & 0x1)
			parc++;
		command >>= 1;
	}
	return parc & 0x1;
}

static int as5047_send_command(const struct device *dev, uint16_t addr, uint8_t rw, uint16_t *val)
{
	struct as5047_data *data = dev->data;
	const struct as5047_config *config = dev->config;

	int ret;
	uint16_t req, res = 0;

	req = (addr & BIT_MASK(14)) | ((rw & BIT_MASK(1)) << 14);
	req |= as5047_calc_parc(req) << 15;

	req = sys_cpu_to_be16(req);

	const struct spi_buf tx_buf = {
		.buf = &req,
		.len = sizeof(req),
	};
	const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1U,
	};

	const struct spi_buf rx_buf = {
		.buf = &res,
		.len = sizeof(res),
	};
	const struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1U,
	};

	ret = spi_transceive_dt(&config->bus, &tx_bufs, &rx_bufs);
	if (ret != 0) {
		return ret;
	}

	res = sys_be16_to_cpu(res);

	if ((res >> 14) & BIT_MASK(1)) {
		return EIO;
	}

	if ((res >> 15) != as5047_calc_parc(res & BIT_MASK(15))) {
		return EIO;
	}

	*val = res & BIT_MASK(14);

	return ret;
}

static int as5047_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	struct as5047_data *data = dev->data;
	data->trigger = trig;
	data->handler = handler;
	return 0;
}

static int as5047_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	return 0;
}

static int as5047_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct as5047_data *data = dev->data;

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	val->val1 = data->delta;
	val->val2 = 0;

	return 0;
}

static void as5047_thread(int dev_ptr, int unused)
{
	const struct device *dev = INT_TO_POINTER(dev_ptr);
	struct as5047_data *data = dev->data;
	const struct as5047_config *config = dev->config;
	uint16_t val = 0;

	ARG_UNUSED(unused);

	while (1) {
		as5047_send_command(dev, ADDR_ANGLECOM, RW_READ, &val);
		val >>= config->precision_shift;

		if (data->val != val) {
			LOG_DBG("sensor value: %d -> %d", data->val, val);
			data->delta = CLAMP(val - data->val, -1, 1);
			data->val = val;
			if (data->handler != NULL) {
				data->handler(dev, data->trigger);
			}
		}

		k_usleep(config->poll_interval_us);
	}
}

static const struct sensor_driver_api as5047_driver_api = {
	.trigger_set = as5047_trigger_set,
	.sample_fetch = as5047_sample_fetch,
	.channel_get = as5047_channel_get,
};

int as5047_init(const struct device *dev)
{
	struct as5047_data *data = dev->data;
	const struct as5047_config *config = dev->config;

	data->dev = dev;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("SPI bus is not ready: %s", config->bus.bus->name);
		return -ENODEV;
	}

	k_thread_create(&data->thread, data->thread_stack, CONFIG_AS5047_THREAD_STACK_SIZE,
			(k_thread_entry_t)as5047_thread, dev, 0, NULL,
			K_PRIO_COOP(CONFIG_AS5047_THREAD_PRIORITY), 0, K_NO_WAIT);

	if (config->init_delay_us > 0) {
		k_usleep(config->init_delay_us);
	}

	return 0;
}

#define AS5047_INST(n)                                                                             \
	struct as5047_data as5047_data_##n;                                                        \
	const struct as5047_config as5047_config_##n = {                                           \
		.bus = SPI_DT_SPEC_INST_GET(n,                                                     \
					    SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |                \
						    SPI_WORD_SET(8) | SPI_MODE_CPHA,               \
					    DT_INST_PROP_OR(n, cs_delay_us, 350)),                 \
		.init_delay_us = DT_INST_PROP_OR(n, init_delay_us, 0),                             \
		.poll_interval_us = DT_INST_PROP_OR(n, poll_interval_us, 10000),                   \
		.precision_shift = CLAMP(DT_INST_PROP_OR(n, precision_shift, 0), 0, 14),           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, as5047_init, NULL, &as5047_data_##n, &as5047_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &as5047_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047_INST)
