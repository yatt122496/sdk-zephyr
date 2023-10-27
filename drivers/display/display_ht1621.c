/*
 * Copyright (c) 2022 Jimmy Ou <yanagiis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_ht1621

/**
 * @file
 * @brief HT1621 LED display driver
 *
 * This driver map the segment as x, digit as y.
 *
 * A HT1621 has 8x8 pixels.
 * Two HT1621s (with cascading) have 8x16 pixels.
 * So on and so forth.
 *
 * Datasheet: https://datasheets.maximintegrated.com/en/ds/HT1621-MAX7221.pdf
 *
 * Limitations:
 *  1. This driver only implements no-decode mode.
 */

#include <stddef.h>

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
LOG_MODULE_REGISTER(ht1621, CONFIG_DISPLAY_LOG_LEVEL);

#define HT1621_SEGMENTS_PER_DIGIT 8
#define HT1621_DIGITS_PER_DEVICE  17

/* clang-format off */

#define HT1621_CMD				0x80
#define HT1621_READ				0xC0
#define HT1621_WRITE			0xA0

#define HT1621_CMD_SYS_DIS		0x0
#define HT1621_CMD_SYS_EN		0x2
#define HT1621_CMD_LCD_OFF		0x4
#define HT1621_CMD_LCD_ON		0x6
#define HT1621_CMD_TIMERS_DIS	0x8
#define HT1621_CMD_WDT_DIS		0xA
#define HT1621_CMD_TIMERS_EN	0xC
#define HT1621_CMD_WDT_EN		0xE
#define HT1621_CMD_TONE_OFF		0x10
#define HT1621_CMD_TONE_ON		0x12
#define HT1621_CMD_CLR_TIMER	0x18
#define HT1621_CMD_CLR_WDT		0x1C
#define HT1621_CMD_XTAL_32K		0x28
#define HT1621_CMD_RC256K		0x30
#define HT1621_CMD_EXT_256K		0x38
#define HT1621_CMD_BIAS_1_2_COMS2	0x40
#define HT1621_CMD_BIAS_1_2_COMS3	0x48
#define HT1621_CMD_BIAS_1_2_COMS4	0x50
#define HT1621_CMD_BIAS_1_3_COMS2	0x42
#define HT1621_CMD_BIAS_1_3_COMS3	0x4A
#define HT1621_CMD_BIAS_1_3_COMS4	0x52
#define HT1621_CMD_TONE_4K		0x80
#define HT1621_CMD_TONE_2K		0xC0
#define HT1621_CMD_IRQ_DIS		0x100
#define HT1621_CMD_IRQ_EN		0x110
#define HT1621_CMD_F1			0x140
#define HT1621_CMD_F2			0x142
#define HT1621_CMD_F4			0x144
#define HT1621_CMD_F8			0x146
#define HT1621_CMD_F16			0x148
#define HT1621_CMD_F32			0x14A
#define HT1621_CMD_F64			0x14C
#define HT1621_CMD_F128			0x14E
#define HT1621_CMD_TEST			0x1C0
#define HT1621_CMD_NORMAL		0x1C6

#define HT1621_CMD_TEST_H16		0x148
/* HT1621 registers and fields */
#define HT1621_REG_NOOP		0x00
#define HT1621_NOOP			0x00

#define HT1621_REG_DECODE_MODE		0x09
#define HT1621_NO_DECODE		0x00

#define HT1621_REG_INTENSITY		0x0A

#define HT1621_REG_SCAN_LIMIT		0x0B

#define HT1621_REG_SHUTDOWN		0x0C
#define HT1621_SHUTDOWN_MODE		0x00
#define HT1621_LEAVE_SHUTDOWN_MODE	0x01

#define HT1621_REG_DISPLAY_TEST	0x0F
#define HT1621_LEAVE_DISPLAY_TEST_MODE	0x00
#define HT1621_DISPLAY_TEST_MODE	0x01

/* clang-format on */

struct ht1621_config {
	struct spi_dt_spec spi;
	uint32_t num_cascading;
	uint8_t intensity;
	uint8_t scan_limit;
};

struct ht1621_data {
	/* Keep all digit_buf for all cascading HT1621 */
	uint8_t *digit_buf;
	uint8_t *tx_buf;
};

static int ht1621_transmit_all(const struct device *dev, const uint8_t cmd, const uint16_t value)
{
	const struct ht1621_config *dev_config = dev->config;
	struct ht1621_data *dev_data = dev->data;

	const struct spi_buf tx_buf = {
		.buf = dev_data->tx_buf,
		.len = dev_config->num_cascading * 2,
	};
	const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1U,
	};

	for (int i = 0; i < dev_config->num_cascading; i++) {
		dev_data->tx_buf[i * 2] = cmd | (value >> 4);
		dev_data->tx_buf[i * 2 + 1] = (uint8_t)(value << 4);
	}

	return spi_write_dt(&dev_config->spi, &tx_bufs);
}

static int ht1621_write_all(const struct device *dev)
{
	const struct ht1621_config *dev_config = dev->config;
	struct ht1621_data *dev_data = dev->data;

	const struct spi_buf tx_buf = {
		.buf = dev_data->digit_buf,
		.len = dev_config->num_cascading * HT1621_DIGITS_PER_DEVICE * sizeof(uint8_t) + 1,
	};
	const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1U,
	};

	for (int i = 0; i < dev_config->num_cascading; i++) {
		dev_data->digit_buf[i * 2] = HT1621_WRITE | (0 >> 4);
		dev_data->digit_buf[i * 2 + 1] &= ~(1 << 7);
	}
	// LOG_HEXDUMP_INF(tx_buf.buf, tx_buf.len, "SPI WRITE=");

	return spi_write_dt(&dev_config->spi, &tx_bufs);
}

static int ht1621_transmit_one(const struct device *dev, const uint8_t ht1621_idx,
				const uint8_t addr, const uint8_t value)
{
	const struct ht1621_config *dev_config = dev->config;
	struct ht1621_data *dev_data = dev->data;

	const struct spi_buf tx_buf = {
		.buf = dev_data->tx_buf,
		.len = dev_config->num_cascading * 2,
	};
	const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1U,
	};

	for (int i = 0; i < dev_config->num_cascading; i++) {
		if (i != (dev_config->num_cascading - 1 - ht1621_idx)) {
			dev_data->tx_buf[i * 2] = HT1621_REG_NOOP;
			dev_data->tx_buf[i * 2 + 1] = HT1621_NOOP;
			continue;
		}

		dev_data->tx_buf[i * 2] = addr;
		dev_data->tx_buf[i * 2 + 1] = value;
	}

	return spi_write_dt(&dev_config->spi, &tx_bufs);
}

static inline uint8_t next_pixel(uint8_t *mask, uint8_t *data, const uint8_t **buf)
{
	*mask <<= 1;
	if (!*mask) {
		*mask = 0x01;
		*data = *(*buf)++;
	}
	return *data & *mask;
}

static inline void skip_pixel(uint8_t *mask, uint8_t *data, const uint8_t **buf, uint16_t count)
{
	while (count--) {
		next_pixel(mask, data, buf);
	}
}

static int ht1621_blanking_on(const struct device *dev)
{
	int ret;

	ret = ht1621_transmit_all(dev, HT1621_CMD, HT1621_CMD_LCD_OFF);
	if (ret < 0) {
		LOG_ERR("Failed to set LCD_OFF");
		return ret;
	}

	ret = ht1621_transmit_all(dev, HT1621_CMD, HT1621_CMD_SYS_DIS);
	if (ret < 0) {
		LOG_ERR("Failed to set SYS_DIS");
		return ret;
	}

	return -ENOTSUP;
}

static int ht1621_blanking_off(const struct device *dev)
{
	int ret;

	ret = ht1621_transmit_all(dev, HT1621_CMD, HT1621_CMD_SYS_EN);
	if (ret < 0) {
		LOG_ERR("Failed to set SYS_EN");
		return ret;
	}

	ret = ht1621_transmit_all(dev, HT1621_CMD, HT1621_CMD_BIAS_1_3_COMS4);
	if (ret < 0) {
		LOG_ERR("Failed to set BIAS 1/3, 4 COMS");
		return ret;
	}

	ret = ht1621_transmit_all(dev, HT1621_CMD, HT1621_CMD_LCD_ON);
	if (ret < 0) {
		LOG_ERR("Failed to set LCD_ON");
		return ret;
	}

	return -ENOTSUP;
}

static int ht1621_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ht1621_config *dev_config = dev->config;
	struct ht1621_data *dev_data = dev->data;

	const uint16_t max_width = HT1621_SEGMENTS_PER_DIGIT;
	const uint16_t max_height = dev_config->num_cascading * HT1621_DIGITS_PER_DEVICE;

	/*
	 * HT1621 only supports PIXEL_FORMAT_MONO01. 1 bit stands for 1 pixel.
	 */
	__ASSERT((desc->pitch * desc->height) <= (desc->buf_size * 8U), "Input buffer to small");
	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT(desc->pitch <= max_width, "Pitch in descriptor is larger than screen size");
	__ASSERT(desc->height <= max_height, "Height in descriptor is larger than screen size");
	__ASSERT(x + desc->pitch <= max_width,
		 "Writing outside screen boundaries in horizontal direction");
	__ASSERT(y + desc->height <= max_height,
		 "Writing outside screen boundaries in vertical direction");

	if (desc->width > desc->pitch || (desc->pitch * desc->height) > (desc->buf_size * 8U)) {
		return -EINVAL;
	}

	if ((x + desc->pitch) > max_width || (y + desc->height) > max_height) {
		return -EINVAL;
	}

	const uint16_t end_x = x + desc->width;
	const uint16_t end_y = y + desc->height;
	const uint8_t *byte_buf = buf;
	const uint16_t to_skip = desc->pitch - desc->width;
	uint8_t mask = 0;
	uint8_t data = 0;

	memcpy(dev_data->digit_buf + 1, byte_buf, desc->buf_size);

	ht1621_write_all(dev);

	return 0;
}

static int ht1621_read(const struct device *dev, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, void *buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(desc);
	ARG_UNUSED(buf);

	return -ENOTSUP;
}

static void *ht1621_get_framebuffer(const struct device *dev)
{
	ARG_UNUSED(dev);

	return NULL;
}

static int ht1621_set_brightness(const struct device *dev, const uint8_t brightness)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(brightness);

	return -ENOTSUP;
}

static int ht1621_set_contrast(const struct device *dev, const uint8_t contrast)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(contrast);

	return -ENOTSUP;
}

static int ht1621_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format format)
{
	ARG_UNUSED(dev);

	switch (format) {
	case PIXEL_FORMAT_MONO01:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int ht1621_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	ARG_UNUSED(dev);

	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static void ht1621_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	const struct ht1621_config *dev_config = dev->config;

	caps->x_resolution = HT1621_SEGMENTS_PER_DIGIT;
	caps->y_resolution = HT1621_DIGITS_PER_DEVICE * dev_config->num_cascading;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
	caps->screen_info = 0;
	caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static const struct display_driver_api ht1621_api = {
	.blanking_on = ht1621_blanking_on,
	.blanking_off = ht1621_blanking_off,
	.write = ht1621_write,
	.read = ht1621_read,
	.get_framebuffer = ht1621_get_framebuffer,
	.set_brightness = ht1621_set_brightness,
	.set_contrast = ht1621_set_contrast,
	.get_capabilities = ht1621_get_capabilities,
	.set_pixel_format = ht1621_set_pixel_format,
	.set_orientation = ht1621_set_orientation,
};

static int ht1621_init(const struct device *dev)
{
	const struct ht1621_config *dev_config = dev->config;
	struct ht1621_data *dev_data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&dev_config->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	/* turn off all leds */
	memset(dev_data->digit_buf, 0x0,
	       dev_config->num_cascading * HT1621_DIGITS_PER_DEVICE * sizeof(uint8_t) + 1);

	// ht1621_blanking_off(dev);

	const struct display_buffer_descriptor desc = {
		.buf_size = dev_config->num_cascading * HT1621_DIGITS_PER_DEVICE,
		.height = dev_config->num_cascading * HT1621_DIGITS_PER_DEVICE,
		.width = HT1621_SEGMENTS_PER_DIGIT,
		.pitch = HT1621_SEGMENTS_PER_DIGIT,
	};

	ret = ht1621_write(dev, 0, 0, &desc, dev_data->digit_buf);
	if (ret < 0) {
		return ret;
	}

	ht1621_blanking_on(dev);

	return 0;
}

#define DISPLAY_HT1621_INIT(n)                                                \
	static uint8_t ht1621_digit_data_##n[DT_INST_PROP(n, num_cascading) * \
					      HT1621_DIGITS_PER_DEVICE + 1];      \
	static uint8_t ht1621_tx_buf##n[DT_INST_PROP(n, num_cascading) * 2];  \
	static struct ht1621_data ht1621_data_##n = {                        \
		.digit_buf = ht1621_digit_data_##n,                           \
		.tx_buf = ht1621_tx_buf##n,                                   \
	};                                                                     \
	static const struct ht1621_config ht1621_config_##n = {              \
		.spi = SPI_DT_SPEC_INST_GET(                                   \
			n, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_WORD_SET(8U), 0U),         \
		.num_cascading = DT_INST_PROP(n, num_cascading),               \
		.intensity = DT_INST_PROP(n, intensity),                       \
		.scan_limit = DT_INST_PROP(n, scan_limit),                     \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(n, ht1621_init, NULL, &ht1621_data_##n,        \
			      &ht1621_config_##n, POST_KERNEL,                \
			      CONFIG_DISPLAY_INIT_PRIORITY, &ht1621_api);

DT_INST_FOREACH_STATUS_OKAY(DISPLAY_HT1621_INIT)
