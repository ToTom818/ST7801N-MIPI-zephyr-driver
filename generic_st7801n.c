/*
 * Copyright (c) 2026 TOTOM818
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sitronix_st7801n

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/dt-bindings/mipi_dsi/mipi_dsi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <string.h>

LOG_MODULE_REGISTER(st7801n, CONFIG_DISPLAY_LOG_LEVEL);

/* ST7801N DCS commands */
#define ST7801N_CMD_SLPIN           0x10U
#define ST7801N_CMD_SLPOUT          0x11U
#define ST7801N_CMD_INVOFF          0x20U
#define ST7801N_CMD_INVON           0x21U
#define ST7801N_CMD_DISPOFF         0x28U
#define ST7801N_CMD_DISPON          0x29U
#define ST7801N_CMD_CASET           0x2AU
#define ST7801N_CMD_RASET           0x2BU
#define ST7801N_CMD_RAMWR           0x2CU
#define ST7801N_CMD_RAMWRC          0x3CU
#define ST7801N_CMD_TEOFF           0x34U
#define ST7801N_CMD_TEON            0x35U
#define ST7801N_CMD_MADCTR          0x36U
#define ST7801N_CMD_COLMOD          0x3AU
#define ST7801N_CMD_WRDISBV         0x51U

/* COLMOD values */
#define ST7801N_PIXFMT_RGB565       0x05U
#define ST7801N_PIXFMT_RGB666       0x06U
#define ST7801N_PIXFMT_RGB888       0x07U

/* TEON parameter */
#define ST7801N_TEON_MODE1          0x00U /* only V-blanking */
#define ST7801N_TEON_MODE2          0x01U /* V+H blanking */

#define ST7801N_RESET_ASSERT_MS     1U
#define ST7801N_RESET_WAIT_MS       2U
#define ST7801N_SLPOUT_DELAY_MS     120U
#define ST7801N_SLPIN_DELAY_MS      120U

/* Keep firmware memory use bounded if runtime allocation is enabled. */
#define ST7801N_MAX_TRANSFER_CHUNK  65534U

#define ST7801N_TE_MODE_TO_PARAM(inst) \
	(DT_INST_NODE_HAS_PROP(inst, te_mode) ? \
		(DT_INST_ENUM_IDX(inst, te_mode) == 0 ? ST7801N_TEON_MODE1 : ST7801N_TEON_MODE2) : 0U)

struct st7801n_config {
	const struct device *mipi_dsi;
	uint8_t channel;
	uint8_t data_lanes;
	uint32_t dsi_pixfmt;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec te_gpio;
	struct regulator_dt_spec vddi_reg;
	struct regulator_dt_spec vci_reg;
	uint16_t width;
	uint16_t height;
	uint8_t madctr;
	bool invert_display;
	uint8_t te_param;
	uint32_t max_dsi_payload; /* includes one command byte */
};

struct st7801n_data {
	struct k_mutex lock;
	bool powered_on;
	bool blanking_on;
	bool dsi_attached;
	uint8_t brightness;
	uint8_t colmod;
	uint8_t bytes_per_pixel;
	struct mipi_dsi_device mdev;
#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
	uint8_t tmp_buf[CONFIG_ST7801N_STATIC_BUF_SIZE];
#endif
};

static int st7801n_dcs_write(const struct device *dev, uint8_t cmd, const uint8_t *params, size_t len)
{
	const struct st7801n_config *cfg = dev->config;
	ssize_t ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, params, len);
	if (ret < 0) {
		LOG_ERR("DCS cmd 0x%02x failed (%d)", cmd, (int)ret);
		return (int)ret;
	}

	return 0;
}

static int st7801n_transfer_pixels(const struct device *dev, bool first_cmd,
				   const uint8_t *payload, size_t len)
{
	const struct st7801n_config *cfg = dev->config;
	/* In Zephyr MIPI-DSI API, DCS command is carried in msg.cmd. */
	struct mipi_dsi_msg msg = {
		.type = MIPI_DSI_DCS_LONG_WRITE,
		.cmd = first_cmd ? ST7801N_CMD_RAMWR : ST7801N_CMD_RAMWRC,
		.tx_buf = payload,
		.tx_len = len,
	};
	ssize_t ret;

	ret = mipi_dsi_transfer(cfg->mipi_dsi, cfg->channel, &msg);
	if (ret < 0) {
		LOG_ERR("Pixel transfer failed (%d)", (int)ret);
		return (int)ret;
	}
	if ((size_t)ret != len) {
		LOG_ERR("Short pixel transfer (%d/%u)", (int)ret, (unsigned int)len);
		return -EIO;
	}

	return 0;
}

static int st7801n_map_dsi_pixfmt(uint32_t dsi_pixfmt, uint8_t *colmod, uint8_t *bpp)
{
	switch (dsi_pixfmt) {
	case MIPI_DSI_PIXFMT_RGB565:
		*colmod = ST7801N_PIXFMT_RGB565;
		*bpp = 2U;
		return 0;
	case MIPI_DSI_PIXFMT_RGB666:
		*colmod = ST7801N_PIXFMT_RGB666;
		*bpp = 3U;
		return 0;
	case MIPI_DSI_PIXFMT_RGB888:
		*colmod = ST7801N_PIXFMT_RGB888;
		*bpp = 3U;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int st7801n_attach_dsi(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	if (data->dsi_attached) {
		return 0;
	}

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &data->mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed (%d)", ret);
		return ret;
	}

	data->dsi_attached = true;
	return 0;
}

static int st7801n_detach_dsi(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	if (!data->dsi_attached) {
		return 0;
	}

	ret = mipi_dsi_detach(cfg->mipi_dsi, cfg->channel, &data->mdev);
	if (ret == -ENOSYS) {
		/* Host does not support detach; treat link as still attached. */
		return 0;
	}
	if (ret < 0) {
		LOG_WRN("MIPI-DSI detach failed (%d)", ret);
		return ret;
	}

	data->dsi_attached = false;
	return 0;
}

static int st7801n_reset_panel(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	int ret;

	if (cfg->reset_gpio.port == NULL) {
		return 0;
	}

	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret < 0) {
		return ret;
	}
	k_msleep(ST7801N_RESET_ASSERT_MS);

	ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
	if (ret < 0) {
		return ret;
	}
	k_msleep(ST7801N_RESET_WAIT_MS);

	return 0;
}

static int st7801n_hw_init(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	ret = st7801n_reset_panel(dev);
	if (ret < 0) {
		LOG_ERR("Panel reset failed (%d)", ret);
		return ret;
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_SLPOUT, NULL, 0);
	if (ret < 0) {
		return ret;
	}
	k_msleep(ST7801N_SLPOUT_DELAY_MS);

	ret = st7801n_dcs_write(dev, ST7801N_CMD_COLMOD, &data->colmod, 1);
	if (ret < 0) {
		return ret;
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_MADCTR, &cfg->madctr, 1);
	if (ret < 0) {
		return ret;
	}

	if (cfg->invert_display) {
		ret = st7801n_dcs_write(dev, ST7801N_CMD_INVON, NULL, 0);
	} else {
		ret = st7801n_dcs_write(dev, ST7801N_CMD_INVOFF, NULL, 0);
	}
	if (ret < 0) {
		return ret;
	}

	if (cfg->te_param != 0U) {
		uint8_t te_param = cfg->te_param;

		ret = st7801n_dcs_write(dev, ST7801N_CMD_TEON, &te_param, 1);
		if (ret < 0) {
			LOG_WRN("TEON failed (%d)", ret);
		}
	} else {
		(void)st7801n_dcs_write(dev, ST7801N_CMD_TEOFF, NULL, 0);
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_DISPON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_WRDISBV, &data->brightness, 1);
	if (ret < 0) {
		LOG_WRN("Brightness set failed during init (%d)", ret);
	}

	data->powered_on = true;
	data->blanking_on = false;
	return 0;
}

static int st7801n_blanking_on(const struct device *dev)
{
	struct st7801n_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	if (!data->blanking_on) {
		ret = st7801n_dcs_write(dev, ST7801N_CMD_DISPOFF, NULL, 0);
		if (ret == 0) {
			data->blanking_on = true;
		}
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_blanking_off(const struct device *dev)
{
	struct st7801n_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	if (data->blanking_on) {
		ret = st7801n_dcs_write(dev, ST7801N_CMD_DISPON, NULL, 0);
		if (ret == 0) {
			data->blanking_on = false;
		}
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_write(const struct device *dev, uint16_t x, uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	uint8_t col_params[4];
	uint8_t row_params[4];
	size_t bytes_per_row;
	size_t req_buf_size;
	size_t max_payload_bytes;
	size_t max_rows_per_packet;
	size_t rows_remaining;
	size_t row_offset;
	bool first_cmd = true;
	const uint8_t *src = buf;
	uint8_t bpp;
	int ret = 0;

	if ((desc == NULL) || (buf == NULL)) {
		return -EINVAL;
	}

	if ((desc->width == 0U) || (desc->height == 0U)) {
		return -EINVAL;
	}

	if (desc->pitch < desc->width) {
		return -EINVAL;
	}

	if ((desc->width % 2 != 0) || (desc->height % 2 != 0)) {
		return -EINVAL;
	}

	if ((x + desc->width > cfg->width) || (y + desc->height > cfg->height)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	bpp = data->bytes_per_pixel;
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	bytes_per_row = (size_t)desc->width * bpp;
	req_buf_size = (size_t)desc->pitch * desc->height * bpp;
	if (desc->buf_size < req_buf_size) {
		ret = -EINVAL;
		goto out;
	}

	if (cfg->max_dsi_payload < 2U) {
		ret = -EINVAL;
		goto out;
	}

	max_payload_bytes = cfg->max_dsi_payload - 1U;
	if ((max_payload_bytes == 0U) || (max_payload_bytes > ST7801N_MAX_TRANSFER_CHUNK)) {
		ret = -EINVAL;
		goto out;
	}

	max_rows_per_packet = max_payload_bytes / bytes_per_row;
	if (max_rows_per_packet == 0U) {
		ret = -EINVAL;
		goto out;
	}

	col_params[0] = (uint8_t)(x >> 8);
	col_params[1] = (uint8_t)(x & 0xFFU);
	col_params[2] = (uint8_t)((x + desc->width - 1U) >> 8);
	col_params[3] = (uint8_t)((x + desc->width - 1U) & 0xFFU);
	ret = st7801n_dcs_write(dev, ST7801N_CMD_CASET, col_params, sizeof(col_params));
	if (ret < 0) {
		goto out;
	}

	row_params[0] = (uint8_t)(y >> 8);
	row_params[1] = (uint8_t)(y & 0xFFU);
	row_params[2] = (uint8_t)((y + desc->height - 1U) >> 8);
	row_params[3] = (uint8_t)((y + desc->height - 1U) & 0xFFU);
	ret = st7801n_dcs_write(dev, ST7801N_CMD_RASET, row_params, sizeof(row_params));
	if (ret < 0) {
		goto out;
	}

	rows_remaining = desc->height;
	row_offset = 0U;

	while (rows_remaining > 0U) {
		size_t rows_this_packet = MIN(rows_remaining, max_rows_per_packet);
		size_t data_bytes_this_packet = rows_this_packet * bytes_per_row;
		uint8_t *tmp_buf;

#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
		if (data_bytes_this_packet > CONFIG_ST7801N_STATIC_BUF_SIZE) {
			ret = -ENOMEM;
			break;
		}
		tmp_buf = data->tmp_buf;
#else
		tmp_buf = k_malloc(data_bytes_this_packet);
		if (tmp_buf == NULL) {
			ret = -ENOMEM;
			break;
		}
#endif

		for (size_t r = 0U; r < rows_this_packet; r++) {
			size_t src_row = row_offset + r;

			memcpy(tmp_buf + (r * bytes_per_row),
			       src + (src_row * desc->pitch * bpp),
			       bytes_per_row);
		}

		ret = st7801n_transfer_pixels(dev, first_cmd, tmp_buf, data_bytes_this_packet);

#ifndef CONFIG_ST7801N_USE_STATIC_BUFFER
		k_free(tmp_buf);
#endif

		if (ret < 0) {
			break;
		}

		first_cmd = false;
		row_offset += rows_this_packet;
		rows_remaining -= rows_this_packet;
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_brightness(const struct device *dev, uint8_t brightness)
{
	struct st7801n_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_WRDISBV, &brightness, 1);
	if (ret == 0) {
		data->brightness = brightness;
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_contrast(const struct device *dev, uint8_t contrast)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(contrast);
	return -ENOTSUP;
}

static void st7801n_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;

	memset(capabilities, 0, sizeof(*capabilities));
	capabilities->x_resolution = cfg->width;
	capabilities->y_resolution = cfg->height;
	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_RGB_888 | PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_666;
	capabilities->screen_info = 0;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (data->colmod) {
	case ST7801N_PIXFMT_RGB565:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
		break;
	case ST7801N_PIXFMT_RGB666:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_666;
		break;
	case ST7801N_PIXFMT_RGB888:
	default:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
		break;
	}
	k_mutex_unlock(&data->lock);
}

static int st7801n_set_pixel_format(const struct device *dev, enum display_pixel_format pixel_format)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	uint8_t colmod;
	uint8_t bpp;
	int ret = 0;
	uint32_t requested_dsi_fmt;

	switch (pixel_format) {
	case PIXEL_FORMAT_RGB_565:
		requested_dsi_fmt = MIPI_DSI_PIXFMT_RGB565;
		break;
	case PIXEL_FORMAT_RGB_666:
		requested_dsi_fmt = MIPI_DSI_PIXFMT_RGB666;
		break;
	case PIXEL_FORMAT_RGB_888:
		requested_dsi_fmt = MIPI_DSI_PIXFMT_RGB888;
		break;
	default:
		return -ENOTSUP;
	}

	if (requested_dsi_fmt != cfg->dsi_pixfmt) {
		return -ENOTSUP;
	}

	ret = st7801n_map_dsi_pixfmt(requested_dsi_fmt, &colmod, &bpp);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	ret = st7801n_dcs_write(dev, ST7801N_CMD_COLMOD, &colmod, 1);
	if (ret == 0) {
		data->colmod = colmod;
		data->bytes_per_pixel = bpp;
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_orientation(const struct device *dev, enum display_orientation orientation)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(orientation);
	return -ENOTSUP;
}

static int st7801n_read(const struct device *dev, uint16_t x, uint16_t y,
			const struct display_buffer_descriptor *desc, void *buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(desc);
	ARG_UNUSED(buf);
	return -ENOTSUP;
}

#ifdef CONFIG_PM_DEVICE
static int st7801n_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (data->powered_on) {
			(void)st7801n_dcs_write(dev, ST7801N_CMD_DISPOFF, NULL, 0);
			(void)st7801n_dcs_write(dev, ST7801N_CMD_SLPIN, NULL, 0);
			k_msleep(ST7801N_SLPIN_DELAY_MS);
			data->powered_on = false;
			data->blanking_on = true;
		}

		ret = st7801n_detach_dsi(dev);
		if (ret < 0) {
			goto out;
		}

		if (cfg->vci_reg.dev != NULL) {
			(void)regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
		}
		if (cfg->vddi_reg.dev != NULL) {
			(void)regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
		}
		break;

	case PM_DEVICE_ACTION_RESUME:
		if (cfg->vddi_reg.dev != NULL) {
			ret = regulator_enable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
			if (ret < 0) {
				goto out;
			}
			k_msleep(1);
		}
		if (cfg->vci_reg.dev != NULL) {
			ret = regulator_enable(cfg->vci_reg.dev, cfg->vci_reg.id);
			if (ret < 0) {
				if (cfg->vddi_reg.dev != NULL) {
					(void)regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
				}
				goto out;
			}
			k_msleep(1);
		}

		ret = st7801n_attach_dsi(dev);
		if (ret < 0) {
			goto err_resume_power;
		}

		ret = st7801n_hw_init(dev);
		if (ret < 0) {
			goto err_resume_dsi;
		}
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	goto out;

err_resume_dsi:
	(void)st7801n_detach_dsi(dev);
err_resume_power:
	if (cfg->vci_reg.dev != NULL) {
		(void)regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
	}
	if (cfg->vddi_reg.dev != NULL) {
		(void)regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(display, st7801n_api) = {
	.blanking_on = st7801n_blanking_on,
	.blanking_off = st7801n_blanking_off,
	.write = st7801n_write,
	.read = st7801n_read,
	.set_brightness = st7801n_set_brightness,
	.set_contrast = st7801n_set_contrast,
	.get_capabilities = st7801n_get_capabilities,
	.set_pixel_format = st7801n_set_pixel_format,
	.set_orientation = st7801n_set_orientation,
};

static int st7801n_init(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	if ((cfg->width == 0U) || (cfg->height == 0U)) {
		return -EINVAL;
	}

	if ((cfg->width % 2U != 0U) || (cfg->height % 2U != 0U)) {
		return -EINVAL;
	}

	if (cfg->max_dsi_payload < 2U) {
		return -EINVAL;
	}

	ret = st7801n_map_dsi_pixfmt(cfg->dsi_pixfmt, &data->colmod, &data->bytes_per_pixel);
	if (ret < 0) {
		LOG_ERR("Unsupported DSI pixel format 0x%x", cfg->dsi_pixfmt);
		return ret;
	}

	data->brightness = 0xFFU;
	data->powered_on = false;
	data->blanking_on = true;
	data->dsi_attached = false;

	data->mdev.data_lanes = cfg->data_lanes;
	data->mdev.pixfmt = cfg->dsi_pixfmt;
	data->mdev.mode_flags = MIPI_DSI_MODE_LPM;

#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
	if (CONFIG_ST7801N_STATIC_BUF_SIZE < (cfg->max_dsi_payload - 1U)) {
		LOG_ERR("Static buffer (%d) smaller than required payload (%u)",
			CONFIG_ST7801N_STATIC_BUF_SIZE, cfg->max_dsi_payload - 1U);
		return -ENOMEM;
	}
#endif

	if (!device_is_ready(cfg->mipi_dsi)) {
		return -ENODEV;
	}

	if (cfg->vddi_reg.dev != NULL) {
		ret = regulator_enable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
		if (ret < 0) {
			return ret;
		}
		k_msleep(1);
	}
	if (cfg->vci_reg.dev != NULL) {
		ret = regulator_enable(cfg->vci_reg.dev, cfg->vci_reg.id);
		if (ret < 0) {
			if (cfg->vddi_reg.dev != NULL) {
				(void)regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
			}
			return ret;
		}
		k_msleep(1);
	}

	if (cfg->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			ret = -ENODEV;
			goto err_power;
		}
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			goto err_power;
		}
	}

	if (cfg->te_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->te_gpio)) {
			ret = -ENODEV;
			goto err_power;
		}
		ret = gpio_pin_configure_dt(&cfg->te_gpio, GPIO_INPUT);
		if (ret < 0) {
			goto err_power;
		}
	}

	k_mutex_init(&data->lock);

	ret = st7801n_attach_dsi(dev);
	if (ret < 0) {
		goto err_power;
	}

	ret = st7801n_hw_init(dev);
	if (ret < 0) {
		goto err_attach;
	}

	LOG_INF("ST7801N initialized");
	return 0;

err_attach:
	(void)st7801n_detach_dsi(dev);
err_power:
	if (cfg->vci_reg.dev != NULL) {
		(void)regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
	}
	if (cfg->vddi_reg.dev != NULL) {
		(void)regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
	}
	return ret;
}

#define ST7801N_DT_INST_DEFINE(inst)                                                           \
	static const struct st7801n_config st7801n_config_##inst = {                         \
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(inst)),                                \
		.channel = DT_INST_REG_ADDR(inst),                                            \
		.data_lanes = DT_INST_PROP_BY_IDX(inst, data_lanes, 0),                       \
		.dsi_pixfmt = DT_INST_PROP(inst, pixel_format),                               \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),               \
		.te_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, te_gpios, {0}),                     \
		.vddi_reg = REGULATOR_DT_SPEC_INST_GET_OR(inst, vddi_supply, {}),             \
		.vci_reg = REGULATOR_DT_SPEC_INST_GET_OR(inst, vci_supply, {}),               \
		.width = DT_INST_PROP(inst, width),                                            \
		.height = DT_INST_PROP(inst, height),                                          \
		.madctr = DT_INST_PROP(inst, madctr),                                          \
		.invert_display = DT_INST_PROP(inst, invert_display),                          \
		.te_param = ST7801N_TE_MODE_TO_PARAM(inst),                                    \
		.max_dsi_payload = DT_INST_PROP(inst, max_dsi_payload),                        \
	};                                                                                     \
	static struct st7801n_data st7801n_data_##inst;                                        \
	PM_DEVICE_DT_INST_DEFINE(inst, st7801n_pm_action);                                     \
	DEVICE_DT_INST_DEFINE(inst, st7801n_init, PM_DEVICE_DT_INST_GET(inst),                 \
			      &st7801n_data_##inst, &st7801n_config_##inst, POST_KERNEL,      \
			      CONFIG_DISPLAY_INIT_PRIORITY, &st7801n_api);

DT_INST_FOREACH_STATUS_OKAY(ST7801N_DT_INST_DEFINE)