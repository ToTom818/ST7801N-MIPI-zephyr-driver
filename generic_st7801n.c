/*
 * Copyright (c) 2026 TOTOM818
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <string.h>

LOG_MODULE_REGISTER(st7801n, CONFIG_DISPLAY_LOG_LEVEL);

/* ST7801N DCS commands (based on datasheet) */
#define ST7801N_CMD_NOP             0x00
#define ST7801N_CMD_SWRESET         0x01
#define ST7801N_CMD_SLPIN           0x10
#define ST7801N_CMD_SLPOUT          0x11
#define ST7801N_CMD_PTLON           0x12
#define ST7801N_CMD_NORON           0x13
#define ST7801N_CMD_INVOFF          0x20
#define ST7801N_CMD_INVON           0x21
#define ST7801N_CMD_ALLPOFF         0x22
#define ST7801N_CMD_ALLPON          0x23
#define ST7801N_CMD_DISPOFF         0x28
#define ST7801N_CMD_DISPON          0x29
#define ST7801N_CMD_CASET           0x2A
#define ST7801N_CMD_RASET           0x2B
#define ST7801N_CMD_RAMWR           0x2C
#define ST7801N_CMD_TEOFF           0x34
#define ST7801N_CMD_TEON            0x35
#define ST7801N_CMD_MADCTR          0x36
#define ST7801N_CMD_IDMOFF          0x38
#define ST7801N_CMD_IDMON           0x39
#define ST7801N_CMD_COLMOD          0x3A
#define ST7801N_CMD_WRDISBV         0x51
#define ST7801N_CMD_WRCTRLD         0x53
#define ST7801N_CMD_WRACL           0x55
#define ST7801N_CMD_HBMEN           0x5E

/* COLMOD (0x3A) values with SPI_IF_SEL=0 and VIPF=0 (bits 2-0 = IFPF) */
#define ST7801N_PIXFMT_RGB565       0x05  /* 16-bit/pixel RGB565 */
#define ST7801N_PIXFMT_RGB666       0x06  /* 18-bit/pixel RGB666 */
#define ST7801N_PIXFMT_RGB888       0x07  /* 24-bit/pixel RGB888 */

/* MADCTR bits */
#define ST7801N_MADCTR_MY            BIT(7)  /* Row address order */
#define ST7801N_MADCTR_MX            BIT(6)  /* Column address order */
#define ST7801N_MADCTR_MV            BIT(5)  /* Row/column exchange */
#define ST7801N_MADCTR_RSMY          BIT(4)  /* Vertical flip */
#define ST7801N_MADCTR_BGR           BIT(3)  /* RGB/BGR order */
#define ST7801N_MADCTR_RSMX          BIT(2)  /* Horizontal flip */

/* TEON parameter */
#define ST7801N_TEON_MODE1           0x00    /* only V-blanking */
#define ST7801N_TEON_MODE2           0x03    /* V+H blanking (both TE_M and TELOM set) */

/* Maximum number of parameters for a DCS command (prevents stack overflow) */
#define ST7801N_MAX_DCS_PARAMS       32

/* Helper to convert DT te-mode string to TEON parameter */
#define ST7801N_TE_MODE_TO_PARAM(inst) \
	(DT_INST_NODE_HAS_PROP(inst, te_mode) ? \
		(DT_INST_ENUM_IDX(inst, te_mode) == 0 ? ST7801N_TEON_MODE1 : ST7801N_TEON_MODE2) : 0)

/* Device configuration (read-only) */
struct st7801n_config {
	struct mipi_dsi_dt_spec dsi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec te_gpio;
	struct regulator_dt_spec vddi_reg;
	struct regulator_dt_spec vci_reg;
	uint16_t width;
	uint16_t height;
	uint32_t pix_fmt_token;		/* from DT, used to set data->pix_fmt */
	uint8_t madctr;
	bool invert_display;
	uint8_t te_param;			/* TEON parameter (0 = disabled) */
	uint32_t max_dsi_payload;		/* Maximum DSI long packet payload */
};

/* Device run-time data */
struct st7801n_data {
	struct mipi_dsi_device *dsi_dev;
	struct k_mutex lock;
	bool powered_on;
	bool blanking_on;
	uint8_t brightness;
	uint8_t pix_fmt;			/* current pixel format (COLMOD value) */
#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
	uint8_t tmp_buf[CONFIG_ST7801N_STATIC_BUF_SIZE];
#endif
#ifdef CONFIG_PM_DEVICE
	uint32_t pm_state;
#endif
};

/* Helper to map device tree string token to pixel format byte */
static uint8_t st7801n_pixfmt_from_token(uint32_t token)
{
	switch (token) {
	case DT_STRING_TOKEN(rgb888):
		return ST7801N_PIXFMT_RGB888;
	case DT_STRING_TOKEN(rgb666):
		return ST7801N_PIXFMT_RGB666;
	case DT_STRING_TOKEN(rgb565):
		return ST7801N_PIXFMT_RGB565;
	default:
		return 0;
	}
}

/* Helper to send DCS commands using appropriate packet type.
 * Returns 0 on success, negative error code on failure.
 */
static int st7801n_dcs_write_packet(const struct device *dev, uint8_t cmd,
				     const uint8_t *params, size_t num_params)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	struct mipi_dsi_packet packet;
	int ret;

	/* Prevent stack overflow if an unreasonably large parameter count is passed */
	if (num_params > ST7801N_MAX_DCS_PARAMS) {
		LOG_ERR("Too many DCS parameters (%zu > %d)", num_params,
			ST7801N_MAX_DCS_PARAMS);
		return -EINVAL;
	}

	if (num_params == 0) {
		ret = mipi_dsi_packet_create_short(&packet,
						    cfg->dsi.config.virtual_channel,
						    MIPI_DSI_DCS_SHORT_WRITE,
						    cmd, 0);
	} else if (num_params == 1) {
		ret = mipi_dsi_packet_create_short(&packet,
						    cfg->dsi.config.virtual_channel,
						    MIPI_DSI_DCS_SHORT_WRITE_PARAM,
						    cmd, params[0]);
	} else {
		uint8_t buf[1 + num_params];
		buf[0] = cmd;
		memcpy(&buf[1], params, num_params);
		/* For long packet, header must be the word count (payload length) */
		ret = mipi_dsi_packet_create_long(&packet,
						   cfg->dsi.config.virtual_channel,
						   MIPI_DSI_DCS_LONG_WRITE,
						   1 + num_params,	/* header = word count */
						   1 + num_params, buf);
	}

	if (ret < 0) {
		return ret;
	}

	ret = mipi_dsi_transfer(data->dsi_dev, &packet);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* Locking wrapper */
static int st7801n_dcs_write(const struct device *dev, uint8_t cmd,
			      const uint8_t *params, size_t num_params)
{
	struct st7801n_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = st7801n_dcs_write_packet(dev, cmd, params, num_params);
	k_mutex_unlock(&data->lock);

	return ret;
}

static inline int st7801n_dcs_write_simple(const struct device *dev, uint8_t cmd)
{
	return st7801n_dcs_write(dev, cmd, NULL, 0);
}

/* Initialize display hardware (called from init and resume) */
static int st7801n_hw_init(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	/* Hardware reset */
	if (cfg->reset_gpio.port) {
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(1);
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_msleep(2);
	}

	/* Attach DSI device if not already attached (e.g., after resume) */
	if (!data->dsi_dev) {
		data->dsi_dev = mipi_dsi_device_attach(cfg->dsi.bus, &cfg->dsi.config);
		if (!data->dsi_dev) {
			LOG_ERR("Failed to attach DSI device");
			return -ENODEV;
		}
	}

	/* Exit sleep mode */
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_SLPOUT, NULL, 0);
	if (ret < 0) {
		LOG_ERR("SLPOUT failed: %d", ret);
		return ret;
	}
	k_msleep(5);

	/* Set pixel format (use data->pix_fmt which was set during init) */
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_COLMOD, &data->pix_fmt, 1);
	if (ret < 0) {
		LOG_ERR("COLMOD failed: %d", ret);
		return ret;
	}

	/* Set address mode */
	if (cfg->madctr) {
		uint8_t madctr = cfg->madctr;
		ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_MADCTR, &madctr, 1);
		if (ret < 0) {
			LOG_ERR("MADCTR failed: %d", ret);
			return ret;
		}
	}

	/* Display inversion */
	if (cfg->invert_display) {
		ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_INVON, NULL, 0);
	} else {
		ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_INVOFF, NULL, 0);
	}
	if (ret < 0) {
		LOG_ERR("INVON/OFF failed: %d", ret);
		return ret;
	}

	/* Tearing effect (if pin present and mode selected) */
	if (cfg->te_gpio.port && cfg->te_param) {
		uint8_t te_param = cfg->te_param;
		ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_TEON, &te_param, 1);
		if (ret < 0) {
			LOG_ERR("TEON failed: %d", ret);
			/* non‑fatal, continue */
		}
	}

	/* Turn on display */
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_DISPON, NULL, 0);
	if (ret < 0) {
		LOG_ERR("DISPON failed: %d", ret);
		return ret;
	}

	/* Always set brightness to the stored value (initialized to 0xFF) */
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_WRDISBV, &data->brightness, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set brightness: %d", ret);
		/* non‑fatal, continue */
	}

	data->powered_on = true;
	data->blanking_on = false;

	return 0;
}

/* Full device initialization (first time) */
static int st7801n_init(const struct device *dev)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret;

	LOG_DBG("Initializing ST7801N display");

	/* Validate even dimensions */
	if ((cfg->width % 2 != 0) || (cfg->height % 2 != 0)) {
		LOG_ERR("Width and height must be even");
		return -EINVAL;
	}

	/* Sanity check: max_dsi_payload must be at least 2 (command + 1 byte) */
	if (cfg->max_dsi_payload < 2) {
		LOG_ERR("max-dsi-payload is too small (%u)", cfg->max_dsi_payload);
		return -EINVAL;
	}

	/* Convert pixel format token to COLMOD value and store in data */
	data->pix_fmt = st7801n_pixfmt_from_token(cfg->pix_fmt_token);
	if (data->pix_fmt == 0) {
		LOG_ERR("Invalid pixel format token");
		return -EINVAL;
	}

	/* Initialize brightness to hardware default (0xFF) */
	data->brightness = 0xFF;

#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
	/* Verify that static buffer is large enough for DSI payload */
	if (CONFIG_ST7801N_STATIC_BUF_SIZE < cfg->max_dsi_payload) {
		LOG_ERR("Static buffer size (%d) is too small for max DSI payload (%d)",
			CONFIG_ST7801N_STATIC_BUF_SIZE, cfg->max_dsi_payload);
		return -ENOMEM;
	}
#endif

	/* Power on regulators */
	if (cfg->vddi_reg.dev) {
		ret = regulator_enable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
		if (ret < 0) {
			LOG_ERR("Failed to enable VDDIO regulator: %d", ret);
			return ret;
		}
		/* Allow regulator to stabilize */
		k_msleep(1);
	}
	if (cfg->vci_reg.dev) {
		ret = regulator_enable(cfg->vci_reg.dev, cfg->vci_reg.id);
		if (ret < 0) {
			LOG_ERR("Failed to enable VCI regulator: %d", ret);
			goto err_vci;
		}
		k_msleep(1);
	}

	/* Configure reset GPIO */
	if (cfg->reset_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			ret = -ENODEV;
			goto err_gpio;
		}
		gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
	}

	/* Configure TE GPIO if present (input) */
	if (cfg->te_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->te_gpio)) {
			LOG_ERR("TE GPIO not ready");
			ret = -ENODEV;
			goto err_gpio;
		}
		gpio_pin_configure_dt(&cfg->te_gpio, GPIO_INPUT);
	}

	/* Ensure DSI bus is ready */
	if (!device_is_ready(cfg->dsi.bus)) {
		LOG_ERR("DSI bus %s not ready", cfg->dsi.bus->name);
		ret = -ENODEV;
		goto err_gpio;
	}

	k_mutex_init(&data->lock);

	ret = st7801n_hw_init(dev);
	if (ret < 0) {
		LOG_ERR("Hardware init failed: %d", ret);
		goto err_hw;
	}

	LOG_INF("ST7801N initialized");
	return 0;

err_hw:
	/* Fall through to regulator cleanup */
err_gpio:
	if (cfg->vci_reg.dev) {
		regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
	}
err_vci:
	if (cfg->vddi_reg.dev) {
		regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
	}
	return ret;
}

/* Power management */
#ifdef CONFIG_PM_DEVICE
static int st7801n_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Turn off display, sleep, disable regulators */
		k_mutex_lock(&data->lock, K_FOREVER);
		if (data->powered_on) {
			ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_DISPOFF, NULL, 0);
			if (ret < 0) {
				LOG_WRN("DISPOFF during suspend failed: %d", ret);
			}
			ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_SLPIN, NULL, 0);
			if (ret < 0) {
				LOG_WRN("SLPIN during suspend failed: %d", ret);
			}
			k_msleep(5);
			if (cfg->vci_reg.dev) {
				regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
			}
			if (cfg->vddi_reg.dev) {
				regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
			}
			data->powered_on = false;
			/* NOTE: Do not invalidate dsi_dev handle; keep it for resume */
		}
		k_mutex_unlock(&data->lock);
		break;

	case PM_DEVICE_ACTION_RESUME:
		/* Enable regulators, re-init hardware */
		k_mutex_lock(&data->lock, K_FOREVER);
		if (cfg->vddi_reg.dev) {
			ret = regulator_enable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
			if (ret < 0) {
				LOG_ERR("Failed to re-enable VDDIO regulator: %d", ret);
				goto out;
			}
			k_msleep(1);
		}
		if (cfg->vci_reg.dev) {
			ret = regulator_enable(cfg->vci_reg.dev, cfg->vci_reg.id);
			if (ret < 0) {
				LOG_ERR("Failed to re-enable VCI regulator: %d", ret);
				if (cfg->vddi_reg.dev) {
					regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
				}
				goto out;
			}
			k_msleep(1);
		}
		ret = st7801n_hw_init(dev);
		if (ret < 0) {
			LOG_ERR("Resume init failed: %d", ret);
			if (cfg->vci_reg.dev) {
				regulator_disable(cfg->vci_reg.dev, cfg->vci_reg.id);
			}
			if (cfg->vddi_reg.dev) {
				regulator_disable(cfg->vddi_reg.dev, cfg->vddi_reg.id);
			}
		}
	out:
		k_mutex_unlock(&data->lock);
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Display driver API functions */
static int st7801n_blanking_on(const struct device *dev)
{
	struct st7801n_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}
	if (data->blanking_on) {
		ret = 0;
		goto out;
	}

	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_DISPOFF, NULL, 0);
	if (ret == 0) {
		data->blanking_on = true;
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_blanking_off(const struct device *dev)
{
	struct st7801n_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}
	if (!data->blanking_on) {
		ret = 0;
		goto out;
	}

	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_DISPON, NULL, 0);
	if (ret == 0) {
		data->blanking_on = false;
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_write(const struct device *dev, const uint16_t x,
			 const uint16_t y, const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;
	uint8_t col_params[4];
	uint8_t row_params[4];
	uint8_t bpp;
	int ret = 0;

	if (!data->powered_on) {
		return -EIO;
	}

	if (desc->pitch < desc->width) {
		LOG_ERR("Pitch is smaller than width");
		return -EINVAL;
	}

	/* According to datasheet, only the total width and height must be even;
	 * start coordinates can be odd.
	 */
	if ((desc->width % 2 != 0) || (desc->height % 2 != 0)) {
		LOG_ERR("Write width and height must be even");
		return -EINVAL;
	}

	/* Bounds check */
	if (x + desc->width > cfg->width || y + desc->height > cfg->height) {
		LOG_ERR("Write region exceeds display bounds");
		return -EINVAL;
	}

	/* Determine bytes per pixel from current pixel format */
	switch (data->pix_fmt) {
	case ST7801N_PIXFMT_RGB888:
	case ST7801N_PIXFMT_RGB666:
		bpp = 3;
		break;
	case ST7801N_PIXFMT_RGB565:
		bpp = 2;
		break;
	default:
		LOG_ERR("Unknown pixel format");
		return -EINVAL;
	}

	/* Safety check: bytes_per_row must be non-zero to avoid division by zero later */
	uint32_t bytes_per_row = desc->width * bpp;
	if (bytes_per_row == 0) {
		LOG_ERR("Bytes per row is zero (invalid width or bpp)");
		return -EINVAL;
	}

	if (desc->buf_size < desc->pitch * desc->height * bpp) {
		LOG_ERR("Input buffer too small: %u < %u",
			desc->buf_size, desc->pitch * desc->height * bpp);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}

	/* Set column address */
	col_params[0] = (x >> 8) & 0xFF;
	col_params[1] = x & 0xFF;
	col_params[2] = ((x + desc->width - 1) >> 8) & 0xFF;
	col_params[3] = (x + desc->width - 1) & 0xFF;
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_CASET, col_params, 4);
	if (ret < 0) {
		goto out;
	}

	/* Set row address */
	row_params[0] = (y >> 8) & 0xFF;
	row_params[1] = y & 0xFF;
	row_params[2] = ((y + desc->height - 1) >> 8) & 0xFF;
	row_params[3] = (y + desc->height - 1) & 0xFF;
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_RASET, row_params, 4);
	if (ret < 0) {
		goto out;
	}

	/* Maximum payload per DSI long packet (excluding command byte) */
	uint32_t max_data_per_packet = cfg->max_dsi_payload - 1;  /* reserve one byte for command */
	uint32_t max_rows_per_packet = max_data_per_packet / bytes_per_row;

	if (max_rows_per_packet == 0) {
		LOG_ERR("Row too large for DSI packet");
		ret = -EINVAL;
		goto out;
	}

	uint32_t rows_remaining = desc->height;
	uint32_t row_offset = 0;
	const uint8_t *src = buf;

	while (rows_remaining > 0) {
		uint32_t rows_this_packet = MIN(rows_remaining, max_rows_per_packet);
		uint32_t data_bytes_this_packet = rows_this_packet * bytes_per_row;
		uint8_t *tmp_buf;

#ifdef CONFIG_ST7801N_USE_STATIC_BUFFER
		/* Use pre-allocated static buffer (size already verified at init) */
		tmp_buf = data->tmp_buf;
		if (data_bytes_this_packet + 1 > CONFIG_ST7801N_STATIC_BUF_SIZE) {
			LOG_ERR("Packet too large for static buffer (%u > %d)",
				data_bytes_this_packet + 1, CONFIG_ST7801N_STATIC_BUF_SIZE);
			ret = -ENOMEM;
			break;
		}
#else
		/* Allocate temporary buffer for this chunk */
		tmp_buf = k_malloc(data_bytes_this_packet + 1);
		if (!tmp_buf) {
			LOG_ERR("Failed to allocate temporary buffer");
			ret = -ENOMEM;
			break;
		}
#endif

		tmp_buf[0] = ST7801N_CMD_RAMWR;

		/* Copy rows from source (with pitch) into packet buffer */
		for (uint32_t r = 0; r < rows_this_packet; r++) {
			uint32_t src_row = row_offset + r;
			memcpy(tmp_buf + 1 + r * bytes_per_row,
			       src + src_row * desc->pitch * bpp,
			       bytes_per_row);
		}

		struct mipi_dsi_packet packet;
		/* Header must be the word count = payload length */
		ret = mipi_dsi_packet_create_long(&packet,
						   cfg->dsi.config.virtual_channel,
						   MIPI_DSI_DCS_LONG_WRITE,
						   data_bytes_this_packet + 1,	/* header */
						   data_bytes_this_packet + 1, tmp_buf);
		if (ret == 0) {
			ret = mipi_dsi_transfer(data->dsi_dev, &packet);
		}

#ifndef CONFIG_ST7801N_USE_STATIC_BUFFER
		k_free(tmp_buf);
#endif

		if (ret < 0) {
			LOG_ERR("DSI transfer failed: %d", ret);
			break;
		}
		/* ret is 0 on success; continue to next chunk */

		rows_remaining -= rows_this_packet;
		row_offset += rows_this_packet;
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_brightness(const struct device *dev, uint8_t brightness)
{
	struct st7801n_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		ret = -EIO;
		goto out;
	}
	ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_WRDISBV, &brightness, 1);
	if (ret == 0) {
		data->brightness = brightness;
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_contrast(const struct device *dev, uint8_t contrast)
{
	return -ENOTSUP;
}

static void st7801n_get_capabilities(const struct device *dev,
				      struct display_capabilities *capabilities)
{
	const struct st7801n_config *cfg = dev->config;
	struct st7801n_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->x_resolution = cfg->width;
	capabilities->y_resolution = cfg->height;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888 |
						 PIXEL_FORMAT_RGB_565 |
						 PIXEL_FORMAT_RGB_666;

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (data->pix_fmt) {
	case ST7801N_PIXFMT_RGB888:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
		break;
	case ST7801N_PIXFMT_RGB666:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_666;
		break;
	case ST7801N_PIXFMT_RGB565:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
		break;
	default:
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
		break;
	}
	k_mutex_unlock(&data->lock);

	capabilities->screen_info = 0;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int st7801n_set_pixel_format(const struct device *dev,
				     enum display_pixel_format pixel_format)
{
	struct st7801n_data *data = dev->data;
	uint8_t colmod;

	switch (pixel_format) {
	case PIXEL_FORMAT_RGB_888:
		colmod = ST7801N_PIXFMT_RGB888;
		break;
	case PIXEL_FORMAT_RGB_666:
		colmod = ST7801N_PIXFMT_RGB666;
		break;
	case PIXEL_FORMAT_RGB_565:
		colmod = ST7801N_PIXFMT_RGB565;
		break;
	default:
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->powered_on) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	int ret = st7801n_dcs_write_packet(dev, ST7801N_CMD_COLMOD, &colmod, 1);
	if (ret == 0) {
		data->pix_fmt = colmod;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int st7801n_set_orientation(const struct device *dev,
				    const enum display_orientation orientation)
{
	return -ENOTSUP;
}

static int st7801n_read(const struct device *dev, const uint16_t x,
			const uint16_t y, const struct display_buffer_descriptor *desc,
			void *buf)
{
	return -ENOTSUP;
}

static const struct display_driver_api st7801n_api = {
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

/* Device instantiation macro */
#define ST7801N_DT_INST_DEFINE(inst) \
	static const struct st7801n_config st7801n_config_##inst = { \
		.dsi = MIPI_DSI_DT_SPEC_INST_GET(inst), \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
		.te_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, te_gpios, {0}), \
		.vddi_reg = REGULATOR_DT_SPEC_INST_GET_OR(inst, vddi-supply, {}), \
		.vci_reg = REGULATOR_DT_SPEC_INST_GET_OR(inst, vci-supply, {}), \
		.width = DT_INST_PROP(inst, width), \
		.height = DT_INST_PROP(inst, height), \
		.pix_fmt_token = DT_INST_STRING_TOKEN(inst, pixel_format), \
		.madctr = DT_INST_PROP(inst, madctr), \
		.invert_display = DT_INST_PROP(inst, invert_display), \
		.te_param = ST7801N_TE_MODE_TO_PARAM(inst), \
		.max_dsi_payload = DT_INST_PROP(inst, max_dsi_payload), \
	}; \
	static struct st7801n_data st7801n_data_##inst; \
	PM_DEVICE_DT_INST_DEFINE(inst, st7801n_pm_action); \
	DEVICE_DT_INST_DEFINE(inst, st7801n_init, PM_DEVICE_DT_INST_GET(inst), \
			      &st7801n_data_##inst, &st7801n_config_##inst, \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, \
			      &st7801n_api);

DT_INST_FOREACH_STATUS_OKAY(ST7801N_DT_INST_DEFINE)