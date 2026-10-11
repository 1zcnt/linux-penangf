// SPDX-License-Identifier: GPL-2.0-only
/*
 * BOE 6.5" 720x1600 video mode DSI panel with a Synaptics TD4160 TDDI
 * controller, as found in some Motorola Moto G13 / G23 (penangf) units.
 *
 * Based on the downstream panel-td4160-hdp-dsi-vdo-boe driver:
 *   Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/*
 * The controller takes an 11 bit brightness value. Downstream scales the
 * user range to 65% of it (and uses 80% for its "high brightness" mode), so
 * stay within the 65% that is used for normal operation.
 */
#define BOE_TD4160_MAX_BRIGHTNESS	1330

struct boe_td4160 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	bool enabled;
};

static const struct regulator_bulk_data boe_td4160_supplies[] = {
	{ .supply = "vsp" },
	{ .supply = "vsn" },
};

static inline struct boe_td4160 *to_boe_td4160(struct drm_panel *panel)
{
	return container_of(panel, struct boe_td4160, panel);
}

static void boe_td4160_reset(struct boe_td4160 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int boe_td4160_on(struct boe_td4160 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 brightness = backlight_get_brightness(ctx->backlight);
	const u8 bl_cmd[] = {
		MIPI_DCS_SET_DISPLAY_BRIGHTNESS, brightness >> 8, brightness & 0xff,
	};

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x01);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 110);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	/*
	 * Set the brightness here, while the host is still in command mode.
	 * Leaving it to drm_panel_enable() means a command right after the
	 * video stream starts, which intermittently never completes on the
	 * MediaTek DSI host and takes the whole pipeline down with it.
	 */
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, bl_cmd, sizeof(bl_cmd));

	return dsi_ctx.accum_err;
}

static int boe_td4160_off(struct boe_td4160 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int boe_td4160_prepare(struct drm_panel *panel)
{
	struct boe_td4160 *ctx = to_boe_td4160(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/*
	 * The touch part of the TDDI shares the bias supplies and wants its
	 * own reset released before they come up; that line is left to the
	 * touchscreen driver.
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(boe_td4160_supplies),
				    ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	msleep(15);

	boe_td4160_reset(ctx);

	ret = boe_td4160_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(boe_td4160_supplies),
				       ctx->supplies);
		return ret;
	}

	return 0;
}

static int boe_td4160_unprepare(struct drm_panel *panel)
{
	struct boe_td4160 *ctx = to_boe_td4160(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = boe_td4160_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(3000, 4000);
	regulator_bulk_disable(ARRAY_SIZE(boe_td4160_supplies), ctx->supplies);

	return 0;
}

/* Same pixel clock as the panel's 90 Hz mode, stretched with front porch */
static const struct drm_display_mode boe_td4160_mode = {
	.clock = (720 + 90 + 10 + 66) * (1600 + 1010 + 4 + 20) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 90,
	.hsync_end = 720 + 90 + 10,
	.htotal = 720 + 90 + 10 + 66,
	.vdisplay = 1600,
	.vsync_start = 1600 + 1010,
	.vsync_end = 1600 + 1010 + 4,
	.vtotal = 1600 + 1010 + 4 + 20,
	.width_mm = 65,
	.height_mm = 129,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int boe_td4160_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &boe_td4160_mode);
}

static int boe_td4160_enable(struct drm_panel *panel)
{
	struct boe_td4160 *ctx = to_boe_td4160(panel);

	ctx->enabled = true;

	/* Catch a brightness change that arrived while the panel came up */
	backlight_update_status(ctx->backlight);

	return 0;
}

static int boe_td4160_disable(struct drm_panel *panel)
{
	struct boe_td4160 *ctx = to_boe_td4160(panel);

	ctx->enabled = false;

	return 0;
}

static const struct drm_panel_funcs boe_td4160_panel_funcs = {
	.prepare = boe_td4160_prepare,
	.unprepare = boe_td4160_unprepare,
	.enable = boe_td4160_enable,
	.disable = boe_td4160_disable,
	.get_modes = boe_td4160_get_modes,
};

static int boe_td4160_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct boe_td4160 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);

	/*
	 * Only talk to the panel while the video stream is up and stable.
	 * Anything set in between is picked up by boe_td4160_on() and
	 * boe_td4160_enable(); a command sent while the host is starting or
	 * stopping video can hang the MediaTek DSI host.
	 */
	if (!ctx->enabled)
		return 0;

	return mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
}

static const struct backlight_ops boe_td4160_bl_ops = {
	.update_status = boe_td4160_bl_update_status,
};

static struct backlight_device *
boe_td4160_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = BOE_TD4160_MAX_BRIGHTNESS / 2,
		.max_brightness = BOE_TD4160_MAX_BRIGHTNESS,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &boe_td4160_bl_ops, &props);
}

static int boe_td4160_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct boe_td4160 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct boe_td4160, panel,
				   &boe_td4160_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(boe_td4160_supplies),
					    boe_td4160_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	/* Keep the panel out of reset: the bootloader leaves it running */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	/* Deliberately not handed to the panel core, see boe_td4160_on() */
	ctx->backlight = boe_td4160_create_backlight(dsi);
	if (IS_ERR(ctx->backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void boe_td4160_remove(struct mipi_dsi_device *dsi)
{
	struct boe_td4160 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id boe_td4160_of_match[] = {
	{ .compatible = "boe,td4160" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, boe_td4160_of_match);

static struct mipi_dsi_driver boe_td4160_driver = {
	.probe = boe_td4160_probe,
	.remove = boe_td4160_remove,
	.driver = {
		.name = "panel-boe-td4160",
		.of_match_table = boe_td4160_of_match,
	},
};
module_mipi_dsi_driver(boe_td4160_driver);

MODULE_DESCRIPTION("DRM driver for BOE TD4160 HD+ video mode DSI panel");
MODULE_LICENSE("GPL");
