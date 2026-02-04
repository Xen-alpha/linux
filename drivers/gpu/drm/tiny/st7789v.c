// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix ST7789VW
 * display controller in SPI mode.
 *
 * Author: 2026 Xen-alpha <senouis@gmail.com>
 * SPDX-Licese-Identifier: GPL-2.0
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>

/* Macros that comes from 'drivers/gpu/drm/panel/panel-sitronix-st7789v.c' */
#define ST7789V_RAMCTRL_CMD		0xb0
#define ST7789V_RGBCTRL_CMD		0xb1
#define ST7789V_PORCTRL_CMD		0xb2
#define ST7789V_GCTRL_CMD		0xb7
#define ST7789V_VCOMS_CMD		0xbb
#define ST7789V_LCMCTRL_CMD		0xc0
#define ST7789V_VDVVRHEN_CMD		0xc2
#define ST7789V_VRHS_CMD		0xc3
#define ST7789V_VDVS_CMD		0xc4
#define ST7789V_FRCTRL2_CMD		0xc6
#define ST7789V_PWCTRL1_CMD		0xd0
#define ST7789V_PVGAMCTRL_CMD		0xe0
#define ST7789V_NVGAMCTRL_CMD		0xe1

#define ST7789V_PORCTRL_IDLE_BP(n)		(((n) & 0xf) << 4)
#define ST7789V_PORCTRL_IDLE_FP(n)		((n) & 0xf)
#define ST7789V_PORCTRL_PARTIAL_BP(n)		(((n) & 0xf) << 4)
#define ST7789V_PORCTRL_PARTIAL_FP(n)		((n) & 0xf)

#define ST7789V_GCTRL_VGHS(n)			(((n) & 7) << 4)
#define ST7789V_GCTRL_VGLS(n)			((n) & 7)

#define ST7789V_VDVVRHEN_CMDEN			BIT(0)
#define ST7789V_PWCTRL1_MAGIC			0xa4
#define ST7789V_LCMCTRL_XBGR			BIT(5)
#define ST7789V_LCMCTRL_XMX			BIT(3)
#define ST7789V_LCMCTRL_XMH			BIT(2)

#define ST7789V_PWCTRL1_MAGIC			0xa4
#define ST7789V_PWCTRL1_AVDD(n)			(((n) & 3) << 6)
#define ST7789V_PWCTRL1_AVCL(n)			(((n) & 3) << 4)
#define ST7789V_PWCTRL1_VDS(n)			((n) & 3)

struct st7789v_cfg {
	struct mipi_dbi_dev dbidev;	/* Must be first for .release() */
	struct drm_display_mode mode;
};

static const struct drm_display_mode st7789vw_mode = {
	.clock = 6000,
	.hdisplay = 240,
	.hsync_start = 240 + 10,
	.hsync_end = 240 + 20,
	.htotal = 240 + 30,
	.vdisplay = 240,
	.vsync_start = 240 + 10,
	.vsync_end = 240 + 20,
	.vtotal = 240 + 30,
	.width_mm = 30,
	.height_mm = 30,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};


static void st7789v_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct st7789v_cfg *priv = container_of(dbidev, struct st7789v_cfg,
						 dbidev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;
	
	pr_info("enabling st7789v...\n");
	
	if (!drm_dev_enter(pipe->crtc.dev, &idx)) {
		pr_err("cannot enter drm device for st7789v\n");
		return;
	}

	DRM_DEBUG_KMS("\n");
	
	ret = mipi_dbi_poweron_reset(dbidev);
	if (ret) {
		pr_err("cannot reset st7789v to enable\n");
		drm_dev_exit(idx);
		return;
	}
	
	msleep(120);
	
	// send reset command to st7789v
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);	
	msleep(120); // sleep for 120ms
	
	bool is_vw = device_is_compatible(dbidev->drm.dev, "waveshare,st7789vw");
	
	if (is_vw)
		pr_info("st7789vw waveshare variant detected\n");
	
	/* ST7789VW should write 0x70 instead of writing 0. */
	if (is_vw)
		mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, 0x70);
	else
		mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, 0);
	/* Set Pixel Format : currently only can set RGB565 format */
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	
	/* Porch control */
	mipi_dbi_command(dbi, ST7789V_PORCTRL_CMD, 0xc, 0xc, 0x0, 
						ST7789V_PORCTRL_IDLE_BP(3) | ST7789V_PORCTRL_IDLE_FP(3),
						ST7789V_PORCTRL_PARTIAL_BP(3) | ST7789V_PORCTRL_PARTIAL_FP(3));
	/* Gate control : Write 0x35 */
	mipi_dbi_command(dbi, ST7789V_GCTRL_CMD, ST7789V_GCTRL_VGLS(5) | ST7789V_GCTRL_VGHS(3));

	if (is_vw)
		mipi_dbi_command(dbi, ST7789V_VCOMS_CMD, 0x1a);
	else
		mipi_dbi_command(dbi, ST7789V_VCOMS_CMD, 0x2b);

	if (is_vw)
		mipi_dbi_command(dbi, ST7789V_LCMCTRL_CMD, 0x2c);
	else
		mipi_dbi_command(dbi, ST7789V_LCMCTRL_XMH |
							ST7789V_LCMCTRL_XMX |
							ST7789V_LCMCTRL_XBGR);

	mipi_dbi_command(dbi, ST7789V_VDVVRHEN_CMD, ST7789V_VDVVRHEN_CMDEN);

	if (is_vw)
		mipi_dbi_command(dbi, ST7789V_VRHS_CMD, 0xb);
	else
		mipi_dbi_command(dbi, ST7789V_VRHS_CMD, 0xf);

	mipi_dbi_command(dbi, ST7789V_VDVS_CMD, 0x20);

	mipi_dbi_command(dbi, ST7789V_FRCTRL2_CMD, 0xf);

	mipi_dbi_command(dbi, ST7789V_PWCTRL1_CMD, ST7789V_PWCTRL1_MAGIC, ST7789V_PWCTRL1_AVDD(2) |
											ST7789V_PWCTRL1_AVCL(2) |
											ST7789V_PWCTRL1_VDS(1));
	
	/* Positive gamma control */
	mipi_dbi_command(dbi, ST7789V_PVGAMCTRL_CMD, 0x00, 0x19, 0x1e, 0x0a, 0x09, 0x15, 0x3d, 
												0x44, 0x51, 0x12, 0x03, 0x00, 0x3f, 0x3f);
	
	/* Negative gamma control */
	mipi_dbi_command(dbi, ST7789V_NVGAMCTRL_CMD, 0x00, 0x18, 0x1e, 0x0a, 0x09, 0x25, 0x3f,
												0x43, 0x52, 0x33, 0x03, 0x00, 0x3f, 0x3f);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	msleep(100);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
	
	pr_info("enabled st7789v display drm\n");
}

static void st7789v_disable(struct drm_simple_display_pipe *pipe) {
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	pr_info("disabling st7789v drm...\n");
	mipi_dbi_command(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_OFF);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	.enable = st7789v_enable,
	.disable = st7789v_disable,
	.update = mipi_dbi_pipe_update,
};

DEFINE_DRM_GEM_DMA_FOPS(st7789v_fops);

static const struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v",
	.desc			= "Sitronix ST7789V",
	.date			= "20260202",
	.major			= 1,
	.minor			= 0,
};
static const struct of_device_id st7789v_of_match[] = {
	{ .compatible = "waveshare,st7789vw" },
	{ },
};
MODULE_DEVICE_TABLE(of, st7789v_of_match);

static const struct spi_device_id st7789v_id[] = {
	{ "st7789vw", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7789v_id);

static int st7789v_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct st7789v_cfg *cfg;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	int ret;
	u32 rotation = 0;
	

	cfg = devm_kzalloc(dev, sizeof(*cfg), GFP_KERNEL);
	if (IS_ERR(cfg))
		return dev_err_probe(dev, PTR_ERR(cfg), "Failed to get memory area for st7789v context\n");

	dbidev = &cfg->dbidev;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;
	
	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");
	
	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);
	
	device_property_read_u32(dev, "rotation", &rotation);
	cfg->dbidev.rotation = rotation;
	
	// pr_info("st7789v: cfg=%p\n", cfg);
	// pr_info("st7789v: spi=%p\n", spi);

	ret = mipi_dbi_spi_init(spi, dbi, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init mipi spi for st7789v\n");
	
	cfg->mode = st7789vw_mode; // TODO: use switch statement to select display mode for other lcd controllers
	
	//pr_info("st7789v: return value of mipi_dbi_spi_init=%d\n", ret);
	pr_info("st7789v: dbidev.drm.dev=%p\n", cfg->dbidev.drm.dev);
	pr_info("st7789v: dbidev.dbi.spi=%p\n", cfg->dbidev.dbi.spi);


	ret = mipi_dbi_dev_init(dbidev, &st7789v_pipe_funcs, &cfg->mode,
				DRM_FORMAT_RGB565);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init mipi device st7789v\n");


	ret = drm_dev_register(drm, 0);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register st7789v drm device\n");

	spi_set_drvdata(spi, drm);

	pr_info("st7789v probe finished\n");
	return 0;
	
}

static void st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7789v_spi_driver = {
	.driver = {
		.name = "st7789v",
		.of_match_table = st7789v_of_match,
	},
	.id_table = st7789v_id,
	.probe = st7789v_probe,
	.remove = st7789v_remove,
	.shutdown = st7789v_shutdown,
};
module_spi_driver(st7789v_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7789V DRM driver");
MODULE_AUTHOR("Xen alpha <senouis@gmail.com>");
MODULE_LICENSE("GPL");
