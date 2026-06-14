// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for the Waveshare 1.3inch IPS LCD HAT for Raspberry Pi
 *
 * Panel      : 1.3" IPS, 240(H) x 240(V), RGB565 over 4-line SPI
 * Controller : Sitronix ST7789VW (240 source x 320 gate lines;
 *              the 240x240 glass only uses rows 0..239 of the
 *              240x320 frame RAM, so MY/MX-mirrored orientations
 *              need an 80 pixel address-window offset)
 * Wiring (BCM): DC=GPIO25, RESET=GPIO27, BACKLIGHT=GPIO24, CS=CE0
 * SPI mode 0 (CPOL=0, CPHA=0), write-only (no MISO on the HAT)
 *
 * Built on the kernel's MIPI DBI helper (drm_mipi_dbi), the same
 * infrastructure used by drm/tiny drivers such as mi0283qt/st7735r.
 * Tested target: Raspberry Pi OS 64-bit (aarch64), kernels 6.1 - 6.14.
 *
 * Init sequence and electrical parameters follow the Waveshare
 * reference code and the ST7789VW datasheet (v1.0).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/version.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

/* ---- fbdev emulation setup differs across kernel versions ---------- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 14, 0)
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_fbdev_dma.h>
#define WS13_FBDEV_CLIENT_SETUP 1
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
#include <drm/drm_client_setup.h>
#include <drm/drm_fbdev_dma.h>
#define WS13_FBDEV_CLIENT_SETUP 1
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
#include <drm/drm_fbdev_dma.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#include <drm/drm_fbdev_generic.h>
#else /* 6.1.y: declaration lives in drm_fb_helper.h */
#include <drm/drm_fb_helper.h>
#endif

/* ---- ST7789VW command set (beyond standard MIPI DCS) ---------------- */
#define ST7789_PORCTRL		0xb2	/* Porch setting */
#define ST7789_GCTRL		0xb7	/* Gate control */
#define ST7789_VCOMS		0xbb	/* VCOM setting */
#define ST7789_LCMCTRL		0xc0	/* LCM control */
#define ST7789_VDVVRHEN		0xc2	/* VDV and VRH command enable */
#define ST7789_VRHS		0xc3	/* VRH set */
#define ST7789_VDVS		0xc4	/* VDV set */
#define ST7789_FRCTRL2		0xc6	/* Frame rate control (normal mode) */
#define ST7789_PWCTRL1		0xd0	/* Power control 1 */
#define ST7789_PVGAMCTRL	0xe0	/* Positive voltage gamma control */
#define ST7789_NVGAMCTRL	0xe1	/* Negative voltage gamma control */

/* MADCTL (36h) bits */
#define ST7789_MADCTL_MY	BIT(7)	/* Row address order */
#define ST7789_MADCTL_MX	BIT(6)	/* Column address order */
#define ST7789_MADCTL_MV	BIT(5)	/* Row/column exchange */
#define ST7789_MADCTL_BGR	BIT(3)	/* BGR subpixel order */

/*
 * The glass is bonded at the gate-line-0 end of the 240x320 RAM.
 * Whenever the axis that maps onto the gate lines is reversed
 * (MV=0 & MY=1, or MV=1 & MX=1), addressing must start at line 80.
 */
#define ST7789_RAM_GATE_LINES	320
#define WS13_PANEL_LINES	240
#define WS13_LINE_OFFSET	(ST7789_RAM_GATE_LINES - WS13_PANEL_LINES) /* 80 */

static void ws13_pipe_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;	/* still initialised, just re-enable */

	/*
	 * Datasheet: after HW reset wait >120 ms is already handled by the
	 * helper; SLPOUT itself requires another 120 ms before further
	 * commands may rely on stable internal supplies.
	 */
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(120);

	/* 16 bit/pixel, 65K RGB565 (datasheet COLMOD value 0x55) */
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x55);

	/* Porch: BPA=0x0C, FPA=0x0C, PSEN off, idle/partial porch 0x33 */
	mipi_dbi_command(dbi, ST7789_PORCTRL,
			 0x0c, 0x0c, 0x00, 0x33, 0x33);
	/* Gate control: VGH=13.26 V, VGL=-10.43 V */
	mipi_dbi_command(dbi, ST7789_GCTRL, 0x35);
	/* VCOM = 0.725 V */
	mipi_dbi_command(dbi, ST7789_VCOMS, 0x19);
	/* LCM control: XMX | XMH (default polarity inversion source) */
	mipi_dbi_command(dbi, ST7789_LCMCTRL, 0x2c);
	/* Enable VDV/VRH register programming */
	mipi_dbi_command(dbi, ST7789_VDVVRHEN, 0x01);
	/* VRH = 4.45 V + (vcom + vcom offset + 0.5*vdv) */
	mipi_dbi_command(dbi, ST7789_VRHS, 0x12);
	/* VDV = 0 V */
	mipi_dbi_command(dbi, ST7789_VDVS, 0x20);
	/* Frame rate 60 Hz in normal mode */
	mipi_dbi_command(dbi, ST7789_FRCTRL2, 0x0f);
	/* AVDD=6.8 V, AVCL=-4.8 V, VDDS=2.3 V */
	mipi_dbi_command(dbi, ST7789_PWCTRL1, 0xa4, 0xa1);

	/* Gamma curves (Waveshare factory calibration for this glass) */
	mipi_dbi_command(dbi, ST7789_PVGAMCTRL,
			 0xd0, 0x04, 0x0d, 0x11, 0x13, 0x2b, 0x3f,
			 0x54, 0x4c, 0x18, 0x0d, 0x0b, 0x1f, 0x23);
	mipi_dbi_command(dbi, ST7789_NVGAMCTRL,
			 0xd0, 0x04, 0x0c, 0x11, 0x13, 0x2c, 0x3f,
			 0x44, 0x51, 0x2f, 0x1f, 0x1f, 0x20, 0x23);

	/*
	 * IPS glass with normally-black liquid crystal: the controller's
	 * "inverted" polarity is the visually correct one.
	 */
	mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE);

	mipi_dbi_command(dbi, MIPI_DCS_ENTER_NORMAL_MODE);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(20);

out_enable:
	switch (dbidev->rotation) {
	default:	/* 0 deg */
		addr_mode = 0x00;
		break;
	case 90:
		addr_mode = ST7789_MADCTL_MV | ST7789_MADCTL_MY;
		break;
	case 180:
		addr_mode = ST7789_MADCTL_MX | ST7789_MADCTL_MY;
		break;
	case 270:
		addr_mode = ST7789_MADCTL_MV | ST7789_MADCTL_MX;
		break;
	}
	/* Subpixel order is RGB: leave the BGR bit clear */
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ws13_pipe_funcs = {
	DRM_MIPI_DBI_SIMPLE_DISPLAY_PIPE_FUNCS(ws13_pipe_enable),
};

/* 240x240 @ 0.0975 mm/px => 23.4 x 23.4 mm active area */
static const struct drm_display_mode ws13_mode = {
	DRM_SIMPLE_MODE(240, 240, 23, 23),
};

DEFINE_DRM_GEM_DMA_FOPS(ws13_fops);

static const struct drm_driver ws13_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ws13_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
#ifdef WS13_FBDEV_CLIENT_SETUP
	DRM_FBDEV_DMA_DRIVER_OPS,
#endif
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789vw_ws13",
	.desc			= "Waveshare 1.3inch IPS LCD HAT (ST7789VW)",
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 14, 0)
	.date			= "20260101",
#endif
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ws13_of_match[] = {
	{ .compatible = "waveshare,ws13-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, ws13_of_match);

static const struct spi_device_id ws13_spi_id[] = {
	{ "ws13-lcd", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ws13_spi_id);

static int ws13_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &ws13_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset),
				     "Failed to get reset GPIO\n");

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc),
				     "Failed to get D/C GPIO\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);
	if (rotation != 0 && rotation != 90 &&
	    rotation != 180 && rotation != 270) {
		dev_warn(dev, "invalid rotation %u, using 0\n", rotation);
		rotation = 0;
	}

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	/* The HAT has no MISO wiring: forbid any register read-back */
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init(dbidev, &ws13_pipe_funcs, &ws13_mode,
				rotation);
	if (ret)
		return ret;

	/*
	 * Address-window offset into the 240x320 controller RAM.
	 * Gate axis reversed:
	 *   180 deg (MV=0,MY=1) -> RASET must start at line 80
	 *   270 deg (MV=1,MX=1) -> CASET must start at line 80
	 */
	switch (dbidev->rotation) {
	case 180:
		dbidev->top_offset = WS13_LINE_OFFSET;
		break;
	case 270:
		dbidev->left_offset = WS13_LINE_OFFSET;
		break;
	}

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

#if defined(WS13_FBDEV_CLIENT_SETUP)
	drm_client_setup(drm, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
	drm_fbdev_dma_setup(drm, 0);
#else
	drm_fbdev_generic_setup(drm, 0);
#endif

	dev_info(dev, "ST7789VW 240x240 initialised, rotation=%u, %u kHz\n",
		 dbidev->rotation, spi->max_speed_hz / 1000);
	return 0;
}

static void ws13_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void ws13_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver ws13_spi_driver = {
	.driver = {
		.name		= "st7789vw_ws13",
		.of_match_table	= ws13_of_match,
	},
	.id_table	= ws13_spi_id,
	.probe		= ws13_probe,
	.remove		= ws13_remove,
	.shutdown	= ws13_shutdown,
};
module_spi_driver(ws13_spi_driver);

MODULE_DESCRIPTION("DRM driver for Waveshare 1.3inch IPS LCD HAT (ST7789VW)");
MODULE_AUTHOR("Xen-alpha, Generated with Claude");
MODULE_LICENSE("GPL");
