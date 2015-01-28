/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/opp.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

/*
 * The length's determined by PINFUNC definition's length.
 * To check the length, see: arch/arm/boot/dts/imx6q-pinfunc.h
 * And the real pinctrl length should be (PINFUNC length + 1),
 * because there's a functional value behind each PINCTRL
 */
#define LEN_OF_PINCTRL	(sizeof(u32) * 6)

#define IMX6Q_C_ERR(fmt, ...) \
	pr_err("mach-imx6q.c ERROR: %s: " fmt, __func__, ##__VA_ARGS__)

static void imx6q_restart(char mode, const char *cmd)
{
	struct device_node *np;
	void __iomem *wdog_base;
	u32 wdog_source = 1; /* use WDOG1 default */
	unsigned int value;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpc");
	if (np)
		of_property_read_u32(np, "fsl,wdog-reset", &wdog_source);
	pr_info("Use WDOG%d as reset source\n", wdog_source);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-wdt");
	wdog_base = of_iomap(np, 0);
	if (!wdog_base)
		goto soft;

	imx_src_prepare_restart();

	if (wdog_source == 2) {
		/*
		 * Some boards  use WDOG2 to reset external pmic in bypass mode,
		 * so do WDOG2 reset here. Do not set SRS,since we will
		 * trigger external POR later. Use WDOG1 to reset in ldo-enable
		 * mode.
		 */
		np = of_find_compatible_node(np, NULL, "fsl,imx6q-wdt");
		wdog_base = of_iomap(np, 0);
		if (!wdog_base) {
			pr_warn("Not found wdt2, please check your dts!\n");
			goto soft;
		}
		value = 0x14;
	} else {
		value = (1 << 2);
	}
	/* enable wdog */
	writew_relaxed(value, wdog_base);
	/* write twice to ensure the request will not get ignored */
	writew_relaxed(value, wdog_base);

	/* wait for reset to assert ... */
	mdelay(500);

	pr_err("Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

soft:
	/* we'll take a jump through zero as a poor second */
	soft_restart(0);
}

static void remove_one_pin_from_node(const char *path,
			   const char *phandle_name,
			   const char *name,
			   u32 pin)
{
	struct device_node *np, *pinctrl;
	struct property *pbase;
	struct property *poldbase;
	u32 *psize;
	int i = 0, j = 0, k;

	np = of_find_node_by_path(path);
	if (!np) {
		IMX6Q_C_ERR("No such path: %s\n", path);
		return;
	}

	pinctrl = of_parse_phandle(np, phandle_name, 0);
	if (!phandle_name) {
		IMX6Q_C_ERR("No such phandle name: %s\n", phandle_name);
		return;
	}

	poldbase = of_find_property(pinctrl, name, NULL);
	if (!poldbase) {
		IMX6Q_C_ERR("No such property name: %s\n", name);
		return;
	}

	pbase = kzalloc(sizeof(*pbase)
			+ poldbase->length - 8, GFP_KERNEL);
	if (!pbase) {
		IMX6Q_C_ERR("No MEMORY to allocate new pbase\n");
		return;
	}

	psize = (u32 *)(pbase + 1);
	pbase->length = poldbase->length - LEN_OF_PINCTRL;
	pbase->name = kstrdup(poldbase->name, GFP_KERNEL);
	if (!pbase->name) {
		IMX6Q_C_ERR("Invalid pbase->name\n");
		kfree(pbase);
		return;
	}

	pbase->value = psize;
	for ( ; j < poldbase->length; i += LEN_OF_PINCTRL, j += LEN_OF_PINCTRL) {
		if (cpu_to_be32(pin) == *(u32 *)(poldbase->value + j)) {
			i -= LEN_OF_PINCTRL;
			continue;
		}
		for (k = 0; k < LEN_OF_PINCTRL; k += sizeof(u32))
			*(u32 *)(pbase->value + i + k) =
				*(u32 *)(poldbase->value + j + k);
	}

	if (i < j) {
		/* Only update the property if found the pin */
		of_update_property(pinctrl, pbase);
	} else {
		IMX6Q_C_ERR("No such pin 0x%x in %s\n", pin, path);
		kfree(pbase);
	}
}


/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, 0x0b, 0x8105);
		phy_write(phydev, 0x0c, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, 0x0b, 0x8104);
		phy_write(phydev, 0x0c, 0xf0f0);
		phy_write(phydev, 0x0b, 0x104);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static void __init imx6q_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
	}
}

static void __init reset_and_set_bootstrap(void)
{
		int err;
		unsigned int phy_reset, phy_poweron, clk125_en, phy_mode0, phy_mode1, phy_mode2, phy_mode3;
		int msec = 2;
		struct device_node *np;

		np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
		if (!np)
			return;

		phy_poweron = of_get_named_gpio(np, "phy-poweron", 0);
		phy_reset = of_get_named_gpio(np, "phy-reset-gpio", 0);
		clk125_en = of_get_named_gpio(np, "phy-clk125-en", 0);
		phy_mode0 = of_get_named_gpio(np, "phy-mode0", 0);
		phy_mode1 = of_get_named_gpio(np, "phy-mode1", 0);
		phy_mode2 = of_get_named_gpio(np, "phy-mode2", 0);
		phy_mode3 = of_get_named_gpio(np, "phy-mode3", 0);

		err = gpio_request_one(phy_poweron, GPIOF_OUT_INIT_LOW, "phy-poweron");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-poweron: %d\n", err);

		err = gpio_request_one(phy_reset, GPIOF_OUT_INIT_LOW, "phy-reset-gpio");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-reset-gpio: %d\n", err);

		err = gpio_request_one(clk125_en, GPIOF_OUT_INIT_HIGH, "phy-clk125-en");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-clk125-en: %d\n", err);

		err = gpio_request_one(phy_mode0, GPIOF_OUT_INIT_HIGH, "phy-mode0");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-mode0: %d\n", err);

		err = gpio_request_one(phy_mode1, GPIOF_OUT_INIT_HIGH, "phy-mode1");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-mode1: %d\n", err);

		err = gpio_request_one(phy_mode2, GPIOF_OUT_INIT_HIGH, "phy-mode2");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-mode2: %d\n", err);

		err = gpio_request_one(phy_mode3, GPIOF_OUT_INIT_HIGH, "phy-mode3");
		if (err) 
			pr_debug("FEC: failed to get gpio phy-mode3: %d\n", err);

		gpio_set_value(phy_poweron, 1);
		gpio_set_value(clk125_en, 1);
		gpio_set_value(phy_mode0, 1);
		gpio_set_value(phy_mode1, 1);
		gpio_set_value(phy_mode2, 1);
		gpio_set_value(phy_mode3, 1);
		gpio_set_value(phy_reset, 0);
		msleep(msec);
		gpio_set_value(phy_reset, 1);
		msleep(msec);
		gpio_free(clk125_en);
		gpio_free(phy_mode0);
		gpio_free(phy_mode1);
		gpio_free(phy_mode2);
		gpio_free(phy_mode3);
		gpio_free(phy_reset);

		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x06c);
		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x070);
		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x078);
		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x07c);
		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x080);
}

static void __init imx6q_1588_init(void)
{
	struct regmap *gpr;

	/* On uDOO need to reset fec and setup bootstrap to have phy addr to :6 */
	if (of_machine_is_compatible("fsl,imx6q-udoo")) {
		remove_one_pin_from_node("/soc/aips-bus@02000000/iomuxc@020e0000",
				"pinctrl-0", "fsl,pins", 0x248);
		reset_and_set_bootstrap();
	}

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

/*
 * Disable Hannstar LVDS panel CABC function.
 * This function turns the panel's backlight density automatically
 * according to the content shown on the panel which may cause
 * annoying unstable backlight issue.
 */
static void __init imx6q_lvds_cabc_init(void)
{
	struct device_node *np = NULL;
	int ret, lvds0_gpio, lvds1_gpio;

	np = of_find_node_by_name(NULL, "lvds_cabc_ctrl");
	if (!np)
		return;

	lvds0_gpio = of_get_named_gpio(np, "lvds0-gpios", 0);
	if (gpio_is_valid(lvds0_gpio)) {
		ret = gpio_request_one(lvds0_gpio, GPIOF_OUT_INIT_LOW,
				"LVDS0 CABC enable");
		if (ret)
			pr_warn("failed to request LVDS0 CABC gpio\n");
	}

	lvds1_gpio = of_get_named_gpio(np, "lvds1-gpios", 0);
	if (gpio_is_valid(lvds1_gpio)) {
		ret = gpio_request_one(lvds1_gpio, GPIOF_OUT_INIT_LOW,
				"LVDS1 CABC enable");
		if (ret)
			pr_warn("failed to request LVDS1 CABC gpio\n");
	}
}

static void __init imx6q_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	imx6q_enet_phy_init();

	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	imx_anatop_init();
	imx6_pm_init();
	imx6q_1588_init();
	imx6q_csi_mux_init();
	imx6q_lvds_cabc_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_1GHZ		0x2
#define OCOTP_CFG3_SPEED_850MHZ		0x1
#define OCOTP_CFG3_SPEED_800MHZ		0x0

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 1000000000Hz;
	 * 2b'01: 850000000Hz; -- i.MX6Q Only, exclusive with 1GHz.
	 * 2b'00: 800000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	if ((val & 0x3) < OCOTP_CFG3_SPEED_1P2GHZ)
		if (opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if ((val & 0x3) < OCOTP_CFG3_SPEED_1GHZ)
		if (opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 1 GHz OPP\n");
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_850MHZ ||
			(val & 0x3) == OCOTP_CFG3_SPEED_1GHZ)
			if (opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 850 MHz OPP\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6q_init_late(void)
{
	struct regmap *gpr;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT_MASK,
			IMX6Q_GPR1_GINT_ASSERT);

	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		imx6q_opp_init(&imx6q_cpufreq_pdev.dev);
		platform_device_register(&imx6q_cpufreq_pdev);
	}
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init imx6q_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());
}

static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_time	= imx6q_timer_init,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= imx6q_restart,
MACHINE_END
