/*
 * Copyright (c) 2013 Linaro Ltd.
 * Copyright (c) 2013 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/of_address.h>
#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define SOC_AO_SCTRL_SC_MCU_SUBSYS_CTRL3_aob_io_sel18_sd_START  (10)
#define SOC_AO_SCTRL_SC_MCU_SUBSYS_CTRL3_ADDR			(0x40C)
#define SOC_AO_SCTRL_BASE_ADDR				(0xF7800000)
#define REG_SC_OFF_IOSIZE				PAGE_ALIGN(SZ_4K)

#define SDMMC_CCLK_MAX_24M     24000000
#define SDMMC_CCLK_MAX_25M     25000000
#define SDMMC_CCLK_MAX_48M     48000000
#define SDMMC_CCLK_MAX_50M     50000000
#define SDMMC_CCLK_MAX_80M     80000000
#define SDMMC_CCLK_MAX_96M     96000000
#define SDMMC_CCLK_MAX_100M    100000000
#define SDMMC_CCLK_MAX_150M    150000000
#define SDMMC_CCLK_MAX_180M    180000000
#define SDMMC_CCLK_MAX_200M    200000000

#define GET_FREQ(id, f1, f2) (id == MMC_SD ? f1 : f2)

static int switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci_priv_data *priv = mci_priv(slot->host);
	int min_uv, max_uv;
	unsigned int sd_io;
	int ret = -1;

	/* default to generic function */
        if (priv->id != MMC_SD)
		return ret;

	/*
	 * Program the voltage.  Note that some instances of dw_mmc may use
	 * the UHS_REG for this.  For other instances (like exynos) the UHS_REG
	 * does no harm but you need to set the regulator directly.  Try both.
	 */
	sd_io = 0x1 << SOC_AO_SCTRL_SC_MCU_SUBSYS_CTRL3_aob_io_sel18_sd_START;
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		sd_io = ~sd_io &
			readl(priv->ao_sysctrl +
				SOC_AO_SCTRL_SC_MCU_SUBSYS_CTRL3_ADDR);
		writel(sd_io,
			priv->ao_sysctrl +
			SOC_AO_SCTRL_SC_MCU_SUBSYS_CTRL3_ADDR);

		min_uv = 3000000;
		max_uv = 3000000;

	} else {
		min_uv = 1800000;
		max_uv = 1800000;
	}

	if (IS_ERR_OR_NULL(mmc->supply.vqmmc))
		return 0;

	ret = regulator_set_voltage(mmc->supply.vqmmc, min_uv, max_uv);
	if (ret) {
		dev_dbg(&mmc->class_dev, "Regulator set error %d: %d - %d\n",
				 ret, min_uv, max_uv);
		return ret;
	}

	ret = regulator_enable(mmc->supply.vqmmc);
	if (ret) {
		dev_dbg(&mmc->class_dev, "Regulator enable error %d \n", ret);
		return ret;
	}

	usleep_range(5000, 5500);

	return 0;

}

static void disable_boot(struct dw_mci *host)
{
	int timeout = 2000;
	unsigned int data;

	mci_writel(host, CTRL, SDMMC_CTRL_FIFO_RESET);
	mci_writel(host, CMD, SDMMC_CMD_START | SDMMC_CMD_DISABLE_BOOT);

	while(timeout--) {
		data = mci_readl(host, CMD) & 0x80000000;
		if (data == 0)
			return;
		mdelay(1);
	}

	dev_warn(host->dev, "failed to stop MMC\n");
}

static void dw_mci_k3_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	int id = mci_priv(host)->id;
	u32 freq;

	if (mci_priv(host)->timing == ios->timing)
		return;

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_24M, SDMMC_CCLK_MAX_25M);
		break;
	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_UHS_SDR25:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_48M, SDMMC_CCLK_MAX_50M);
		break;
	case MMC_TIMING_UHS_DDR50:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_100M, SDMMC_CCLK_MAX_50M);
		break;
	case MMC_TIMING_UHS_SDR50:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_96M, SDMMC_CCLK_MAX_100M);
		break;
	default:
		dev_err(host->dev, "ios timing %d not supported \n",
			ios->timing);
		return;
	}

	if (clk_set_rate(host->biu_clk, freq)) {
		dev_err(host->dev, "failed to set rate %uHz\n", freq);
		return;
	}

	host->bus_hz = clk_get_rate(host->biu_clk);
	mci_priv(host)->timing = ios->timing;
}

static int dw_mci_k3_init(struct dw_mci *host)
{
	struct device_node *np = host->dev->of_node;
	struct dw_mci_priv_data *priv;
	void __iomem *p;

	/* complete the device reset */
	if (of_find_property(np, "hisilicon,disable-boot", NULL))
		disable_boot(host);

	/* allocate private region */
	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(host->dev, "failed to alloc private data \n");
		return -ENOMEM;
	}

	priv->id = of_alias_get_id(np, "mshc");
	if (priv->id == MMC_SD) {
		p = ioremap_nocache(SOC_AO_SCTRL_BASE_ADDR, REG_SC_OFF_IOSIZE);
		if (!p) {
			devm_kfree(host->dev, priv);
			return -ENOMEM;
		}
		priv->ao_sysctrl = p;
	}

	priv->switch_voltage = switch_voltage;
	host->priv = priv;

	return 0;
}

static int dw_mci_k3_setup_clock(struct dw_mci *host)
{
	int bus_hz = 0;
	int ret;

	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk))
		dev_dbg(host->dev, "biu clock not available\n");
	else {
		clk_set_rate(host->biu_clk, SDMMC_CCLK_MAX_25M);
		ret = clk_prepare_enable(host->biu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable biu clock\n");
			return ret;
		}
		bus_hz = clk_get_rate(host->biu_clk);
	}

	host->ciu_clk = devm_clk_get(host->dev, "ciu");
	if (IS_ERR(host->ciu_clk)) {
		host->bus_hz = bus_hz ? bus_hz: host->pdata->bus_hz;
		dev_dbg(host->dev, "ciu clock not available\n");
	} else {
		if (host->pdata->bus_hz) {
			/* set biu clock */
			ret = clk_set_rate(host->biu_clk, host->pdata->bus_hz);
			if (ret)
				dev_warn(host->dev,
					 "Unable to set bus rate to %uHz\n",
					 host->pdata->bus_hz);
		}
		host->bus_hz = clk_get_rate(host->biu_clk);
	}

	return 0;
}

static const struct dw_mci_drv_data k3_drv_data = {
	.set_ios		= dw_mci_k3_set_ios,
	.init			= dw_mci_k3_init,
	.setup_clock		= dw_mci_k3_setup_clock,
};

static const struct of_device_id dw_mci_k3_match[] = {
        { .compatible = "hisilicon,hisi-dw-mshc", .data = &k3_drv_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_k3_match);

static int dw_mci_k3_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;

	match = of_match_node(dw_mci_k3_match, pdev->dev.of_node);
	drv_data = match->data;

	return dw_mci_pltfm_register(pdev, drv_data);
}

#ifdef CONFIG_PM_SLEEP
static int dw_mci_k3_suspend(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = dw_mci_suspend(host);
	if (!ret)
		clk_disable_unprepare(host->ciu_clk);

	return ret;
}

static int dw_mci_k3_resume(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(host->ciu_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable ciu clock\n");
		return ret;
	}

	return dw_mci_resume(host);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(dw_mci_k3_pmops, dw_mci_k3_suspend, dw_mci_k3_resume);

static struct platform_driver dw_mci_k3_pltfm_driver = {
	.probe		= dw_mci_k3_probe,
	.remove		= dw_mci_pltfm_remove,
	.driver		= {
		.name		= "dwmmc_k3",
		.of_match_table	= dw_mci_k3_match,
		.pm		= &dw_mci_k3_pmops,
	},
};

module_platform_driver(dw_mci_k3_pltfm_driver);

MODULE_DESCRIPTION("K3 Specific DW-MSHC Driver Extension");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwmmc-k3");
