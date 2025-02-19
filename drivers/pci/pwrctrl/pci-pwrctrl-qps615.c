// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-pwrctrl.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "../pci.h"

#define QPS615_GPIO_CONFIG		0x801208
#define QPS615_RESET_GPIO		0x801210

#define QPS615_BUS_CONTROL		0x801014

#define QPS615_PORT_L0S_DELAY		0x82496c
#define QPS615_PORT_L1_DELAY		0x824970

#define QPS615_EMBEDDED_ETH_DELAY	0x8200d8
#define QPS615_ETH_L1_DELAY_MASK	GENMASK(27, 18)
#define QPS615_ETH_L1_DELAY_VALUE(x)	FIELD_PREP(QPS615_ETH_L1_DELAY_MASK, x)
#define QPS615_ETH_L0S_DELAY_MASK	GENMASK(17, 13)
#define QPS615_ETH_L0S_DELAY_VALUE(x)	FIELD_PREP(QPS615_ETH_L0S_DELAY_MASK, x)

#define QPS615_NFTS_2_5_GT		0x824978
#define QPS615_NFTS_5_GT		0x82497c

#define QPS615_PORT_LANE_ACCESS_ENABLE	0x828000

#define QPS615_PHY_RATE_CHANGE_OVERRIDE	0x828040
#define QPS615_PHY_RATE_CHANGE		0x828050

#define QPS615_TX_MARGIN		0x828234

#define QPS615_DFE_ENABLE		0x828a04
#define QPS615_DFE_EQ0_MODE		0x828a08
#define QPS615_DFE_EQ1_MODE		0x828a0c
#define QPS615_DFE_EQ2_MODE		0x828a14
#define QPS615_DFE_PD_MASK		0x828254

#define QPS615_PORT_SELECT		0x82c02c
#define QPS615_PORT_ACCESS_ENABLE	0x82c030

#define QPS615_POWER_CONTROL		0x82b09c
#define QPS615_POWER_CONTROL_OVREN	0x82b2c8

#define QPS615_FREQ_125_MHZ		125000000
#define QPS615_FREQ_250_MHZ		250000000

#define QPS615_GPIO_MASK		0xfffffff3

struct qps615_pwrctrl_reg_setting {
	unsigned int offset;
	unsigned int val;
};

enum qps615_pwrctrl_ports {
	QPS615_USP,
	QPS615_DSP1,
	QPS615_DSP2,
	QPS615_DSP3,
	QPS615_ETHERNET,
	QPS615_MAX
};

struct qps615_pwrctrl_cfg {
	u32 l0s_delay;
	u32 l1_delay;
	u32 tx_amp;
	u32 nfts;
	bool disable_dfe;
	bool disable_port;
	bool axi_freq_125;
};

#define QPS615_PWRCTRL_MAX_SUPPLY	6

struct qps615_pwrctrl_ctx {
	struct regulator_bulk_data supplies[QPS615_PWRCTRL_MAX_SUPPLY];
	struct qps615_pwrctrl_cfg cfg[QPS615_MAX];
	struct gpio_desc *reset_gpio;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct pci_pwrctrl pwrctrl;
};

/*
 * downstream port power off sequence, hardcoding the address
 * as we don't know register names for these register offsets.
 */
static const struct qps615_pwrctrl_reg_setting common_pwroff_seq[] = {
	{0x82900c, 0x1},
	{0x829010, 0x1},
	{0x829018, 0x0},
	{0x829020, 0x1},
	{0x82902c, 0x1},
	{0x829030, 0x1},
	{0x82903c, 0x1},
	{0x829058, 0x0},
	{0x82905c, 0x1},
	{0x829060, 0x1},
	{0x8290cc, 0x1},
	{0x8290d0, 0x1},
	{0x8290d8, 0x1},
	{0x8290e0, 0x1},
	{0x8290e8, 0x1},
	{0x8290ec, 0x1},
	{0x8290f4, 0x1},
	{0x82910c, 0x1},
	{0x829110, 0x1},
	{0x829114, 0x1},
};

static const struct qps615_pwrctrl_reg_setting dsp1_pwroff_seq[] = {
	{QPS615_PORT_ACCESS_ENABLE, 0x2},
	{QPS615_PORT_LANE_ACCESS_ENABLE, 0x3},
	{QPS615_POWER_CONTROL, 0x014f4804},
	{QPS615_POWER_CONTROL_OVREN, 0x1},
	{QPS615_PORT_ACCESS_ENABLE, 0x4},
};

static const struct qps615_pwrctrl_reg_setting dsp2_pwroff_seq[] = {
	{QPS615_PORT_ACCESS_ENABLE, 0x8},
	{QPS615_PORT_LANE_ACCESS_ENABLE, 0x1},
	{QPS615_POWER_CONTROL, 0x014f4804},
	{QPS615_POWER_CONTROL_OVREN, 0x1},
	{QPS615_PORT_ACCESS_ENABLE, 0x8},
};

/*
 * Since all transfers are initiated by the probe, no locks are necessary,
 * ensuring there are no concurrent calls.
 */
static int qps615_pwrctrl_i2c_write(struct i2c_client *client,
				   u32 reg_addr, u32 reg_val)
{
	struct i2c_msg msg;
	u8 msg_buf[7];
	int ret;

	msg.addr = client->addr;
	msg.len = 7;
	msg.flags = 0;

	/* Big Endian for reg addr */
	put_unaligned_be24(reg_addr, &msg_buf[0]);

	/* Little Endian for reg val */
	put_unaligned_le32(reg_val, &msg_buf[3]);

	msg.buf = msg_buf;
	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : ret;
}

static int qps615_pwrctrl_i2c_read(struct i2c_client *client,
				  u32 reg_addr, u32 *reg_val)
{
	struct i2c_msg msg[2];
	u8 wr_data[3];
	u32 rd_data;
	int ret;

	msg[0].addr = client->addr;
	msg[0].len = 3;
	msg[0].flags = 0;

	/* Big Endian for reg addr */
	put_unaligned_be24(reg_addr, &wr_data[0]);

	msg[0].buf = wr_data;

	msg[1].addr = client->addr;
	msg[1].len = 4;
	msg[1].flags = I2C_M_RD;

	msg[1].buf = (u8 *)&rd_data;

	ret = i2c_transfer(client->adapter, &msg[0], 2);
	if (ret == 2) {
		*reg_val = get_unaligned_le32(&rd_data);
		return 0;
	}

	/* If only one message successfully completed, return -ENODEV */
	return ret == 1 ? -ENODEV : ret;
}

static int qps615_pwrctrl_i2c_bulk_write(struct i2c_client *client,
					const struct qps615_pwrctrl_reg_setting *seq, int len)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = qps615_pwrctrl_i2c_write(client, seq[i].offset, seq[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int qps615_pwrctrl_disable_port(struct qps615_pwrctrl_ctx *ctx,
				      enum qps615_pwrctrl_ports port)
{
	const struct qps615_pwrctrl_reg_setting *seq;
	int ret, len;

	if (port == QPS615_DSP1) {
		seq = dsp1_pwroff_seq;
		len = ARRAY_SIZE(dsp1_pwroff_seq);
	} else {
		seq = dsp2_pwroff_seq;
		len = ARRAY_SIZE(dsp2_pwroff_seq);
	}

	ret = qps615_pwrctrl_i2c_bulk_write(ctx->client, seq, len);
	if (ret)
		return ret;

	return qps615_pwrctrl_i2c_bulk_write(ctx->client,
					    common_pwroff_seq, ARRAY_SIZE(common_pwroff_seq));
}

static int qps615_pwrctrl_set_l0s_l1_entry_delay(struct qps615_pwrctrl_ctx *ctx,
						enum qps615_pwrctrl_ports port, bool is_l1, u32 ns)
{
	u32 rd_val, units, mask;
	int ret;

	/* convert to units of 256ns */
	units = ns / 256;

	if (port == QPS615_ETHERNET) {
		ret = qps615_pwrctrl_i2c_read(ctx->client, QPS615_EMBEDDED_ETH_DELAY, &rd_val);
		if (ret)
			return ret;
		mask = is_l1 ? QPS615_ETH_L1_DELAY_MASK : QPS615_ETH_L0S_DELAY_MASK;
		rd_val = u32_replace_bits(rd_val, units, mask);
		return qps615_pwrctrl_i2c_write(ctx->client, QPS615_EMBEDDED_ETH_DELAY, rd_val);
	}

	ret = qps615_pwrctrl_i2c_write(ctx->client, QPS615_PORT_SELECT, BIT(port));
	if (ret)
		return ret;

	return qps615_pwrctrl_i2c_write(ctx->client,
				       is_l1 ? QPS615_PORT_L1_DELAY : QPS615_PORT_L0S_DELAY, units);
}

static int qps615_pwrctrl_set_tx_amplitude(struct qps615_pwrctrl_ctx *ctx,
					  enum qps615_pwrctrl_ports port, u32 amp)
{
	int port_access;

	switch (port) {
	case QPS615_USP:
		port_access = 0x1;
		break;
	case QPS615_DSP1:
		port_access = 0x2;
		break;
	case QPS615_DSP2:
		port_access = 0x8;
		break;
	default:
		return -EINVAL;
	};

	struct qps615_pwrctrl_reg_setting tx_amp_seq[] = {
		{QPS615_PORT_ACCESS_ENABLE, port_access},
		{QPS615_PORT_LANE_ACCESS_ENABLE, 0x3},
		{QPS615_TX_MARGIN, amp},
	};

	return qps615_pwrctrl_i2c_bulk_write(ctx->client, tx_amp_seq, ARRAY_SIZE(tx_amp_seq));
}

static int qps615_pwrctrl_disable_dfe(struct qps615_pwrctrl_ctx *ctx,
				     enum qps615_pwrctrl_ports port)
{
	int port_access, lane_access = 0x3;
	u32 phy_rate = 0x21;

	switch (port) {
	case QPS615_USP:
		phy_rate = 0x1;
		port_access = 0x1;
		break;
	case QPS615_DSP1:
		port_access = 0x2;
		break;
	case QPS615_DSP2:
		port_access = 0x8;
		lane_access = 0x1;
		break;
	default:
		return -EINVAL;
	};

	struct qps615_pwrctrl_reg_setting disable_dfe_seq[] = {
		{QPS615_PORT_ACCESS_ENABLE, port_access},
		{QPS615_PORT_LANE_ACCESS_ENABLE, lane_access},
		{QPS615_DFE_ENABLE, 0x0},
		{QPS615_DFE_EQ0_MODE, 0x411},
		{QPS615_DFE_EQ1_MODE, 0x11},
		{QPS615_DFE_EQ2_MODE, 0x11},
		{QPS615_DFE_PD_MASK, 0x7},
		{QPS615_PHY_RATE_CHANGE_OVERRIDE, 0x10},
		{QPS615_PHY_RATE_CHANGE, phy_rate},
		{QPS615_PHY_RATE_CHANGE, 0x0},
		{QPS615_PHY_RATE_CHANGE_OVERRIDE, 0x0},
	};

	return qps615_pwrctrl_i2c_bulk_write(ctx->client,
					    disable_dfe_seq, ARRAY_SIZE(disable_dfe_seq));
}

static int qps615_pwrctrl_set_nfts(struct qps615_pwrctrl_ctx *ctx,
				  enum qps615_pwrctrl_ports port, u32 nfts)
{
	int ret;
	struct qps615_pwrctrl_reg_setting nfts_seq[] = {
		{QPS615_NFTS_2_5_GT, nfts},
		{QPS615_NFTS_5_GT, nfts},
	};

	ret =  qps615_pwrctrl_i2c_write(ctx->client, QPS615_PORT_SELECT, BIT(port));
	if (ret)
		return ret;

	return qps615_pwrctrl_i2c_bulk_write(ctx->client, nfts_seq, ARRAY_SIZE(nfts_seq));
}

static int qps615_pwrctrl_assert_deassert_reset(struct qps615_pwrctrl_ctx *ctx, bool deassert)
{
	int ret, val;

	ret = qps615_pwrctrl_i2c_write(ctx->client, QPS615_GPIO_CONFIG, QPS615_GPIO_MASK);
	if (ret)
		return ret;

	val = deassert ? 0xc : 0;

	return qps615_pwrctrl_i2c_write(ctx->client, QPS615_RESET_GPIO, val);
}

static int qps615_pwrctrl_parse_device_dt(struct qps615_pwrctrl_ctx *ctx, struct device_node *node,
					 enum qps615_pwrctrl_ports port)
{
	struct qps615_pwrctrl_cfg *cfg;
	u32 axi_freq = 0;
	int ret;

	cfg = &ctx->cfg[port];

	if (!of_device_is_available(node)) {
		cfg->disable_port = true;
		return 0;
	};

	ret = of_property_read_u32(node, "qcom,axi-clk-freq-hz", &axi_freq);
	if (ret && ret != -EINVAL)
		return ret;
	else if (axi_freq && (axi_freq != QPS615_FREQ_125_MHZ || axi_freq != QPS615_FREQ_250_MHZ))
		return -EINVAL;
	else if (axi_freq == QPS615_FREQ_125_MHZ)
		cfg->axi_freq_125 = true;

	ret = of_property_read_u32(node, "qcom,l0s-entry-delay-ns", &cfg->l0s_delay);
	if (ret && ret != -EINVAL)
		return ret;

	ret = of_property_read_u32(node, "qcom,l1-entry-delay-ns", &cfg->l1_delay);
	if (ret && ret != -EINVAL)
		return ret;

	ret = of_property_read_u32(node, "qcom,tx-amplitude-millivolt", &cfg->tx_amp);
	if (ret && ret != -EINVAL)
		return ret;

	ret = of_property_read_u32(node, "qcom,nfts", &cfg->nfts);
	if (ret && ret != -EINVAL)
		return ret;

	cfg->disable_dfe = of_property_read_bool(node, "qcom,no-dfe-support");

	return 0;
}

static void qps615_pwrctrl_power_off(struct qps615_pwrctrl_ctx *ctx)
{
	gpiod_set_value(ctx->reset_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int qps615_pwrctrl_power_on(struct qps615_pwrctrl_ctx *ctx)
{
	struct qps615_pwrctrl_cfg *cfg;
	int ret, i;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return dev_err_probe(ctx->pwrctrl.dev, ret, "cannot enable regulators\n");

	gpiod_set_value(ctx->reset_gpio, 0);

	 /* wait for the internal osc frequency to stablise */
	usleep_range(10000, 10500);

	ret = qps615_pwrctrl_assert_deassert_reset(ctx, false);
	if (ret)
		goto out;

	if (ctx->cfg[QPS615_USP].axi_freq_125) {
		ret = qps615_pwrctrl_i2c_write(ctx->client, QPS615_BUS_CONTROL, BIT(16));
		if (ret)
			dev_err(ctx->pwrctrl.dev, "Setting AXI clk freq failed %d\n", ret);
	}

	for (i = 0; i < QPS615_MAX; i++) {
		cfg = &ctx->cfg[i];
		if (cfg->disable_port) {
			ret = qps615_pwrctrl_disable_port(ctx, i);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Disabling port failed\n");
				goto out;
			}
		}

		if (cfg->l0s_delay) {
			ret = qps615_pwrctrl_set_l0s_l1_entry_delay(ctx, i, false, cfg->l0s_delay);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Setting L0s entry delay failed\n");
				goto out;
			}
		}

		if (cfg->l1_delay) {
			ret = qps615_pwrctrl_set_l0s_l1_entry_delay(ctx, i, true, cfg->l1_delay);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Setting L1 entry delay failed\n");
				goto out;
			}
		}

		if (cfg->tx_amp) {
			ret = qps615_pwrctrl_set_tx_amplitude(ctx, i, cfg->tx_amp);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Setting Tx amplitube failed\n");
				goto out;
			}
		}

		if (cfg->nfts) {
			ret = qps615_pwrctrl_set_nfts(ctx, i, cfg->nfts);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Setting nfts failed\n");
				goto out;
			}
		}

		if (cfg->disable_dfe) {
			ret = qps615_pwrctrl_disable_dfe(ctx, i);
			if (ret) {
				dev_err(ctx->pwrctrl.dev, "Disabling DFE failed\n");
				goto out;
			}
		}
	}

	ret = qps615_pwrctrl_assert_deassert_reset(ctx, true);
	if (!ret)
		return 0;

out:
	qps615_pwrctrl_power_off(ctx);
	return ret;
}

static int qps615_pwrctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	enum qps615_pwrctrl_ports port;
	struct qps615_pwrctrl_ctx *ctx;
	int ret, addr;

	bridge = pci_find_host_bridge(to_pci_dev(dev->parent)->bus);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ret = of_property_read_u32_index(pdev->dev.of_node, "i2c-parent", 1, &addr);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read i2c-parent property\n");

	ctx->adapter = of_find_i2c_adapter_by_node(of_parse_phandle(dev->of_node, "i2c-parent", 0));
	of_node_put(dev->of_node);
	if (!ctx->adapter)
		return dev_err_probe(dev, -EPROBE_DEFER, "Failed to find I2C adapter\n");

	ctx->client = i2c_new_dummy_device(ctx->adapter, addr);
	if (IS_ERR(ctx->client)) {
		dev_err(dev, "Failed to create I2C client\n");
		i2c_put_adapter(ctx->adapter);
		return PTR_ERR(ctx->client);
	}

	ctx->supplies[0].supply = "vddc";
	ctx->supplies[1].supply = "vdd18";
	ctx->supplies[2].supply = "vdd09";
	ctx->supplies[3].supply = "vddio1";
	ctx->supplies[4].supply = "vddio2";
	ctx->supplies[5].supply = "vddio18";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret) {
		dev_err_probe(dev, ret,
			      "failed to get supply regulator\n");
		goto remove_i2c;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "failed to get reset GPIO\n");
		goto remove_i2c;
	}

	platform_set_drvdata(pdev, ctx);

	pci_pwrctrl_init(&ctx->pwrctrl, dev);

	port = QPS615_USP;
	ret = qps615_pwrctrl_parse_device_dt(ctx, pdev->dev.of_node, port);
	if (ret) {
		dev_err(dev, "failed to parse device tree properties: %d\n", ret);
		goto remove_i2c;
	}

	/*
	 * Downstream ports are always children of the upstream port.
	 * The first node represents DSP1, the second node represents DSP2, and so on.
	 */
	for_each_child_of_node_scoped(pdev->dev.of_node, child) {
		ret = qps615_pwrctrl_parse_device_dt(ctx, child, port++);
		if (ret)
			break;
		/* Embedded ethernet device are under DSP3 */
		if (port == QPS615_DSP3)
			for_each_child_of_node_scoped(child, child1) {
				ret = qps615_pwrctrl_parse_device_dt(ctx, child1, port++);
				if (ret)
					break;
			}
	}
	if (ret) {
		dev_err(dev, "failed to parse device tree properties: %d\n", ret);
		goto remove_i2c;
	}

	if (bridge->ops->stop_link)
		bridge->ops->stop_link(to_pci_dev(dev->parent)->bus);

	ret = qps615_pwrctrl_power_on(ctx);
	if (ret)
		goto remove_i2c;

	if (bridge->ops->start_link) {
		ret = bridge->ops->start_link(to_pci_dev(dev->parent)->bus);
		if (ret)
			goto power_off;
	}

	ret = devm_pci_pwrctrl_device_set_ready(dev, &ctx->pwrctrl);
	if (ret)
		goto power_off;

	return 0;

power_off:
	qps615_pwrctrl_power_off(ctx);
remove_i2c:
	i2c_unregister_device(ctx->client);
	i2c_put_adapter(ctx->adapter);
	return ret;
}

static void qps615_pwrctrl_remove(struct platform_device *pdev)
{
	struct qps615_pwrctrl_ctx *ctx = platform_get_drvdata(pdev);

	qps615_pwrctrl_power_off(ctx);
	i2c_unregister_device(ctx->client);
	i2c_put_adapter(ctx->adapter);
}

static const struct of_device_id qps615_pwrctrl_of_match[] = {
	{ .compatible = "pci1179,0623"},
	{ }
};
MODULE_DEVICE_TABLE(of, qps615_pwrctrl_of_match);

static struct platform_driver qps615_pwrctrl_driver = {
	.driver = {
		.name = "pwrctrl-qps615",
		.of_match_table = qps615_pwrctrl_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = qps615_pwrctrl_probe,
	.remove = qps615_pwrctrl_remove,
};
module_platform_driver(qps615_pwrctrl_driver);

MODULE_AUTHOR("Krishna chaitanya chundru <quic_krichai@quicinc.com>");
MODULE_DESCRIPTION("Qualcomm QPS615 power control driver");
MODULE_LICENSE("GPL");
