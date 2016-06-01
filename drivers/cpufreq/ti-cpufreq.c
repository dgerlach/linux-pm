/*
 * TI CPUFreq/OPP hw-supported driver
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *	 Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>

#define REVISION_MASK				0xF
#define REVISION_SHIFT				28

#define AM33XX_800M_ARM_MPU_MAX_FREQ		0x1E2F
#define AM43XX_600M_ARM_MPU_MAX_FREQ		0xFFA

#define DRA7_EFUSE_HAS_OD_MPU_OPP		11
#define DRA7_EFUSE_HAS_HIGH_MPU_OPP		15
#define DRA7_EFUSE_HAS_ALL_MPU_OPP		23

#define DRA7_EFUSE_NOM_MPU_OPP			BIT(0)
#define DRA7_EFUSE_OD_MPU_OPP			BIT(1)
#define DRA7_EFUSE_HIGH_MPU_OPP			BIT(2)

#define VERSION_COUNT				2

static struct ti_cpufreq_data {
	struct device *cpu;
	struct regmap *opp_efuse;
	struct regmap *revision;
} opp_data;

const static struct ti_cpufreq_soc_data {
	unsigned long (*efuse_xlate)(unsigned long efuse);
	unsigned long efuse_fallback;
} *soc_data;

static unsigned long amx3_efuse_xlate(unsigned long efuse)
{
	if (!efuse)
		efuse = soc_data->efuse_fallback;
	/* AM335x and AM437x use "OPP disable" bits, so invert */
	return ~efuse;
}

static unsigned long dra7_efuse_xlate(unsigned long efuse)
{
	unsigned long calculated_efuse = DRA7_EFUSE_NOM_MPU_OPP;

	/*
	 * The efuse on dra7 and am57 parts contains a specific
	 * value indicating the highest available OPP.
	 */

	switch (efuse) {
	case DRA7_EFUSE_HAS_ALL_MPU_OPP:
	case DRA7_EFUSE_HAS_HIGH_MPU_OPP:
		calculated_efuse |= DRA7_EFUSE_HIGH_MPU_OPP;
	case DRA7_EFUSE_HAS_OD_MPU_OPP:
		calculated_efuse |= DRA7_EFUSE_OD_MPU_OPP;
	}

	return calculated_efuse;
}

static struct ti_cpufreq_soc_data am3x_soc_data = {
	.efuse_xlate = amx3_efuse_xlate,
	.efuse_fallback = AM33XX_800M_ARM_MPU_MAX_FREQ,
};

static struct ti_cpufreq_soc_data am4x_soc_data = {
	.efuse_xlate = amx3_efuse_xlate,
	.efuse_fallback = AM43XX_600M_ARM_MPU_MAX_FREQ,
};

static struct ti_cpufreq_soc_data dra7_soc_data = {
	.efuse_xlate = dra7_efuse_xlate,
};

/**
 * ti_cpufreq_get_efuse() - Parse and return efuse value present on SoC
 * @efuse_value: Set to the value parsed from efuse
 *
 * Returns error code if efuse not read properly.
 */
static int ti_cpufreq_get_efuse(u32 *efuse_value)
{
	struct device *dev = opp_data.cpu;
	struct device_node *np = dev->of_node;
	unsigned int efuse_offset;
	u32 efuse, efuse_mask, efuse_shift, vals[4];
	int ret;

	ret = of_property_read_u32_array(np, "ti,syscon-efuse", vals, 4);
	if (ret) {
		dev_err(dev, "ti,syscon-efuse cannot be read %s: %d\n", np->full_name,
			ret);
		return ret;
	}

	efuse_offset = vals[1];
	efuse_mask = vals[2];
	efuse_shift = vals[3];

	ret = regmap_read(opp_data.opp_efuse, efuse_offset, &efuse);
	if (ret) {
		dev_err(dev,
			"Failed to read the efuse value from syscon: %d\n",
			ret);
		return ret;
	}

	efuse = (efuse & efuse_mask) >> efuse_shift;

	*efuse_value = soc_data->efuse_xlate(efuse);

	return 0;
}

/**
 * ti_cpufreq_get_rev() - Parse and return rev value present on SoC
 * @revision_value: Set to the value parsed from revision register
 *
 * Returns error code if revision not read properly.
 */
static int ti_cpufreq_get_rev(u32 *revision_value)
{
	struct device *dev = opp_data.cpu;
	struct device_node *np = dev->of_node;
	unsigned int revision_offset;
	u32 revision;
	int ret;

	ret = of_property_read_u32_index(np, "ti,syscon-rev",
					 1, &revision_offset);
	if (ret) {
		dev_err(dev,
			"No revision offset provided %s [%d]\n",
			np->full_name, ret);
		return ret;
	}

	ret = regmap_read(opp_data.revision, revision_offset, &revision);
	if (ret) {
		dev_err(dev,
			"Failed to read the revision number from syscon: %d\n",
			ret);
		return ret;
	}

	*revision_value = BIT((revision >> REVISION_SHIFT) & REVISION_MASK);

	return 0;
}

static int ti_cpufreq_setup_syscon_registers(void)
{
	struct device *dev = opp_data.cpu;
	struct device_node *np = dev->of_node;

	opp_data.opp_efuse = syscon_regmap_lookup_by_phandle(np,
							"ti,syscon-efuse");
	if (IS_ERR(opp_data.opp_efuse)) {
		dev_dbg(dev,  "\"ti,syscon-efuse\" is missing, cannot use OPPv2 table.\n");
		return PTR_ERR(opp_data.opp_efuse);
	}

	opp_data.revision = syscon_regmap_lookup_by_phandle(np,
							"ti,syscon-rev");
	if (IS_ERR(opp_data.revision)) {
		dev_dbg(dev,  "\"ti,syscon-rev\" is missing, cannot use OPPv2 table.\n");
		return PTR_ERR(opp_data.revision);
	}

	return 0;
}

static const struct of_device_id compatible_machine_match[] = {
	{ .compatible = "ti,am33xx", .data = &am3x_soc_data, },
	{ .compatible = "ti,am4372", .data = &am4x_soc_data, },
	{ .compatible = "ti,dra7", .data = &dra7_soc_data },
	{},
};

static int __init ti_cpufreq_init(void)
{
	int ret;
	u32 version[VERSION_COUNT];
	struct device_node *root = of_find_node_by_path("/");
	const struct of_device_id *match;

	if (!root)
		return -ENODEV;

	match = of_match_node(compatible_machine_match, root);
	if (!match )
		return -ENODEV;

	soc_data = match->data;

	opp_data.cpu = get_cpu_device(0);
	if (!opp_data.cpu) {
		pr_err("%s: Failed to get device for CPU0\n", __func__);
		return -ENODEV;
	}

	if (!of_get_property(opp_data.cpu->of_node, "operating-points-v2",
			     NULL)) {
		dev_info(opp_data.cpu, "OPP-v2 not supported, cpufreq-dt will attempt to use legacy tables.\n");
		goto register_cpufreq_dt;
	}

	ret = ti_cpufreq_setup_syscon_registers();
	if (ret)
		goto register_cpufreq_dt;

	/*
	 * OPPs determine whether or not they are supported based on
	 * two metrics:
	 *	0 - SoC Revision
	 *	1 - eFuse value
	 */
	ret = ti_cpufreq_get_rev(&version[0]);
	if (ret)
		return ret;

	ret = ti_cpufreq_get_efuse(&version[1]);
	if (ret)
		return ret;

	ret = dev_pm_opp_set_supported_hw(opp_data.cpu, version, VERSION_COUNT);
	if (ret) {
		dev_err(opp_data.cpu, "Failed to set supported hardware\n");
		return ret;
	}

register_cpufreq_dt:
	platform_device_register_simple("cpufreq-dt", -1, NULL, 0);

	return 0;
}
module_init(ti_cpufreq_init);

MODULE_DESCRIPTION("TI CPUFreq/OPP hw-supported driver");
MODULE_AUTHOR("Dave Gerlach <d-gerlach@ti.com>");
MODULE_LICENSE("GPL v2");
