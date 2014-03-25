/*
 * AMx3 Wkup M3 Remote Processor driver
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>

#include <linux/platform_data/wkup_m3.h>

#include "remoteproc_internal.h"

struct wkup_m3_rproc {
	struct rproc *rproc;
	struct platform_device *pdev;
};

static int wkup_m3_rproc_start(struct rproc *rproc)
{
	struct wkup_m3_rproc *m3_rproc = rproc->priv;
	struct platform_device *pdev = m3_rproc->pdev;
	struct device *dev = &pdev->dev;
	struct wkup_m3_platform_data *pdata = dev->platform_data;
	int ret;

	ret = pdata->deassert_reset(pdev, pdata->reset_name);
	if (ret) {
		dev_err(dev, "Unable to reset wkup_m3!\n");
		return -ENODEV;
	}

	return 0;
}

static int wkup_m3_rproc_stop(struct rproc *rproc)
{
	struct wkup_m3_rproc *m3_rproc = rproc->priv;
	struct platform_device *pdev = m3_rproc->pdev;
	struct device *dev = &pdev->dev;
	struct wkup_m3_platform_data *pdata = dev->platform_data;
	int ret;

	ret = pdata->assert_reset(pdev, pdata->reset_name);
	if (ret) {
		dev_err(dev, "Unable to assert reset of wkup_m3!\n");
		return -ENODEV;
	}
	return 0;
}

static struct rproc_ops wkup_m3_rproc_ops = {
	.start		= wkup_m3_rproc_start,
	.stop		= wkup_m3_rproc_stop,
};

static const struct of_device_id wkup_m3_rproc_of_match[] = {
	{
		.compatible = "ti,am3353-wkup-m3",
		.data = (void *)"am335x-pm-firmware.elf",
	},
	{},
};

static int wkup_m3_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *fw_name;
	struct wkup_m3_platform_data *pdata = dev->platform_data;
	struct wkup_m3_rproc *m3_rproc;
	const struct of_device_id *match;
	struct rproc *rproc;
	int ret;

	if (!(pdata && pdata->deassert_reset && pdata->assert_reset &&
	      pdata->reset_name)) {
		dev_err(dev, "Platform data missing!\n");
		return -ENODEV;
	}

	match = of_match_device(wkup_m3_rproc_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;
	fw_name = (char *)match->data;

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "pm_runtime_get_sync() failed\n");
		return ret;
	}

	rproc = rproc_alloc(dev, "wkup_m3", &wkup_m3_rproc_ops,
			    fw_name, sizeof(*m3_rproc));
	if (!rproc)
		return -ENOMEM;

	m3_rproc = rproc->priv;
	m3_rproc->rproc = rproc;
	m3_rproc->pdev = pdev;

	dev_set_drvdata(dev, rproc);

	/* Register as a remoteproc device */
	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err;
	}

	return 0;

err:
	rproc_put(rproc);
	pm_runtime_put_sync(&pdev->dev);
	return ret;
}

static int wkup_m3_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_put(rproc);
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int wkup_m3_rpm_suspend(struct device *dev)
{
	return -EBUSY;
}

static int wkup_m3_rpm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops wkup_m3_rproc_pm_ops = {
	SET_RUNTIME_PM_OPS(wkup_m3_rpm_suspend, wkup_m3_rpm_resume, NULL)
};

static struct platform_driver wkup_m3_rproc_driver = {
	.probe = wkup_m3_rproc_probe,
	.remove = wkup_m3_rproc_remove,
	.driver = {
		.name = "wkup_m3",
		.of_match_table = wkup_m3_rproc_of_match,
		.pm = &wkup_m3_rproc_pm_ops,
	},
};

module_platform_driver(wkup_m3_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("wkup m3 remote processor control driver");
