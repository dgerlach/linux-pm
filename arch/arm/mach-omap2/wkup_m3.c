/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * Vaibhav Bedia <vaibhav.bedia@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>

#include "pm33xx.h"
#include "control.h"
#include "omap_device.h"
#include "soc.h"

struct wkup_m3_context {
	struct device	*dev;
	void __iomem	*code;
};

struct wkup_m3_context *wkup_m3;

int wkup_m3_copy_code(const u8 *data, size_t size)
{
	if (size > SZ_16K)
		return -ENOMEM;

	memcpy_toio(wkup_m3->code, data, size);

	return 0;
}

extern int am33xx_txev_handler();
/* have platforms do what they want in atomic context over here? */
static irqreturn_t wkup_m3_txev_handler(int irq, void *unused)
{
	am33xx_txev_eoi();

	/* callback to be executed in atomic context */
	/* return 0 implies IRQ_HANDLED else IRQ_NONE */
	am33xx_txev_handler();

	am33xx_txev_enable();

	return IRQ_HANDLED;
}

int wkup_m3_prepare()
{
	struct platform_device *pdev = to_platform_device(wkup_m3->dev);

	/* check that the code is loaded */
	omap_device_deassert_hardreset(pdev, "wkup_m3");

	return 0;
}

static int wkup_m3_probe(struct platform_device *pdev)
{
	int irq, ret = 0;
	struct resource *mem;

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "pm_runtime_get_sync() failed\n");
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(wkup_m3->dev, "no irq resource\n");
		ret = -ENXIO;
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(wkup_m3->dev, "no memory resource\n");
		ret = -ENXIO;
		goto err;
	}

	wkup_m3 = kzalloc(sizeof(struct wkup_m3_context), GFP_KERNEL);
	if (!wkup_m3) {
		pr_err("Memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	wkup_m3->dev = &pdev->dev;

	wkup_m3->code = devm_request_and_ioremap(wkup_m3->dev, mem);
	if (!wkup_m3->code) {
		dev_err(wkup_m3->dev, "could not ioremap\n");
		ret = -EADDRNOTAVAIL;
		goto err;
	}

	ret = devm_request_irq(wkup_m3->dev, irq, wkup_m3_txev_handler,
		  IRQF_DISABLED, "wkup_m3_txev", NULL);
	if (ret) {
		dev_err(wkup_m3->dev, "request_irq failed\n");
		goto err;
	}

err:
	return ret;
}

static int wkup_m3_remove(struct platform_device *pdev)
{

}

static struct of_device_id wkup_m3_dt_ids[] = {
	{ .compatible = "ti,am3353-wkup-m3" },
	{ }
};
MODULE_DEVICE_TABLE(of, wkup_m3_dt_ids);

static int wkup_m3_rpm_suspend(struct device *dev)
{
	return -EBUSY;
}

static int wkup_m3_rpm_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops wkup_m3_ops = {
	SET_RUNTIME_PM_OPS(wkup_m3_rpm_suspend, wkup_m3_rpm_resume, NULL)
};

static struct platform_driver wkup_m3_driver = {
	.probe		= wkup_m3_probe,
	.remove		= wkup_m3_remove,
	.driver		= {
		.name	= "wkup_m3",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(wkup_m3_dt_ids),
		.pm	= &wkup_m3_ops,
	},
};

static __init int wkup_m3_init(void)
{
	return platform_driver_register(&wkup_m3_driver);
}

static __exit void wkup_m3_exit(void)
{
	platform_driver_unregister(&wkup_m3_driver);
}
omap_postcore_initcall(wkup_m3_init);
module_exit(wkup_m3_exit);
