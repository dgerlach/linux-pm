/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012-2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Vaibhav Bedia, Dave Gerlach
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

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/ti-emif-sram.h>
#include <linux/wkup_m3_ipc.h>

#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/smp_scu.h>
#include <asm/system_misc.h>

#include "clockdomain.h"
#include "cm33xx.h"
#include "common.h"
#include "pm.h"
#include "powerdomain.h"
#include "soc.h"

static struct powerdomain *cefuse_pwrdm, *gfx_pwrdm;
static void __iomem *scu_base;

static int (*am43xx_do_wfi_sram)(unsigned long unused);
static phys_addr_t am43xx_do_wfi_sram_phys;

#ifdef CONFIG_SUSPEND
static int am43xx_pm_suspend(void)
{
	int i, ret = 0;
	int status = 0;

	/* Try to put GFX to sleep */
	omap_set_pwrdm_state(gfx_pwrdm, PWRDM_POWER_OFF);

	scu_power_mode(scu_base, SCU_PM_POWEROFF);
	ret = cpu_suspend(0, am43xx_do_wfi_sram);
	scu_power_mode(scu_base, SCU_PM_NORMAL);

	status = pwrdm_read_pwrst(gfx_pwrdm);
	if (status != PWRDM_POWER_OFF)
		pr_err("PM: GFX domain did not transition\n");

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = wkup_m3_request_pm_status();

		switch (i) {
		case 0:
			pr_info("PM: Successfully put all powerdomains to target state\n");
			break;
		case 1:
			pr_err("PM: Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("PM: CM3 returned unknown result = %d\n", i);
			ret = -1;
		}
	}

	return ret;
}

static int am43xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
		ret = am43xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am43xx_pm_begin(suspend_state_t state)
{
	int ret = -EINVAL;

	switch (state) {
	case PM_SUSPEND_MEM:
		ret = wkup_m3_prepare_low_power(state);
		break;
	}

	return ret;
}

static void am43xx_pm_end(void)
{
	wkup_m3_finish_low_power();
}

static int am43xx_pm_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static const struct platform_suspend_ops am43xx_pm_ops = {
	.begin		= am43xx_pm_begin,
	.end		= am43xx_pm_end,
	.enter		= am43xx_pm_enter,
	.valid		= am43xx_pm_valid,
};
#endif /* CONFIG_SUSPEND */

/*
 * Push the minimal suspend-resume code to SRAM
 */
static int am43xx_push_sram_idle(void)
{
	struct device_node *np;
	struct gen_pool *sram_pool;
	phys_addr_t ocmcram_location;
	int ret;

	ret = ti_emif_copy_pm_function_table(&am43xx_emif_sram_table);
	if (ret) {
		pr_err("PM: Cannot copy emif functions to sram, no PM available\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "ti,omap4-mpu");

	if (!np) {
		pr_warn("PM: %s: Unable to find device node for mpu\n",
			__func__);
		return -ENODEV;
	}

	sram_pool = of_get_named_gen_pool(np, "sram", 0);

	if (!sram_pool) {
		pr_warn("PM: %s: Unable to allocate sram pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, am43xx_do_wfi_sz);
	if (!ocmcram_location)
		return -EINVAL;

	/* Save physical address to calculate resume offset during pm init */
	am43xx_do_wfi_sram_phys = gen_pool_virt_to_phys(sram_pool,
							ocmcram_location);
	am43xx_do_wfi_sram = (void *)fncpy((void *)ocmcram_location,
					   &am43xx_do_wfi,
					   am43xx_do_wfi_sz);
	return 0;
}

int __init am43xx_pm_init(void)
{
	int ret;
	void *resume_address;
	u32 temp;

	if (!soc_is_am43xx())
		return -ENODEV;

	gfx_pwrdm = pwrdm_lookup("gfx_pwrdm");

	if (!gfx_pwrdm) {
		pr_err("PM: Could not lookup GFX power domain\n");
		return -ENODEV;
	}

	(void)clkdm_for_each(omap_pm_clkdms_setup, NULL);

	/* CEFUSE domain can be turned off post bootup */
	cefuse_pwrdm = pwrdm_lookup("cefuse_pwrdm");
	if (cefuse_pwrdm)
		omap_set_pwrdm_state(cefuse_pwrdm, PWRDM_POWER_OFF);
	else
		pr_warn("PM: Failed to get cefuse_pwrdm\n");

	scu_base = ioremap(scu_a9_get_base(), SZ_256);
	if (!scu_base) {
		pr_err("PM: Cannot get SCU base addr");
		return -ENODEV;
	}

	ret = am43xx_push_sram_idle();
	if (ret)
		return ret;

	/* Physical resume address to be used by ROM code */
	resume_address = (void *)am43xx_do_wfi_sram_phys +
			 am43xx_resume_offset + 0x4;

	wkup_m3_set_resume_address(resume_address);

	temp = ti_emif_get_mem_type();
	if (temp < 0) {
		pr_err("PM: Cannot determine memory type, no PM available\n");
		return -ENODEV;
	}
	wkup_m3_set_mem_type(temp);

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am43xx_pm_ops);
#endif /* CONFIG_SUSPEND */

	return 0;
}
