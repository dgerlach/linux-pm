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
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/completion.h>
#include <linux/module.h>
#include "../plat-omap/include/plat/mailbox.h"
#include <linux/interrupt.h>
#include <linux/ti_emif.h>

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/sizes.h>
#include <asm/fncpy.h>
#include <asm/system_misc.h>

#include "pm.h"
#include "cm33xx.h"
#include "pm33xx.h"
#include "control.h"
#include "common.h"
#include "clockdomain.h"
#include "powerdomain.h"
#include "omap_hwmod.h"
#include "omap_device.h"
#include "soc.h"
#include "sram.h"

static void __iomem *am33xx_emif_base;
static struct powerdomain *cefuse_pwrdm, *gfx_pwrdm, *per_pwrdm;
static struct clockdomain *gfx_l4ls_clkdm;

struct wakeup_src wakeups[] = {
	{.irq_nr = 35,	.src = "USB0_PHY"},
	{.irq_nr = 36,	.src = "USB1_PHY"},
	{.irq_nr = 40,	.src = "I2C0"},
	{.irq_nr = 41,	.src = "RTC Timer"},
	{.irq_nr = 42,	.src = "RTC Alarm"},
	{.irq_nr = 43,	.src = "Timer0"},
	{.irq_nr = 44,	.src = "Timer1"},
	{.irq_nr = 45,	.src = "UART"},
	{.irq_nr = 46,	.src = "GPIO0"},
	{.irq_nr = 48,	.src = "MPU_WAKE"},
	{.irq_nr = 49,	.src = "WDT0"},
	{.irq_nr = 50,	.src = "WDT1"},
	{.irq_nr = 51,	.src = "ADC_TSC"},
};

struct forced_standby_module am33xx_mod[] = {
	{.oh_name = "usb_otg_hs"},
	{.oh_name = "tptc0"},
	{.oh_name = "tptc1"},
	{.oh_name = "tptc2"},
	{.oh_name = "cpgmac0"},
};

#ifdef CONFIG_SUSPEND

static struct am33xx_pm_context *am33xx_pm;

static DECLARE_COMPLETION(am33xx_pm_sync);

static void (*am33xx_do_wfi_sram)(struct am33xx_suspend_params *);

static struct am33xx_suspend_params susp_params;

static int am33xx_do_sram_idle(long unsigned int unused)
{
	am33xx_do_wfi_sram(&susp_params);
	return 0;
}

static int am33xx_pm_suspend(void)
{
	int i, j, ret = 0;

	int status = 0;
	struct platform_device *pdev;
	struct omap_device *od;

	/*
	 * By default the following IPs do not have MSTANDBY asserted
	 * which is necessary for PER domain transition. If the drivers
	 * are not compiled into the kernel HWMOD code will not change the
	 * state of the IPs if the IP was not never enabled. To ensure
	 * that there no issues with or without the drivers being compiled
	 * in the kernel, we forcefully put these IPs to idle.
	 */
	for (i = 0; i < ARRAY_SIZE(am33xx_mod); i++) {
		pdev = to_platform_device(am33xx_mod[i].dev);
		od = to_omap_device(pdev);
		if (od->_driver_status != BUS_NOTIFY_BOUND_DRIVER) {
			omap_device_enable_hwmods(od);
			omap_device_idle_hwmods(od);
		}
	}

	/* Try to put GFX to sleep */
	omap_set_pwrdm_state(gfx_pwrdm, PWRDM_POWER_OFF);
	ret = cpu_suspend(0, am33xx_do_sram_idle);

	status = pwrdm_read_prev_pwrst(gfx_pwrdm);
	if (status != PWRDM_POWER_OFF)
		pr_err("GFX domain did not transition\n");
	else
		pr_info("GFX domain entered low power state\n");

	/*
	 * BUG: GFX_L4LS clock domain needs to be woken up to
	 * ensure thet L4LS clock domain does not get stuck in transition
	 * If that happens L3 module does not get disabled, thereby leading
	 * to PER power domain transition failing
	 */
	clkdm_wakeup(gfx_l4ls_clkdm);
	clkdm_sleep(gfx_l4ls_clkdm);

	if (ret) {
		pr_err("Kernel suspend failure\n");
	} else {
		i = am33xx_pm_status();
		switch (i) {
		case 0:
			pr_info("Successfully put all powerdomains to target state\n");
			/*
			 * XXX: Leads to loss of logic state in PER power domain
			 * Use SOC specific ops for this?
			 */
			break;
		case 1:
			pr_err("Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("Something went wrong :(\nStatus = %d\n", i);
			ret = -1;
		}

		/* print the wakeup reason */
		i = am33xx_pm_wake_src();
		for (j = 0; j < ARRAY_SIZE(wakeups); j++) {
			if (wakeups[j].irq_nr == i) {
				pr_info("Wakeup src %s\n", wakeups[j].src);
				break;
			}
		}

		if (j > ARRAY_SIZE(wakeups))
			pr_info("Unknown wakeup source %d!!!\n", i);
	}

	return ret;
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = am33xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* returns the error code from msg_send - 0 for success, failure otherwise */
static int am33xx_ping_wkup_m3(void)
{
	int ret = 0;

	omap_mbox_enable_irq(am33xx_pm->mbox, IRQ_RX);

	ret = omap_mbox_msg_send(am33xx_pm->mbox, 0xABCDABCD);

	omap_mbox_disable_irq(am33xx_pm->mbox, IRQ_RX);

	return ret;
}

static void am33xx_m3_state_machine_reset(void)
{
	int i;

	am33xx_pm->ipc.sleep_mode = IPC_CMD_RESET;

	am33xx_pm_ipc_cmd(&am33xx_pm->ipc);

	am33xx_pm->state = M3_STATE_MSG_FOR_RESET;

	pr_info("Sending message for resetting M3 state machine\n");

	if (!am33xx_ping_wkup_m3()) {
		i = wait_for_completion_timeout(&am33xx_pm_sync,
					msecs_to_jiffies(500));
		if (WARN(i == 0, "MPU<->CM3 sync failure"))
			am33xx_pm->state = M3_STATE_UNKNOWN;
	}
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int i;

	cpu_idle_poll_ctrl(true);

	am33xx_pm->ipc.sleep_mode	= IPC_CMD_DS0;
	am33xx_pm->ipc.param1		= DS_IPC_DEFAULT;
	am33xx_pm->ipc.param2		= DS_IPC_DEFAULT;

	am33xx_pm_ipc_cmd(&am33xx_pm->ipc);

	am33xx_pm->state = M3_STATE_MSG_FOR_LP;

	pr_info("Sending message for entering DeepSleep mode\n");

	if (!am33xx_ping_wkup_m3()) {
		i = wait_for_completion_timeout(&am33xx_pm_sync,
					msecs_to_jiffies(500));
		if (WARN(i == 0, "MPU<->CM3 sync failure"))
			return -1;
	}

	return 0;
}

static void am33xx_pm_end(void)
{
	am33xx_m3_state_machine_reset();

	cpu_idle_poll_ctrl(false);

	return;
}

static struct platform_suspend_ops am33xx_pm_ops = {
	.begin		= am33xx_pm_begin,
	.end		= am33xx_pm_end,
	.enter		= am33xx_pm_enter,
};
#endif /* CONFIG_SUSPEND */

/*
 * Dummy notifier for the mailbox
 * TODO: Get rid of this requirement once the MBX driver has been finalized
 */
static int wkup_mbox_msg(struct notifier_block *self, unsigned long len,
		void *msg)
{
	return 0;
}

static struct notifier_block wkup_mbox_notifier = {
	.notifier_call = wkup_mbox_msg,
};

/* TODO: register this as a callback from M3 IRQ */
int am33xx_txev_handler()
{
	int ret = 0;

	switch (am33xx_pm->state) {
	case M3_STATE_RESET:
		am33xx_pm->state = M3_STATE_INITED;
		am33xx_pm->ver = am33xx_pm_version_get();
		if (am33xx_pm->ver == M3_VERSION_UNKNOWN ||
			am33xx_pm->ver < M3_BASELINE_VERSION) {
			pr_warn("Invalid CM3 Firmware\n");
		} else {
			pr_info("CM3 Firmware Version = 0x%x\n",
						am33xx_pm->ver);
			am33xx_pm_ops.valid = suspend_valid_only_mem;
		}
		break;
	case M3_STATE_MSG_FOR_RESET:
		am33xx_pm->state = M3_STATE_INITED;
		omap_mbox_msg_rx_flush(am33xx_pm->mbox);
		complete(&am33xx_pm_sync);
		break;
	case M3_STATE_MSG_FOR_LP:
		omap_mbox_msg_rx_flush(am33xx_pm->mbox);
		complete(&am33xx_pm_sync);
		break;
	case M3_STATE_UNKNOWN:
		omap_mbox_msg_rx_flush(am33xx_pm->mbox);
		ret = -1;
	}

	return ret;
}

static void am33xx_pm_firmware_cb(const struct firmware *fw, void *context)
{
	struct am33xx_pm_context *am33xx_pm = context;
	int ret = 0;

	/* no firmware found */
	if (!fw) {
		pr_err("PM request_firmware failed\n");
		return;
	}

	wkup_m3_copy_code(fw->data, fw->size);

	pr_info("Copied the M3 firmware to UMEM\n");

	/*
	 * Invalidate M3 firmware version before hardreset.
	 * Write invalid version in lower 4 nibbles of parameter
	 * register (ipc_regs + 0x8).
	 */
	am33xx_pm_version_clear();

	am33xx_pm->state = M3_STATE_RESET;

	ret = wkup_m3_prepare();
	if (ret) {
		pr_err("Could not prepare WKUP_M3\n");
		return;
	} else {
#ifdef CONFIG_SUSPEND
		suspend_set_ops(&am33xx_pm_ops);
		/* Physical resume address to be used by ROM code */
		am33xx_pm->ipc.resume_addr = (AM33XX_OCMC_END -
				am33xx_do_wfi_sz + am33xx_resume_offset + 0x4);
#endif
	}

	/* Reserve the MBOX for sending messages to M3 */
	am33xx_pm->mbox = omap_mbox_get("wkup_m3", &wkup_mbox_notifier);
	if (IS_ERR(am33xx_pm->mbox))
		pr_err("Could not reserve mailbox for A8->M3 IPC\n");

	return;
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
void am33xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = (void *)omap_sram_push
					(am33xx_do_wfi, am33xx_do_wfi_sz);
}

static int __init am33xx_map_emif(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF_BASE, SZ_32K);

	if (!am33xx_emif_base)
		return -ENOMEM;

	return 0;
}

int __init am33xx_pm_init(void)
{
	int ret;
	u32 temp;
	struct device_node *np;
	int i;

	if (!soc_is_am33xx())
		return -ENODEV;

	pr_info("Power Management for AM33XX family\n");

	/*
	 * By default the following IPs do not have MSTANDBY asserted
	 * which is necessary for PER domain transition. If the drivers
	 * are not compiled into the kernel HWMOD code will not change the
	 * state of the IPs if the IP was not never enabled
	 */
	for (i = 0; i < ARRAY_SIZE(am33xx_mod); i++)
		am33xx_mod[i].dev = omap_device_get_by_hwmod_name(am33xx_mod[i].oh_name);

	gfx_pwrdm = pwrdm_lookup("gfx_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");

	gfx_l4ls_clkdm = clkdm_lookup("gfx_l4ls_gfx_clkdm");

	if ((!gfx_pwrdm) || (!per_pwrdm) || (!gfx_l4ls_clkdm)) {
		ret = -ENODEV;
		goto err;
	}

	am33xx_pm = kzalloc(sizeof(struct am33xx_pm_context), GFP_KERNEL);
	if (!am33xx_pm) {
		pr_err("Memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = am33xx_map_emif();
	if (ret) {
		pr_err("Could not ioremap EMIF\n");
		goto err;
	} else {
		/* Determine Memory Type */
		temp = readl(am33xx_emif_base + EMIF_SDRAM_CONFIG);
		temp = (temp & SDRAM_TYPE_MASK) >> SDRAM_TYPE_SHIFT;
		/* Parameters to pass to aseembly code */
		susp_params.emif_addr_virt = am33xx_emif_base;
		susp_params.dram_sync = am33xx_dram_sync;
		susp_params.mem_type = temp;
		am33xx_pm->ipc.param3 = temp;
	}

	np = of_find_compatible_node(NULL, NULL, "ti,am3353-wkup-m3");
	if (np) {
		if (of_find_property(np, "ti,needs_vtt_toggle", NULL) &&
		    (!(of_property_read_u32(np, "vtt-gpio-pin",
							&temp)))) {
			if (temp >= 0 && temp <= 31)
				am33xx_pm->ipc.param3 |=
					((1 << VTT_STAT_SHIFT) |
					(temp << VTT_GPIO_PIN_SHIFT));
		}
	}

	(void) clkdm_for_each(omap_pm_clkdms_setup, NULL);

	/* CEFUSE domain can be turned off post bootup */
	cefuse_pwrdm = pwrdm_lookup("cefuse_pwrdm");
	if (cefuse_pwrdm)
		omap_set_pwrdm_state(cefuse_pwrdm, PWRDM_POWER_OFF);
	else
		pr_err("Failed to get cefuse_pwrdm\n");

	/* TODO: get the wkup_m3 context pointer */
#if 0
	temp = wkup_m3_is_inited();
#endif

	pr_info("Trying to load am335x-pm-firmware.bin");

	/* We don't want to delay boot */
	request_firmware_nowait(THIS_MODULE, 0, "am335x-pm-firmware.bin",
				NULL, GFP_KERNEL, am33xx_pm,
				am33xx_pm_firmware_cb);
err:
	return ret;
}
