/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Inc.
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
#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#include "control.h"

#ifndef __ASSEMBLER__

struct am33xx_pm_context {
	struct am33xx_ipc_data	ipc;
	struct firmware		*firmware;
	void	*mbox;
	u8			state;
	u32			ver;
};

/*
 * Params passed to suspend routine
 *
 * Since these are used to load into registers by suspend code,
 * entries here must always be in sync with the suspend code
 * in arm/mach-omap2/sleep33xx.S
 */
struct am33xx_suspend_params {
	void __iomem *emif_addr_virt;
	u32 wfi_flags;
	void __iomem *dram_sync;
};

struct wakeup_src {
	int irq_nr;
	char src[10];
};

struct forced_standby_module {
	char oh_name[15];
	struct device *dev;
};

int wkup_m3_copy_code(const u8 *data, size_t size);
int wkup_m3_prepare(void);

#endif

#define	IPC_CMD_DS0			0x3
#define IPC_CMD_RESET                   0xe
#define DS_IPC_DEFAULT			0xffffffff
#define M3_VERSION_UNKNOWN		0x0000ffff
#define M3_BASELINE_VERSION		0x21

#define M3_STATE_UNKNOWN		0
#define M3_STATE_RESET			1
#define M3_STATE_INITED			2
#define M3_STATE_MSG_FOR_LP		3
#define M3_STATE_MSG_FOR_RESET		4

#define AM33XX_OCMC_END			0x40310000
#define AM33XX_EMIF_BASE		0x4C000000

#define MEM_TYPE_DDR2		2
#define MEM_TYPE_DDR3		3

#define WFI_MEM_TYPE_DDR2	(1 << 0)
#define WFI_MEM_TYPE_DDR3	(1 << 1)
#define WFI_SELF_REFRESH	(1 << 2)
#define WFI_SAVE_EMIF		(1 << 3)
#define WFI_WAKE_M3		(1 << 4)

#endif
