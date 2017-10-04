/*
 * TI AM33XX and AM43XX PM Assembly Offsets
 *
 * Copyright (C) 2017 Texas Instruments Inc.
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
#include <linux/kbuild.h>
#include <linux/platform_data/pm33xx.h>
#include <linux/ti-emif-sram.h>

int main(void)
{
	ti_emif_offsets();

	BLANK();

	DEFINE(AMX3_PM_WFI_FLAGS_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, wfi_flags));
	DEFINE(AMX3_PM_L2_AUX_CTRL_VAL_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, l2_aux_ctrl_val));
	DEFINE(AMX3_PM_L2_PREFETCH_CTRL_VAL_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, l2_prefetch_ctrl_val));
	DEFINE(AMX3_PM_SRAM_DATA_SIZE, sizeof(struct am33xx_pm_sram_data));

	BLANK();

	DEFINE(AMX3_PM_RO_SRAM_DATA_VIRT_OFFSET,
	       offsetof(struct am33xx_pm_ro_sram_data, amx3_pm_sram_data_virt));
	DEFINE(AMX3_PM_RO_SRAM_DATA_PHYS_OFFSET,
	       offsetof(struct am33xx_pm_ro_sram_data, amx3_pm_sram_data_phys));
	DEFINE(AMX3_PM_RO_SRAM_DATA_SIZE,
	       sizeof(struct am33xx_pm_ro_sram_data));

	return 0;
}
