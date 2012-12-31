/*
 * Low level suspend code for AM33XX SoCs
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

#include <linux/linkage.h>
#include <linux/ti_emif.h>
#include <asm/memory.h>
#include <asm/assembler.h>

#include "cm33xx.h"
#include "pm33xx.h"
#include "prm33xx.h"

	.text
	.align 3

/*
 * This routine is executed from internal RAM and expects some
 * parameters to be passed in r0 _strictly_ in following order:
 * 1) emif_addr_virt - ioremapped EMIF address
 * 2) mem_type - 2 -> DDR2, 3-> DDR3
 * 3) dram_sync_word - uncached word in SDRAM
 *
 * The code loads these values taking r0 value as reference to
 * the array in registers starting from r0, i.e emif_addr_virt
 * goes to r1, mem_type goes to r2 and and so on. These are
 * then saved into memory locations before proceeding with the
 * sleep sequence and hence registers r0, r1 etc can still be
 * used in the rest of the sleep code.
 */

ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	ldm	r0, {r1-r3}		@ gather values passed

	/* Save the values passed */
	str	r1, emif_addr_virt
	str	r2, mem_type
	str	r3, dram_sync_word

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	ldr	r1, kernel_flush
	mov	lr, pc
	bx	r1

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)	@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	ldr	r1, kernel_flush
	blx	r1
	/*
	 * The kernel doesn't interwork: v7_flush_dcache_all in particluar will
	 * always return in Thumb state when CONFIG_THUMB2_KERNEL is enabled.
	 * This sequence switches back to ARM.  Note that .align may insert a
	 * nop: bx pc needs to be word-aligned in order to work.
	 */
 THUMB(	.thumb		)
 THUMB(	.align		)
 THUMB(	bx	pc	)
 THUMB(	nop		)
	.arm

	ldr	r0, emif_addr_virt
	/* Save EMIF configuration */
	ldr	r1, [r0, #EMIF_SDRAM_CONFIG]
	str	r1, emif_sdcfg_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, emif_ref_ctrl_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, emif_timing1_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, emif_timing2_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, emif_timing3_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, emif_pmcr_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, emif_pmcr_shdw_val
	ldr	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]
	str	r1, emif_zqcfg_val
	ldr	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, emif_rd_lat_val

	/* Put SDRAM in self-refresh */
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	orr	r1, r1, #0xa0
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, [r0, #4]

	ldr	r1, dram_sync_word	@ a dummy access to DDR as per spec
	ldr	r2, [r1, #0]
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	orr	r1, r1, #0x200
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	mov	r1, #0x1000		@ Wait for system to enter SR
wait_sr:
	subs	r1, r1, #1
	bne	wait_sr

	/* Disable EMIF */
	ldr	r1, virt_emif_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #0x03
	str	r2, [r1]

	ldr	r1, virt_emif_clkctrl
wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	ldr	r1, virt_mpu_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #0x03
	str	r2, [r1]

	dsb
	dmb
	isb

	wfi

	/* NOPs to ensure the A8 pipeline is clean */
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	/* We come here in case of an abort due to a late interrupt */

	/* Set MPU_CLKCTRL.MODULEMODE back to ENABLE */
	ldr	r1, virt_mpu_clkctrl
	mov	r2, #0x02
	str	r2, [r1]

	/* Re-enable EMIF */
	ldr	r1, virt_emif_clkctrl
	mov	r2, #0x02
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	/* Disable EMIF self-refresh */
	ldr	r0, emif_addr_virt
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #LP_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
	ldr r0, emif_addr_virt
	add r0, r0, #EMIF_SDRAM_CONFIG
	ldr r4, emif_sdcfg_val
	str r4, [r0]

	/*
	 * Set SCTLR.C bit to allow data cache allocation
	 */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 2)	@ Enable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/* Kill some time for sanity to settle in */
	mov r0, #0x1000
wait_abt:
	subs   r0, r0, #1
	bne wait_abt

	/* Let the suspend code know about the abort */
	mov	r0, #1
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(am33xx_do_wfi)

	.align
ENTRY(am33xx_resume_offset)
	.word . - am33xx_do_wfi

ENTRY(am33xx_resume_from_deep_sleep)
	/* Re-enable EMIF */
	ldr	r0, phys_emif_clkctrl
	mov	r1, #0x02
	str	r1, [r0]
wait_emif_enable1:
	ldr	r2, [r0]
	cmp	r1, r2
	bne	wait_emif_enable1

	/* Config EMIF Timings */
	ldr	r0, emif_phys_addr
	ldr	r1, emif_rd_lat_val
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1_SHDW]
	ldr	r1, emif_timing1_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, [r0, #EMIF_SDRAM_TIMING_1_SHDW]
	ldr	r1, emif_timing2_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, [r0, #EMIF_SDRAM_TIMING_2_SHDW]
	ldr	r1, emif_timing3_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, [r0, #EMIF_SDRAM_TIMING_3_SHDW]
	ldr	r1, emif_ref_ctrl_val
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CTRL_SHDW]
	ldr	r1, emif_pmcr_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	ldr	r1, emif_pmcr_shdw_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	/*
	 * Output impedence calib needed only for DDR3
	 * but since the initial state of this will be
	 * disabled for DDR2 no harm in restoring the
	 * old configuration
	 */
	ldr	r1, emif_zqcfg_val
	str	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]

	/* Write to SDRAM_CONFIG only for DDR2 */
	ldr	r2, mem_type
	cmp	r2, #MEM_TYPE_DDR2
	bne	resume_to_ddr

 	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
	ldr	r1, emif_sdcfg_val
	str	r1, [r0, #EMIF_SDRAM_CONFIG]

resume_to_ddr:
	/* Back from la-la-land. Kill some time for sanity to settle in */
	mov	r0, #0x1000
wait_resume:
	subs	r0, r0, #1
	bne	wait_resume

	/* We are back. Branch to the common CPU resume routine */
	mov	r0, #0
	ldr	pc, resume_addr
ENDPROC(am33xx_resume_from_deep_sleep)


/*
 * Local variables
 */
	.align
resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000
kernel_flush:
	.word   v7_flush_dcache_all
ddr_start:
	.word	PAGE_OFFSET
emif_phys_addr:
	.word	AM33XX_EMIF_BASE
virt_mpu_clkctrl:
	.word	AM33XX_CM_MPU_MPU_CLKCTRL
virt_emif_clkctrl:
	.word	AM33XX_CM_PER_EMIF_CLKCTRL
phys_emif_clkctrl:
	.word	(AM33XX_CM_BASE + AM33XX_CM_PER_MOD + \
		AM33XX_CM_PER_EMIF_CLKCTRL_OFFSET)
module_disabled_val:
	.word	0x30000

/* DDR related defines */
dram_sync_word:
	.word	0xDEADBEEF
mem_type:
	.word	0xDEADBEEF
emif_addr_virt:
	.word	0xDEADBEEF
emif_rd_lat_val:
	.word	0xDEADBEEF
emif_timing1_val:
	.word	0xDEADBEEF
emif_timing2_val:
	.word	0xDEADBEEF
emif_timing3_val:
	.word	0xDEADBEEF
emif_sdcfg_val:
	.word	0xDEADBEEF
emif_ref_ctrl_val:
	.word	0xDEADBEEF
emif_zqcfg_val:
	.word	0xDEADBEEF
emif_pmcr_val:
	.word	0xDEADBEEF
emif_pmcr_shdw_val:
	.word	0xDEADBEEF

	.align 3
ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
