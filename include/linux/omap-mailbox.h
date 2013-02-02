/*
 * omap-mailbox: interprocessor communication module for OMAP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef OMAP_MAILBOX_H
#define OMAP_MAILBOX_H

typedef u32 mbox_msg_t;

typedef int __bitwise omap_mbox_irq_t;
#define IRQ_TX ((__force omap_mbox_irq_t) 1)
#define IRQ_RX ((__force omap_mbox_irq_t) 2)

void omap_mbox_save_ctx(void *channel);
void omap_mbox_restore_ctx(void *channel);
void omap_mbox_enable_irq(void *channel, omap_mbox_irq_t irq);
void omap_mbox_disable_irq(void *channel, omap_mbox_irq_t irq);

#endif /* OMAP_MAILBOX_H */
