/*
 * omap-mbox.h: OMAP mailbox internal definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef OMAP_MBOX_H
#define OMAP_MBOX_H

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/omap-mailbox.h>
#include <linux/mailbox_controller.h>

struct omap_mbox;

struct omap_mbox_ops {
	/* fifo */
	mbox_msg_t	(*fifo_read)(struct omap_mbox *mbox);
	int		(*fifo_empty)(struct omap_mbox *mbox);
	/* irq */
	void		(*enable_irq)(struct omap_mbox *mbox,
						omap_mbox_irq_t irq);
	void		(*disable_irq)(struct omap_mbox *mbox,
						omap_mbox_irq_t irq);
	void		(*ack_irq)(struct omap_mbox *mbox, omap_mbox_irq_t irq);
	int		(*is_irq)(struct omap_mbox *mbox, omap_mbox_irq_t irq);
	/* ctx */
	void		(*save_ctx)(struct omap_mbox *mbox);
	void		(*restore_ctx)(struct omap_mbox *mbox);
};

struct omap_mbox_queue {
	spinlock_t		lock;
	struct kfifo		fifo;
	struct work_struct	work;
	struct omap_mbox	*mbox;
	bool full;
};

struct omap_mbox_device {
	struct device *dev;
	struct mutex cfg_lock;
	void __iomem *mbox_base;
	u32 num_users;
	u32 num_fifos;
	struct omap_mbox **mboxes;
	struct ipc_controller controller;
};

struct omap_mbox {
	unsigned int		irq;
	struct omap_mbox_queue	*rxq;
	struct omap_mbox_ops	*ops;
	struct device		*dev;
	struct omap_mbox_device *parent;
	void			*priv;
	int			use_count;
	struct ipc_link		link;
};

static inline struct omap_mbox *link_to_omap_mbox(struct ipc_link *l)
{
	if (!l)
		return NULL;

	return container_of(l, struct omap_mbox, link);
}

int omap_mbox_startup(struct omap_mbox *mbox);
void omap_mbox_fini(struct omap_mbox *mbox);

int omap_mbox_register(struct omap_mbox_device *device);
int omap_mbox_unregister(struct omap_mbox_device *device);

int omap_mbox_init(int size);
void omap_mbox_exit(void);

#endif /* OMAP_MBOX_H */
