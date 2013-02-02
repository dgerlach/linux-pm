/*
 * OMAP mailbox driver
 *
 * Copyright (C) 2006-2009 Nokia Corporation. All rights reserved.
 *
 * Contact: Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/mailbox_controller.h>

#include "mailbox_internal.h"
#include "omap-mbox.h"

static unsigned int mbox_kfifo_size;

static struct omap_mbox *to_omap_mbox(void *channel)
{
	struct ipc_chan *chan = (struct ipc_chan *)channel;

	if (!chan || !chan->link)
		return NULL;

	return container_of(chan->link, struct omap_mbox, link);
}

/* Mailbox FIFO handle functions */
static inline mbox_msg_t mbox_fifo_read(struct omap_mbox *mbox)
{
	return mbox->ops->fifo_read(mbox);
}
static inline int mbox_fifo_empty(struct omap_mbox *mbox)
{
	return mbox->ops->fifo_empty(mbox);
}

/* Mailbox IRQ handle functions */
static inline void mbox_enable_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	mbox->ops->enable_irq(mbox, irq);
}
static inline void mbox_disable_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	mbox->ops->disable_irq(mbox, irq);
}
static inline void ack_mbox_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	if (mbox->ops->ack_irq)
		mbox->ops->ack_irq(mbox, irq);
}
static inline int is_mbox_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	return mbox->ops->is_irq(mbox, irq);
}

void omap_mbox_save_ctx(void *channel)
{
	struct omap_mbox *mbox = to_omap_mbox(channel);

	if (WARN_ON(!mbox))
		return;

	if (!mbox->ops->save_ctx) {
		dev_err(mbox->dev, "%s:\tno save\n", __func__);
		return;
	}

	mbox->ops->save_ctx(mbox);
}
EXPORT_SYMBOL(omap_mbox_save_ctx);

void omap_mbox_restore_ctx(void *channel)
{
	struct omap_mbox *mbox = to_omap_mbox(channel);

	if (WARN_ON(!mbox))
		return;

	if (!mbox->ops->restore_ctx) {
		dev_err(mbox->dev, "%s:\tno restore\n", __func__);
		return;
	}

	mbox->ops->restore_ctx(mbox);
}
EXPORT_SYMBOL(omap_mbox_restore_ctx);

void omap_mbox_enable_irq(void *channel, omap_mbox_irq_t irq)
{
	struct omap_mbox *mbox = to_omap_mbox(channel);

	if (WARN_ON(!mbox))
		return;

	mbox_enable_irq(mbox, irq);
}
EXPORT_SYMBOL(omap_mbox_enable_irq);

void omap_mbox_disable_irq(void *channel, omap_mbox_irq_t irq)
{
	struct omap_mbox *mbox = to_omap_mbox(channel);

	if (WARN_ON(!mbox))
		return;

	mbox_disable_irq(mbox, irq);
}
EXPORT_SYMBOL(omap_mbox_disable_irq);

/*
 * Message receiver(workqueue)
 */
static void mbox_rx_work(struct work_struct *work)
{
	struct omap_mbox_queue *mq =
			container_of(work, struct omap_mbox_queue, work);
	mbox_msg_t msg;
	int len;

	while (kfifo_len(&mq->fifo) >= sizeof(msg)) {
		len = kfifo_out(&mq->fifo, (unsigned char *)&msg, sizeof(msg));
		WARN_ON(len != sizeof(msg));

		ipc_link_received_data(&mq->mbox->link, (void *)msg);
		spin_lock_irq(&mq->lock);
		if (mq->full) {
			mq->full = false;
			mbox_enable_irq(mq->mbox, IRQ_RX);
		}
		spin_unlock_irq(&mq->lock);
	}
}

/*
 * Mailbox interrupt handler
 */
static void __mbox_tx_interrupt(struct omap_mbox *mbox)
{
	mbox_disable_irq(mbox, IRQ_TX);
	ack_mbox_irq(mbox, IRQ_TX);
	ipc_link_txdone(&mbox->link, XFER_OK);
}

static void __mbox_rx_interrupt(struct omap_mbox *mbox)
{
	struct omap_mbox_queue *mq = mbox->rxq;
	mbox_msg_t msg;
	int len;

	while (!mbox_fifo_empty(mbox)) {
		if (unlikely(kfifo_avail(&mq->fifo) < sizeof(msg))) {
			mbox_disable_irq(mbox, IRQ_RX);
			mq->full = true;
			goto nomem;
		}

		msg = mbox_fifo_read(mbox);

		len = kfifo_in(&mq->fifo, (unsigned char *)&msg, sizeof(msg));
		WARN_ON(len != sizeof(msg));
	}

	/* no more messages in the fifo. clear IRQ source. */
	ack_mbox_irq(mbox, IRQ_RX);
nomem:
	schedule_work(&mbox->rxq->work);
}

static irqreturn_t mbox_interrupt(int irq, void *p)
{
	struct omap_mbox *mbox = p;

	if (is_mbox_irq(mbox, IRQ_TX))
		__mbox_tx_interrupt(mbox);

	if (is_mbox_irq(mbox, IRQ_RX))
		__mbox_rx_interrupt(mbox);

	return IRQ_HANDLED;
}

static struct omap_mbox_queue *mbox_queue_alloc(struct omap_mbox *mbox,
					void (*work) (struct work_struct *))
{
	struct omap_mbox_queue *mq;

	if (!work)
		return NULL;

	mq = kzalloc(sizeof(struct omap_mbox_queue), GFP_KERNEL);
	if (!mq)
		return NULL;

	spin_lock_init(&mq->lock);

	if (kfifo_alloc(&mq->fifo, mbox_kfifo_size, GFP_KERNEL))
		goto error;

	INIT_WORK(&mq->work, work);
	return mq;

error:
	kfree(mq);
	return NULL;
}

static void mbox_queue_free(struct omap_mbox_queue *q)
{
	kfifo_free(&q->fifo);
	kfree(q);
}

int omap_mbox_startup(struct omap_mbox *mbox)
{
	int ret = 0;
	struct omap_mbox_queue *mq;

	if (mbox->use_count != 1)
		return -EINVAL;

	mq = mbox_queue_alloc(mbox, mbox_rx_work);
	if (!mq)
		return -ENOMEM;
	mbox->rxq = mq;
	mq->mbox = mbox;
	ret = request_irq(mbox->irq, mbox_interrupt, IRQF_SHARED,
						mbox->link.link_name, mbox);
	if (unlikely(ret)) {
		pr_err("failed to register mailbox interrupt:%d\n", ret);
		goto fail_request_irq;
	}

	mbox_enable_irq(mbox, IRQ_RX);
	return 0;

fail_request_irq:
	mbox_queue_free(mbox->rxq);
	return ret;
}

void omap_mbox_fini(struct omap_mbox *mbox)
{
	mbox_disable_irq(mbox, IRQ_RX);
	free_irq(mbox->irq, mbox);
	flush_work(&mbox->rxq->work);
	mbox_queue_free(mbox->rxq);
}

static struct class omap_mbox_class = { .name = "mbox", };

int omap_mbox_register(struct omap_mbox_device *mdev)
{
	int ret;
	int i;
	struct omap_mbox **mboxes;

	if (!mdev || !mdev->mboxes)
		return -EINVAL;

	mboxes = mdev->mboxes;
	for (i = 0; mboxes[i]; i++) {
		struct omap_mbox *mbox = mboxes[i];
		mbox->dev = device_create(&omap_mbox_class, mdev->dev,
					0, mbox, "%s", mbox->link.link_name);
		if (IS_ERR(mbox->dev)) {
			ret = PTR_ERR(mbox->dev);
			goto err_out;
		}
	}

	ret = ipc_links_register(&mdev->controller);

err_out:
	if (ret) {
		while (i--)
			device_unregister(mboxes[i]->dev);
	}
	return ret;
}

int omap_mbox_unregister(struct omap_mbox_device *mdev)
{
	int i;
	struct omap_mbox **mboxes;

	if (!mdev || !mdev->mboxes)
		return -EINVAL;

	ipc_links_unregister(&mdev->controller);

	mboxes = mdev->mboxes;
	for (i = 0; mboxes[i]; i++)
		device_unregister(mboxes[i]->dev);
	return 0;
}

int __init omap_mbox_init(int size)
{
	int err;

	err = class_register(&omap_mbox_class);
	if (err)
		return err;

	/* kfifo size sanity check: alignment and minimal size */
	mbox_kfifo_size = ALIGN(size, sizeof(mbox_msg_t));
	mbox_kfifo_size = max_t(unsigned int, size, sizeof(mbox_msg_t));

	return 0;
}

void __exit omap_mbox_exit(void)
{
	class_unregister(&omap_mbox_class);
}
