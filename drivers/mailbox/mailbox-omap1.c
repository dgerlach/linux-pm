/*
 * Mailbox reservation modules for OMAP1
 *
 * Copyright (C) 2006-2009 Nokia Corporation
 * Written by: Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "omap-mbox.h"

#define MAILBOX_ARM2DSP1		0x00
#define MAILBOX_ARM2DSP1b		0x04
#define MAILBOX_DSP2ARM1		0x08
#define MAILBOX_DSP2ARM1b		0x0c
#define MAILBOX_DSP2ARM2		0x10
#define MAILBOX_DSP2ARM2b		0x14
#define MAILBOX_ARM2DSP1_Flag		0x18
#define MAILBOX_DSP2ARM1_Flag		0x1c
#define MAILBOX_DSP2ARM2_Flag		0x20

static unsigned int mbox_kfifo_size = CONFIG_OMAP_MBOX_KFIFO_SIZE;
module_param(mbox_kfifo_size, uint, S_IRUGO);
MODULE_PARM_DESC(mbox_kfifo_size, "Size of omap's mailbox kfifo (bytes)");

static struct omap_mbox_device omap1_mbox_device;

struct omap_mbox1_fifo {
	unsigned long cmd;
	unsigned long data;
	unsigned long flag;
};

struct omap_mbox1_priv {
	struct omap_mbox1_fifo tx_fifo;
	struct omap_mbox1_fifo rx_fifo;
	bool empty_flag;
};

static inline int mbox_read_reg(size_t ofs)
{
	return __raw_readw(omap1_mbox_device.mbox_base + ofs);
}

static inline void mbox_write_reg(u32 val, size_t ofs)
{
	__raw_writew(val, omap1_mbox_device.mbox_base + ofs);
}

/* msg */
static mbox_msg_t omap1_mbox_fifo_read(struct omap_mbox *mbox)
{
	struct omap_mbox1_fifo *fifo =
		&((struct omap_mbox1_priv *)mbox->priv)->rx_fifo;
	mbox_msg_t msg;

	msg = mbox_read_reg(fifo->data);
	msg |= ((mbox_msg_t) mbox_read_reg(fifo->cmd)) << 16;

	(struct omap_mbox1_priv *)(mbox->priv)->empty_flag = false;
	return msg;
}

static void
omap1_mbox_fifo_write(struct omap_mbox *mbox, mbox_msg_t msg)
{
	struct omap_mbox1_fifo *fifo =
		&((struct omap_mbox1_priv *)mbox->priv)->tx_fifo;

	mbox_write_reg(msg & 0xffff, fifo->data);
	mbox_write_reg(msg >> 16, fifo->cmd);
}

static int omap1_mbox_fifo_empty(struct omap_mbox *mbox)
{
	struct omap_mbox1_priv *priv = (struct omap_mbox1_priv *)mbox->priv;

	return priv->empty_flag ? 0 : 1;
}

static int omap1_mbox_fifo_full(struct omap_mbox *mbox)
{
	struct omap_mbox1_fifo *fifo =
		&((struct omap_mbox1_priv *)mbox->priv)->rx_fifo;

	return mbox_read_reg(fifo->flag);
}

/* irq */
static void
omap1_mbox_enable_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	if (irq == IRQ_RX)
		enable_irq(mbox->irq);
}

static void
omap1_mbox_disable_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	if (irq == IRQ_RX)
		disable_irq(mbox->irq);
}

static int
omap1_mbox_is_irq(struct omap_mbox *mbox, omap_mbox_irq_t irq)
{
	struct omap_mbox1_priv *priv = (struct omap_mbox1_priv *)mbox->priv;

	if (irq == IRQ_TX)
		return 0;
	if (irq == IRQ_RX)
		priv->empty_flag = true;

	return 1;
}

static int omap1_mbox_startup(struct ipc_link *link, void *ignored)
{
	struct omap_mbox *mbox = link_to_omap_mbox(link);
	struct omap_mbox_device *mdev = mbox->parent;
	int ret = 0;

	mutex_lock(&mdev->cfg_lock);
	if (!mbox->use_count++) {
		ret = omap_mbox_startup(mbox);
		if (ret)
			mbox->use_count--;
	}
	mutex_unlock(&mdev->cfg_lock);
	return ret;
}

static void omap1_mbox_shutdown(struct ipc_link *link)
{
	struct omap_mbox *mbox = link_to_omap_mbox(link);
	struct omap_mbox_device *mdev = mbox->parent;

	mutex_lock(&mdev->cfg_lock);
	if (!--mbox->use_count)
		omap_mbox_fini(mbox);
	mutex_unlock(&mdev->cfg_lock);
}

static int omap1_mbox_send_data(struct ipc_link *link, void *data)
{
	struct omap_mbox *mbox = link_to_omap_mbox(link);

	if (!mbox)
		return -EINVAL;

	/* sanity check, this should never happen with the current framework */
	if (omap1_mbox_fifo_full(mbox))
		return -EBUSY;

	omap1_mbox_fifo_write(mbox, (mbox_msg_t) data);

	return 0;
}

static bool omap1_mbox_is_ready(struct ipc_link *link)
{
	struct omap_mbox *mbox = link_to_omap_mbox(link);

	if (WARN_ON(!mbox))
		return false;

	return omap1_mbox_fifo_full(mbox) ? false : true;
}

static struct ipc_link_ops omap1_link_ops = {
	.startup	= omap1_mbox_startup,
	.send_data	= omap1_mbox_send_data,
	.shutdown	= omap1_mbox_shutdown,
	.is_ready	= omap1_mbox_is_ready,
};

static struct omap_mbox_ops omap1_mbox_ops = {
	.fifo_read	= omap1_mbox_fifo_read,
	.fifo_empty	= omap1_mbox_fifo_empty,
	.enable_irq	= omap1_mbox_enable_irq,
	.disable_irq	= omap1_mbox_disable_irq,
	.is_irq		= omap1_mbox_is_irq,
};

/* FIXME: the following struct should be created automatically by the user id */

/* DSP */
static struct omap_mbox1_priv omap1_mbox_dsp_priv = {
	.tx_fifo = {
		.cmd	= MAILBOX_ARM2DSP1b,
		.data	= MAILBOX_ARM2DSP1,
		.flag	= MAILBOX_ARM2DSP1_Flag,
	},
	.rx_fifo = {
		.cmd	= MAILBOX_DSP2ARM1b,
		.data	= MAILBOX_DSP2ARM1,
		.flag	= MAILBOX_DSP2ARM1_Flag,
	},
};

static struct omap_mbox mbox_dsp_info = {
	.name	= "dsp",
	.ops	= &omap1_mbox_ops,
	.priv	= &omap1_mbox_dsp_priv,
	.parent	= &omap1_mbox_device,
};

static struct omap_mbox *omap1_mboxes[] = { &mbox_dsp_info, NULL };

static int omap1_mbox_probe(struct platform_device *pdev)
{
	struct resource *mem;
	int ret = 0;
	struct omap_mbox **list;
	struct omap_mbox_device *mdev = &omap1_mbox_device;
	struct ipc_links *links[2] = {NULL, NULL};

	list = omap1_mboxes;
	list[0]->irq = platform_get_irq_byname(pdev, "dsp");
	snprintf(list[0]->link.link_name, 16, "dsp");
	links[0] = &list[0]->link;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENOENT;

	mdev->mbox_base = ioremap(mem->start, resource_size(mem));
	if (!mdev->mbox_base)
		return -ENOMEM;
	mutex_init(&mdev->cfg_lock);
	mdev->dev = &pdev->dev;
	mdev->list = omap1_mboxes;
	mdev->num_users = 2;
	mdev->num_fifos = 4;

	/* OMAP1 does not have a Tx-Done or Tx-Ready IRQ */
	mdev->controller.txdone_irq = false;
	mdev->controller.txdone_poll = true;
	mdev->controller.txpoll_period = 1;
	mdev->controller.ops = &omap1_link_ops;
	mdev->controller.links = links;
	snprintf(mdev->controller.controller_name, 16, "omap1");
	ret = omap_mbox_register(mdev);
	if (ret)
		goto unmap_mbox;

	return 0;

unmap_mbox:
	iounmap(mdev->mbox_base);
	return ret;
}

static int omap1_mbox_remove(struct platform_device *pdev)
{
	struct omap_mbox_device *mdev = &omap1_mbox_device;

	omap_mbox_unregister(mdev);
	iounmap(mdev->mbox_base);
	mdev->mbox_base = NULL;
	mdev->list = NULL;
	mdev->dev = NULL;
	mdev->controller.links = NULL;

	return 0;
}

static struct platform_driver omap1_mbox_driver = {
	.probe	= omap1_mbox_probe,
	.remove	= omap1_mbox_remove,
	.driver	= {
		.name	= "omap-mailbox",
	},
};

static int __init omap1_mbox_init(void)
{
	omap_mbox_init(mbox_kfifo_size);

	return platform_driver_register(&omap1_mbox_driver);
}

static void __exit omap1_mbox_exit(void)
{
	platform_driver_unregister(&omap1_mbox_driver);

	omap_mbox_exit();
}

module_init(omap1_mbox_init);
module_exit(omap1_mbox_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("omap mailbox: omap1 architecture specific functions");
MODULE_AUTHOR("Hiroshi DOYU <Hiroshi.DOYU@nokia.com>");
MODULE_ALIAS("platform:omap1-mailbox");
