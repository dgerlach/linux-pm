/*
 * AMx3 Wkup M3 IPC driver
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/omap-mailbox.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/suspend.h>
#include <linux/wkup_m3_ipc.h>

#define AM33XX_CTRL_IPC_REG_COUNT	0x8
#define AM33XX_CTRL_IPC_REG_OFFSET(m)	(0x4 + 4 * (m))

/* AM33XX M3_TXEV_EOI register */
#define AM33XX_CONTROL_M3_TXEV_EOI	0x00

#define AM33XX_M3_TXEV_ACK		(0x1 << 0)
#define AM33XX_M3_TXEV_ENABLE		(0x0 << 0)

#define IPC_CMD_DS0			0x4
#define IPC_CMD_STANDBY			0xc
#define IPC_CMD_RESET			0xe
#define DS_IPC_DEFAULT			0xffffffff
#define M3_VERSION_UNKNOWN		0x0000ffff
#define M3_BASELINE_VERSION		0x187
#define M3_WAKE_SRC_MASK		0xFF
#define M3_STATUS_RESP_MASK		(0xffff << 16)
#define M3_FW_VERSION_MASK		0xffff

#define M3_STATE_UNKNOWN		0
#define M3_STATE_RESET			1
#define M3_STATE_INITED			2
#define M3_STATE_MSG_FOR_LP		3
#define M3_STATE_MSG_FOR_RESET		4

struct wkup_m3_wakeup_src {
	int irq_nr;
	char src[10];
};

struct wkup_m3_ipc {
	struct rproc *rproc;

	void __iomem *ipc_mem_base;
	struct device *dev;

	int mem_type;
	unsigned long resume_addr;
	int state;

	struct mbox_client mbox_client;
	struct mbox_chan *mbox;
};

static struct wkup_m3_ipc m3_ipc_state;

static DECLARE_COMPLETION(m3_ipc_sync);

static const struct wkup_m3_wakeup_src wakeups[] = {
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
	{.irq_nr = 0,	.src = "Unknown"},
};

static inline void am33xx_txev_eoi(struct wkup_m3_ipc *m3_ipc)
{
	writel(AM33XX_M3_TXEV_ACK,
	       m3_ipc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static inline void am33xx_txev_enable(struct wkup_m3_ipc *m3_ipc)
{
	writel(AM33XX_M3_TXEV_ENABLE,
	       m3_ipc->ipc_mem_base + AM33XX_CONTROL_M3_TXEV_EOI);
}

static inline void wkup_m3_ctrl_ipc_write(struct wkup_m3_ipc *m3_ipc,
					  u32 val, int ipc_reg_num)
{
	if (ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT)
		return;

	writel(val, m3_ipc->ipc_mem_base +
	       AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static inline unsigned int wkup_m3_ctrl_ipc_read(struct wkup_m3_ipc *m3_ipc,
						 int ipc_reg_num)
{
	if (ipc_reg_num < 0 || ipc_reg_num > AM33XX_CTRL_IPC_REG_COUNT)
		return 0;

	return readl(m3_ipc->ipc_mem_base +
		     AM33XX_CTRL_IPC_REG_OFFSET(ipc_reg_num));
}

static int wkup_m3_fw_version_read(void)
{
	int val;

	val = wkup_m3_ctrl_ipc_read(&m3_ipc_state, 2);

	return val & M3_FW_VERSION_MASK;
}

static irqreturn_t wkup_m3_txev_handler(int irq, void *ipc_data)
{
	struct wkup_m3_ipc *m3_ipc = (struct wkup_m3_ipc *)ipc_data;
	struct device *dev = m3_ipc->dev;
	int ver = 0;

	am33xx_txev_eoi(m3_ipc);

	switch (m3_ipc->state) {
	case M3_STATE_RESET:
		ver = wkup_m3_fw_version_read();

		if (ver == M3_VERSION_UNKNOWN ||
		    ver < M3_BASELINE_VERSION) {
			dev_warn(dev, "CM3 Firmware Version %x not supported\n",
				 ver);
		} else {
			dev_info(dev, "CM3 Firmware Version = 0x%x\n", ver);
		}

		m3_ipc->state = M3_STATE_INITED;
		complete(&m3_ipc_sync);
		break;
	case M3_STATE_MSG_FOR_RESET:
		m3_ipc->state = M3_STATE_INITED;
		complete(&m3_ipc_sync);
		break;
	case M3_STATE_MSG_FOR_LP:
		complete(&m3_ipc_sync);
		break;
	case M3_STATE_UNKNOWN:
		dev_warn(dev, "Unknown CM3 State\n");
	}

	am33xx_txev_enable(m3_ipc);

	return IRQ_HANDLED;
}

static void wkup_m3_mbox_callback(struct mbox_client *client, void *data)
{
	struct wkup_m3_ipc *m3_ipc = container_of(client, struct wkup_m3_ipc,
						  mbox_client);

	omap_mbox_disable_irq(m3_ipc->mbox, IRQ_RX);
}

static int wkup_m3_ping(void)
{
	struct device *dev = m3_ipc_state.dev;
	mbox_msg_t dummy_msg = 0;
	int ret;

	if (!m3_ipc_state.mbox) {
		dev_err(m3_ipc_state.dev,
			"No IPC channel to communicate with wkup_m3!\n");
		return -EIO;
	}

	/*
	 * Write a dummy message to the mailbox in order to trigger the RX
	 * interrupt to alert the M3 that data is available in the IPC
	 * registers. We must enable the IRQ here and disable it after in
	 * the RX callback to avoid multiple interrupts being received
	 * by the CM3.
	 */
	omap_mbox_enable_irq(m3_ipc_state.mbox, IRQ_RX);
	ret = mbox_send_message(m3_ipc_state.mbox, (void *)dummy_msg);
	if (ret < 0) {
		dev_err(dev, "%s: mbox_send_message() failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&m3_ipc_sync,
					  msecs_to_jiffies(500));
	if (!ret) {
		dev_err(dev, "MPU<->CM3 sync failure\n");
		m3_ipc_state.state = M3_STATE_UNKNOWN;
		return -EIO;
	}

	return 0;
}

static const char *wkup_m3_request_wake_src(void)
{
	unsigned int wakeup_src_idx;
	int j, val;

	val = wkup_m3_ctrl_ipc_read(&m3_ipc_state, 6);

	wakeup_src_idx = val & M3_WAKE_SRC_MASK;

	for (j = 0; j < ARRAY_SIZE(wakeups)-1; j++) {
		if (wakeups[j].irq_nr == wakeup_src_idx)
			return wakeups[j].src;
	}
	return wakeups[j].src;
}

static int wkup_m3_is_available(void)
{
	return (m3_ipc_state.state != M3_STATE_RESET);
}

/* Public functions */
/**
 * wkup_m3_set_mem_type - Pass wkup_m3 which type of memory is in use
 * @mem_type: memory type value read directly from emif
 *
 * wkup_m3 must know what memory type is in use to properly suspend
 * and resume.
 */
void wkup_m3_set_mem_type(int mem_type)
{
	m3_ipc_state.mem_type = mem_type;
}

/**
 * wkup_m3_set_resume_address - Pass wkup_m3 resume address
 * @addr: Physical address from which resume code should execute
 */
void wkup_m3_set_resume_address(void *addr)
{
	m3_ipc_state.resume_addr = (unsigned long)addr;
}

/**
 * wkup_m3_set_resume_address - Retrieve wkup_m3 status code after suspend
 *
 * Returns code representing the status of a low power mode transition.
 *	0 - Successful transition
 *	1 - Failure to transition to low power state
 */
int wkup_m3_request_pm_status(void)
{
	struct device *dev = m3_ipc_state.dev;
	unsigned int i;
	int val;
	const char *wake_src;

	/* print the wakeup reason */
	wake_src = wkup_m3_request_wake_src();
	dev_dbg(dev, "Wakeup source %s\n", wake_src);

	val = wkup_m3_ctrl_ipc_read(&m3_ipc_state, 1);

	i = M3_STATUS_RESP_MASK & val;
	i >>= __ffs(M3_STATUS_RESP_MASK);

	return i;
}

/**
 * wkup_m3_prepare_low_power - Request preparation for transition to
 *			       low power state
 * @state: A kernel suspend state to enter, either MEM or STANDBY
 *
 * Returns 0 if preparation was successful, otherwise returns error code
 */
int wkup_m3_prepare_low_power(int state)
{
	struct device *dev = m3_ipc_state.dev;
	int m3_power_state;
	int ret = 0;

	if (!wkup_m3_is_available()) {
		dev_err(dev, "wkup_m3 not available. DeepSleep entry not possible.\n");
		return -ENODEV;
	}

	switch (state) {
	case PM_SUSPEND_MEM:
		m3_power_state = IPC_CMD_DS0;
		break;
	default:
		return 1;
	}

	/* Program each required IPC register then write defaults to others */
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, m3_ipc_state.resume_addr, 0);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, m3_power_state, 1);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, m3_ipc_state.mem_type, 4);

	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 2);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 3);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 5);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 6);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 7);

	m3_ipc_state.state = M3_STATE_MSG_FOR_LP;

	ret = wkup_m3_ping();
	if (ret) {
		dev_err(dev, "Unable to ping CM3\n");
		return ret;
	}

	return 0;
}

/**
 * wkup_m3_finish_low_power - Return m3 to reset state
 *
 * Returns 0 if reset was successful, otherwise returns error code
 */
int wkup_m3_finish_low_power(void)
{
	struct device *dev = m3_ipc_state.dev;
	int ret = 0;

	if (!wkup_m3_is_available())
		return -ENODEV;

	wkup_m3_ctrl_ipc_write(&m3_ipc_state, IPC_CMD_RESET, 1);
	wkup_m3_ctrl_ipc_write(&m3_ipc_state, DS_IPC_DEFAULT, 2);

	m3_ipc_state.state = M3_STATE_MSG_FOR_RESET;

	ret = wkup_m3_ping();
	if (ret) {
		dev_err(dev, "Unable to ping CM3\n");
		return ret;
	}

	return 0;
}

static int match(struct device *dev, void *data)
{
	struct device_node *node = (struct device_node *)data;

	return dev->of_node == node;
}

static struct rproc *wkup_m3_get_rproc(struct device *dev)
{
	struct device_node *node;
	struct rproc *rp;

	node = of_parse_phandle(dev->of_node, "ti,rproc", 0);
	if (!node)
		return NULL;

	dev = bus_find_device(&platform_bus_type, NULL, node, match);
	if (!dev)
		return NULL;

	rp = dev_get_drvdata(dev);
	return rp;
}

static void wkup_m3_rproc_boot_thread(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	int ret;

	wait_for_completion(&rproc->firmware_loading_complete);

	ret = rproc_boot(rproc);
	if (ret)
		dev_err(dev, "rproc_boot failed\n");

	do_exit(0);
}

static int wkup_m3_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret;
	struct rproc *m3_rproc;
	struct resource *res;
	struct task_struct *task;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ipc_regs");
	m3_ipc_state.ipc_mem_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(m3_ipc_state.ipc_mem_base)) {
		dev_err(dev, "could not ioremap ipc_mem\n");
		ret = PTR_ERR(m3_ipc_state.ipc_mem_base);
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource\n");
		ret = -ENXIO;
		goto err;
	}

	ret = devm_request_irq(dev, irq, wkup_m3_txev_handler,
			       IRQF_DISABLED, "wkup_m3_txev", &m3_ipc_state);
	if (ret) {
		dev_err(dev, "request_irq failed\n");
		goto err;
	}

	m3_ipc_state.mbox_client.dev = dev;
	m3_ipc_state.mbox_client.tx_done = NULL;
	m3_ipc_state.mbox_client.rx_callback = wkup_m3_mbox_callback;
	m3_ipc_state.mbox_client.tx_block = false;
	m3_ipc_state.mbox_client.knows_txdone = false;

	m3_ipc_state.mbox = mbox_request_channel(&m3_ipc_state.mbox_client, 0);
	if (IS_ERR(m3_ipc_state.mbox)) {
		dev_err(dev, "IPC Request for A8->M3 Channel failed! %ld\n",
			PTR_ERR(m3_ipc_state.mbox));
		ret = PTR_ERR(m3_ipc_state.mbox);
		m3_ipc_state.mbox = NULL;
		return ret;
	}

	m3_rproc = wkup_m3_get_rproc(dev);
	if (!m3_rproc) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	m3_ipc_state.dev = dev;
	m3_ipc_state.state = M3_STATE_RESET;
	/*
	 * Wait for firmware loading completion in a thread so we
	 * can boot the wkup_m3 as soon as it's ready without holding
	 * up kernel boot
	 */
	task = kthread_run((void *)wkup_m3_rproc_boot_thread, m3_rproc,
			   "wkup_m3_rproc_loader");

	if (IS_ERR(task)) {
		dev_err(dev, "can't create rproc_boot thread\n");
		goto err;
	}

	return 0;

err:
	mbox_free_channel(m3_ipc_state.mbox);
	return ret;
}

static int wkup_m3_ipc_remove(struct platform_device *pdev)
{
	if (m3_ipc_state.mbox)
		mbox_free_channel(m3_ipc_state.mbox);

	return 0;
}

static const struct of_device_id wkup_m3_ipc_of_match[] = {
	{ .compatible = "ti,am3353-wkup-m3-ipc", .data = NULL, },
	{},
};

static struct platform_driver wkup_m3_ipc_driver = {
	.probe = wkup_m3_ipc_probe,
	.remove = wkup_m3_ipc_remove,
	.driver = {
		.name = "wkup_m3_ipc",
		.of_match_table = wkup_m3_ipc_of_match,
	},
};

module_platform_driver(wkup_m3_ipc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("wkup m3 remote processor ipc driver");
