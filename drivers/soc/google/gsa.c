// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google GSA mailbox (gs201)
 *
 * The GSA is the security core of Tensor SoCs. The AP reaches it through a
 * Samsung-style mailbox: the command goes into shared register 0, the
 * argument count into 1 and the arguments after them; the AP raises
 * interrupt 0 on the GSA side, and the GSA answers in the same registers
 * (command | BIT(31), error, argument count, arguments) and raises interrupt
 * 0 on the AP side.
 *
 * The GSA authenticates and loads the AoC (audio and sensor DSP) firmware
 * (gsa_load_aoc_fw_image(), mailbox command 90). AoC state commands
 * (gsa_send_aoc_cmd()) go to the Trusty service
 * com.android.trusty.gsa.hwmgr.aoc, as in Google's kernels, when the
 * driver is built with CONFIG_GOOGLE_GSA_TRUSTY.
 *
 * While the GSA reads an image header by DMA, its stage-2 MPU (S2MPU) must
 * let it through. On stock Android the hypervisor programs that S2MPU each
 * time the GSA core powers up; without a hypervisor nothing does, so with
 * the optional "s2mpu" register range this driver sets its CTRL0 to 0 (no
 * other S2MPU register is touched) for the duration of an image load, under
 * a GSA wakelock, which keeps the GSA core and its S2MPU powered.
 *
 * debugfs, for bring-up:
 *
 *   sjtag_status  GSA_MB_CMD_SJTAG_GET_STATUS, which stock Android sends at
 *                 every boot; a ping that proves the mailbox works.
 *   aoc_state     GSA_MB_CMD_AOC_CMD with GSA_AOC_GET_STATE, only with the
 *                 aoc_query parameter: does the GSA firmware still serve AoC
 *                 state commands on the mailbox?
 *   aoc_state_tz  GSA_AOC_GET_STATE through Trusty (CONFIG_GOOGLE_GSA_TRUSTY)
 *   s2mpu_ctrl0   the S2MPU's CTRL0, read under a GSA wakelock, only with the
 *                 s2mpu_probe parameter
 *
 * Command numbers, error codes, the register protocol and the Trusty
 * message format follow Google's driver,
 * google-modules/soc/gs/drivers/soc/google/gsa/ (gsa_mbox.[ch], gsa_core.c,
 * gsa_tz.c, hwmgr-ipc.h). Unlike Google's driver, a mailbox command that
 * gets no answer times out, and the mailbox is then refused until reload.
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/soc/google/gsa.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
#include <linux/trusty/trusty_ipc.h>
#endif

#define MBOX_INTCR0			0x0024
#define MBOX_INTMR0			0x0028
#define MBOX_INTMSR0			0x0030
#define MBOX_INTGR1			0x0040
#define MBOX_SR(n)			(0x0080 + (n) * 4)
#define MBOX_SR_NUM			16

#define MBOX_HOST_REQ_IRQ		BIT(0)	/* AP -> GSA, INTGR1 */
#define MBOX_CLIENT_RSP_IRQ		BIT(0)	/* GSA -> AP, IRQ0 */

#define GSA_MB_CMD_RSP			BIT(31)

/* Commands (gsa_mbox.h, enum gsa_mbox_cmd) */
#define GSA_MB_CMD_LOAD_TPU_FW_IMG	50
#define GSA_MB_CMD_TPU_CMD		51
#define GSA_MB_CMD_UNLOAD_TPU_FW_IMG	52
#define GSA_MB_CMD_LOAD_AOC_FW_IMG	90
#define GSA_MB_CMD_AOC_CMD		91
#define GSA_MB_CMD_UNLOAD_AOC_FW_IMG	92
#define GSA_MB_CMD_SJTAG_GET_STATUS	100
#define GSA_MB_CMD_WAKELOCK_ACQUIRE	150
#define GSA_MB_CMD_WAKELOCK_RELEASE	151

/* Errors (gsa_mbox.c, enum gsa_mb_error) */
#define GSA_MB_OK			0
#define GSA_MB_ERR_INVALID_ARGS		1
#define GSA_MB_ERR_AUTH_FAILED		2
#define GSA_MB_ERR_BUSY			3
#define GSA_MB_ERR_ALREADY_RUNNING	4
#define GSA_MB_ERR_OUT_OF_RESOURCES	5
#define GSA_MB_ERR_BAD_HANDLE		6
#define GSA_MB_ERR_TIMED_OUT		130

/* S2MPU (s2mpu-regs.h) */
#define S2MPU_CTRL0			0x0000

#define GSA_CMD_TIMEOUT_MS		1000
/* Authenticating a 21 MB AoC image takes the GSA a while */
#define GSA_LOAD_TIMEOUT_MS		10000

static bool aoc_query;
module_param(aoc_query, bool, 0444);
MODULE_PARM_DESC(aoc_query, "Expose aoc_state (GSA_MB_CMD_AOC_CMD, GET_STATE) in debugfs");

static bool aoc_cmd_mbox;
module_param(aoc_cmd_mbox, bool, 0444);
MODULE_PARM_DESC(aoc_cmd_mbox, "Send AoC state commands on the mailbox (cmd 91) instead of through Trusty");

static bool tpu_cmd_mbox;
module_param(tpu_cmd_mbox, bool, 0444);
MODULE_PARM_DESC(tpu_cmd_mbox, "Send TPU state commands on the mailbox (cmd 51/52) instead of through Trusty");

static bool s2mpu_probe;
module_param(s2mpu_probe, bool, 0444);
MODULE_PARM_DESC(s2mpu_probe, "Expose s2mpu_ctrl0 (read under a GSA wakelock) in debugfs");

struct gsa_tz;

#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
#define HWMGR_TPU_PORT			"com.android.trusty.gsa.hwmgr.tpu"
#define HWMGR_AOC_PORT			"com.android.trusty.gsa.hwmgr.aoc"
#define HWMGR_CMD_RESP			BIT(31)
#define HWMGR_CMD_STATE_CMD		1
#define HWMGR_CMD_UNLOAD_IMG		2

#define TZ_CON_TIMEOUT_MS		5000
#define TZ_MSG_TIMEOUT_MS		10000
#define TZ_BUF_TIMEOUT_MS		10000

struct gsa_tz {
	struct device *dev;
	const char *port;
	struct tipc_chan *chan;
	struct mutex req_lock;		/* one request at a time */
	struct mutex rsp_lock;		/* rsp_* against the callback */
	struct completion reply;
	void *rsp_buf;
	size_t rsp_size;
	int rsp_res;
};
#endif

struct gsa {
	struct device *dev;
	void __iomem *base;
	void __iomem *s2mpu;
	bool use_mbox;
#if IS_ENABLED(CONFIG_MAILBOX)
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	u32 mbox_rsp[MBOX_SR_NUM];
#endif
	spinlock_t irq_lock;		/* INTMR0 / mbox_rsp synchronization */
	struct mutex cmd_lock;		/* one command in flight */
	struct mutex wake_lock;		/* wake_count, S2MPU */
	struct completion rsp;
	struct dentry *debugfs;
	unsigned int wake_count;
	bool wedged;
#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
	struct gsa_tz tz_aoc;
	struct gsa_tz tz_tpu;
#endif
};

static void gsa_mask_rsp(struct gsa *gsa, bool mask)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&gsa->irq_lock, flags);
	v = readl(gsa->base + MBOX_INTMR0);
	if (mask)
		v |= MBOX_CLIENT_RSP_IRQ;
	else
		v &= ~MBOX_CLIENT_RSP_IRQ;
	writel(v, gsa->base + MBOX_INTMR0);
	spin_unlock_irqrestore(&gsa->irq_lock, flags);
}

static irqreturn_t gsa_irq(int irq, void *data)
{
	struct gsa *gsa = data;

	if (!(readl(gsa->base + MBOX_INTMSR0) & MBOX_CLIENT_RSP_IRQ))
		return IRQ_NONE;

	gsa_mask_rsp(gsa, true);
	complete(&gsa->rsp);

	return IRQ_HANDLED;
}

static int gsa_err_to_errno(u32 err)
{
	switch (err) {
	case GSA_MB_OK:
		return 0;
	case GSA_MB_ERR_INVALID_ARGS:
	case GSA_MB_ERR_BAD_HANDLE:
		return -EINVAL;
	case GSA_MB_ERR_AUTH_FAILED:
		return -EACCES;
	case GSA_MB_ERR_BUSY:
		return -EBUSY;
	case GSA_MB_ERR_ALREADY_RUNNING:
		return -EEXIST;
	case GSA_MB_ERR_OUT_OF_RESOURCES:
		return -ENOMEM;
	case GSA_MB_ERR_TIMED_OUT:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

#if IS_ENABLED(CONFIG_MAILBOX)
static void gsa_mbox_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct gsa *gsa = container_of(cl, struct gsa, mbox_client);
	const u32 *words = mssg;
	unsigned long flags;
	u32 argc, nwords;

	spin_lock_irqsave(&gsa->irq_lock, flags);
	memset(gsa->mbox_rsp, 0, sizeof(gsa->mbox_rsp));
	if (words) {
		argc = READ_ONCE(words[2]);
		/* Copy 3-word header + argc args only; ignore oversize argc */
		nwords = (argc > MBOX_SR_NUM - 3) ? 3 : 3 + argc;
		memcpy(gsa->mbox_rsp, words, nwords * sizeof(u32));
	}
	spin_unlock_irqrestore(&gsa->irq_lock, flags);
	complete(&gsa->rsp);
}

static int gsa_cmd_mbox_locked(struct gsa *gsa, u32 cmd, const u32 *req_args,
			       u32 req_argc, u32 *rsp_args, u32 rsp_max,
			       u32 *gsa_err, unsigned int timeout_ms)
{
	u32 msg[32] = { };
	u32 rsp_cmd, err, argc, i;
	unsigned long flags;
	int ret;

	msg[0] = cmd;
	msg[1] = req_argc;
	for (i = 0; i < req_argc; i++)
		msg[i + 2] = req_args[i];

	reinit_completion(&gsa->rsp);
	ret = mbox_send_message(gsa->mbox_chan, msg);
	if (ret < 0)
		return ret;

	if (!wait_for_completion_timeout(&gsa->rsp,
					 msecs_to_jiffies(timeout_ms))) {
		gsa->wedged = true;
		dev_err(gsa->dev, "cmd %u: no answer in %u ms, mailbox disabled\n",
			cmd, timeout_ms);
		return -ETIMEDOUT;
	}

	spin_lock_irqsave(&gsa->irq_lock, flags);
	rsp_cmd = gsa->mbox_rsp[0];
	err = gsa->mbox_rsp[1];
	argc = gsa->mbox_rsp[2];

	dev_info(gsa->dev, "cmd %u: answer 0x%08x err %u argc %u\n",
		 cmd, rsp_cmd, err, argc);

	if (rsp_cmd != (cmd | GSA_MB_CMD_RSP) || argc > MBOX_SR_NUM - 3) {
		spin_unlock_irqrestore(&gsa->irq_lock, flags);
		return -EIO;
	}

	*gsa_err = err;
	if (err) {
		spin_unlock_irqrestore(&gsa->irq_lock, flags);
		return gsa_err_to_errno(err);
	}

	argc = min(argc, rsp_max);
	for (i = 0; i < argc; i++)
		rsp_args[i] = gsa->mbox_rsp[i + 3];
	spin_unlock_irqrestore(&gsa->irq_lock, flags);

	return argc;
}
#else
static int gsa_cmd_mbox_locked(struct gsa *gsa, u32 cmd, const u32 *req_args,
			       u32 req_argc, u32 *rsp_args, u32 rsp_max,
			       u32 *gsa_err, unsigned int timeout_ms)
{
	return -EOPNOTSUPP;
}
#endif

static int gsa_cmd_mmio_locked(struct gsa *gsa, u32 cmd, const u32 *req_args,
			       u32 req_argc, u32 *rsp_args, u32 rsp_max,
			       u32 *gsa_err, unsigned int timeout_ms)
{
	u32 rsp_cmd, err, argc, i;

	writel(cmd, gsa->base + MBOX_SR(0));
	writel(req_argc, gsa->base + MBOX_SR(1));
	for (i = 0; i < req_argc; i++)
		writel(req_args[i], gsa->base + MBOX_SR(i + 2));

	reinit_completion(&gsa->rsp);
	writel(MBOX_CLIENT_RSP_IRQ, gsa->base + MBOX_INTCR0);
	gsa_mask_rsp(gsa, false);

	dev_dbg(gsa->dev, "cmd %u, %u args\n", cmd, req_argc);
	writel(MBOX_HOST_REQ_IRQ, gsa->base + MBOX_INTGR1);

	if (!wait_for_completion_timeout(&gsa->rsp,
					 msecs_to_jiffies(timeout_ms))) {
		gsa_mask_rsp(gsa, true);
		gsa->wedged = true;
		dev_err(gsa->dev, "cmd %u: no answer in %u ms, mailbox disabled\n",
			cmd, timeout_ms);
		return -ETIMEDOUT;
	}

	rsp_cmd = readl(gsa->base + MBOX_SR(0));
	err = readl(gsa->base + MBOX_SR(1));
	argc = readl(gsa->base + MBOX_SR(2));

	writel(MBOX_CLIENT_RSP_IRQ, gsa->base + MBOX_INTCR0);

	dev_info(gsa->dev, "cmd %u: answer 0x%08x err %u argc %u\n",
		 cmd, rsp_cmd, err, argc);

	if (rsp_cmd != (cmd | GSA_MB_CMD_RSP) || argc > MBOX_SR_NUM - 3)
		return -EIO;

	*gsa_err = err;
	if (err)
		return gsa_err_to_errno(err);

	argc = min(argc, rsp_max);
	for (i = 0; i < argc; i++)
		rsp_args[i] = readl(gsa->base + MBOX_SR(i + 3));
	return argc;
}

/*
 * Run one command. Returns the number of response arguments copied to
 * @rsp_args, or a negative error. The GSA's own error code, if any, is
 * stored in @gsa_err.
 */
static int gsa_cmd_timeout(struct gsa *gsa, u32 cmd, const u32 *req_args,
			   u32 req_argc, u32 *rsp_args, u32 rsp_max,
			   u32 *gsa_err, unsigned int timeout_ms)
{
	int ret;

	if (req_argc + 2 > MBOX_SR_NUM)
		return -EINVAL;

	*gsa_err = 0;

	mutex_lock(&gsa->cmd_lock);

	if (gsa->wedged) {
		ret = -EIO;
		goto out;
	}

	if (gsa->use_mbox)
		ret = gsa_cmd_mbox_locked(gsa, cmd, req_args, req_argc,
					  rsp_args, rsp_max, gsa_err,
					  timeout_ms);
	else
		ret = gsa_cmd_mmio_locked(gsa, cmd, req_args, req_argc,
					  rsp_args, rsp_max, gsa_err,
					  timeout_ms);

out:
	mutex_unlock(&gsa->cmd_lock);
	return ret;
}

static int gsa_cmd(struct gsa *gsa, u32 cmd, const u32 *req_args, u32 req_argc,
		   u32 *rsp_args, u32 rsp_max, u32 *gsa_err)
{
	return gsa_cmd_timeout(gsa, cmd, req_args, req_argc, rsp_args, rsp_max,
			       gsa_err, GSA_CMD_TIMEOUT_MS);
}

/* Keep the GSA core (and its S2MPU) powered. Called with wake_lock held. */
static int gsa_wake_get(struct gsa *gsa)
{
	u32 err;
	int ret;

	if (gsa->wake_count) {
		gsa->wake_count++;
		return 0;
	}

	ret = gsa_cmd(gsa, GSA_MB_CMD_WAKELOCK_ACQUIRE, NULL, 0, NULL, 0, &err);
	if (ret < 0) {
		dev_err(gsa->dev, "wakelock acquire failed: %d (gsa err %u)\n",
			ret, err);
		return ret;
	}
	gsa->wake_count = 1;
	return 0;
}

static void gsa_wake_put(struct gsa *gsa)
{
	u32 err;
	int ret;

	if (WARN_ON(!gsa->wake_count) || --gsa->wake_count)
		return;

	ret = gsa_cmd(gsa, GSA_MB_CMD_WAKELOCK_RELEASE, NULL, 0, NULL, 0, &err);
	if (ret < 0)
		dev_err(gsa->dev, "wakelock release failed: %d (gsa err %u)\n",
			ret, err);
}

/*
 * Let the GSA's DMA through its S2MPU (CTRL0 = 0 passes everything, as
 * Google's pKVM S2MPU driver uses it), and put CTRL0 back afterwards. Called
 * with a wakelock held. Returns the value to restore.
 */
static u32 gsa_s2mpu_open(struct gsa *gsa)
{
	u32 ctrl0;

	if (!gsa->s2mpu)
		return 0;

	ctrl0 = readl(gsa->s2mpu + S2MPU_CTRL0);
	writel(0, gsa->s2mpu + S2MPU_CTRL0);
	dev_info(gsa->dev, "S2MPU CTRL0 %#x -> %#x\n", ctrl0,
		 readl(gsa->s2mpu + S2MPU_CTRL0));
	return ctrl0;
}

static void gsa_s2mpu_restore(struct gsa *gsa, u32 ctrl0)
{
	if (gsa->s2mpu)
		writel(ctrl0, gsa->s2mpu + S2MPU_CTRL0);
}

static int gsa_load_fw_image(struct device *dev, u32 mb_cmd, const char *name,
			     dma_addr_t header, phys_addr_t body)
{
	struct gsa *gsa = dev_get_drvdata(dev);
	bool use_wakelock = !gsa->use_mbox || gsa->s2mpu;
	u32 req[4], err, ctrl0 = 0;
	int ret;

	req[0] = lower_32_bits(header);
	req[1] = upper_32_bits(header);
	req[2] = lower_32_bits(body);
	req[3] = upper_32_bits(body);

	mutex_lock(&gsa->wake_lock);
	if (use_wakelock) {
		ret = gsa_wake_get(gsa);
		if (ret)
			goto out;
		ctrl0 = gsa_s2mpu_open(gsa);
	}

	dev_info(dev, "loading %s image: header %pad body %pa\n",
		 name, &header, &body);
	ret = gsa_cmd_timeout(gsa, mb_cmd, req, 4, NULL, 0,
			      &err, GSA_LOAD_TIMEOUT_MS);
	if (ret > 0)
		ret = 0;
	if (ret)
		dev_err(dev, "%s image load failed: %d (gsa err %u)\n",
			name, ret, err);

	if (use_wakelock) {
		gsa_s2mpu_restore(gsa, ctrl0);
		gsa_wake_put(gsa);
	}
out:
	mutex_unlock(&gsa->wake_lock);
	return ret;
}

/**
 * gsa_load_aoc_fw_image() - authenticate and load an AoC firmware image
 * @dev: the GSA device, from gsa_get_device()
 * @header: DMA address (for the GSA device) of the 4 KiB image header
 * @body: physical address of the image body, at its load address
 *
 * On success the GSA has locked the body: the AP must not access it any
 * more.
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_load_aoc_fw_image(struct device *dev, dma_addr_t header,
			  phys_addr_t body)
{
	return gsa_load_fw_image(dev, GSA_MB_CMD_LOAD_AOC_FW_IMG, "AoC",
				 header, body);
}
EXPORT_SYMBOL_GPL(gsa_load_aoc_fw_image);

/**
 * gsa_load_tpu_fw_image() - authenticate and load a TPU firmware image
 * @dev: the GSA device, from gsa_get_device()
 * @header: DMA address (for the GSA device) of the 4 KiB image header
 * @body: physical address of the image body, at its load address
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_load_tpu_fw_image(struct device *dev, dma_addr_t header,
			  phys_addr_t body)
{
	return gsa_load_fw_image(dev, GSA_MB_CMD_LOAD_TPU_FW_IMG, "TPU",
				 header, body);
}
EXPORT_SYMBOL_GPL(gsa_load_tpu_fw_image);

static int gsa_mbox_state_cmd(struct gsa *gsa, u32 mb_cmd, u32 cmd)
{
	u32 rsp = 0, err;
	int ret;

	ret = gsa_cmd(gsa, mb_cmd, &cmd, 1, &rsp, 1, &err);
	if (ret < 0)
		return ret;
	if (ret < 1)
		return -EIO;
	return rsp;
}

#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
static struct tipc_msg_buf *gsa_tz_handle_msg(void *data,
					      struct tipc_msg_buf *rxbuf)
{
	struct gsa_tz *tz = data;
	size_t len = mb_avail_data(rxbuf);

	mutex_lock(&tz->rsp_lock);
	if (!tz->rsp_buf) {
		dev_err(tz->dev, "TZ: unexpected message, dropped\n");
	} else if (len > tz->rsp_size) {
		dev_err(tz->dev, "TZ: reply too large (%zu > %zu)\n",
			len, tz->rsp_size);
		tz->rsp_res = -EMSGSIZE;
	} else {
		memcpy(tz->rsp_buf, mb_get_data(rxbuf, len), len);
		tz->rsp_res = len;
	}
	mutex_unlock(&tz->rsp_lock);
	complete(&tz->reply);

	return rxbuf;
}

static void gsa_tz_handle_event(void *data, int event)
{
	struct gsa_tz *tz = data;

	complete(&tz->reply);
}

static const struct tipc_chan_ops gsa_tz_ops = {
	.handle_msg = gsa_tz_handle_msg,
	.handle_event = gsa_tz_handle_event,
};

static void gsa_tz_close(struct gsa_tz *tz)
{
	if (tz->chan) {
		tipc_chan_shutdown(tz->chan);
		tipc_chan_destroy(tz->chan);
		tz->chan = NULL;
	}
}

static void gsa_tz_init(struct gsa_tz *tz, struct device *dev, const char *port)
{
	tz->dev = dev;
	tz->port = port;
	mutex_init(&tz->req_lock);
	mutex_init(&tz->rsp_lock);
	init_completion(&tz->reply);
}

static int gsa_tz_connect(struct gsa_tz *tz)
{
	struct tipc_chan *chan;
	int ret;

	chan = tipc_create_channel(NULL, &gsa_tz_ops, tz);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	reinit_completion(&tz->reply);
	ret = tipc_chan_connect(chan, tz->port);
	if (ret < 0)
		goto err;

	/* The connection state is checked when the message is queued */
	if (!wait_for_completion_timeout(&tz->reply,
					 msecs_to_jiffies(TZ_CON_TIMEOUT_MS))) {
		ret = -ETIMEDOUT;
		tipc_chan_shutdown(chan);
		goto err;
	}

	tz->chan = chan;
	dev_info(tz->dev, "TZ: %s connected\n", tz->port);
	return 0;
err:
	tipc_chan_destroy(chan);
	return ret;
}

/* Send @req, wait for the reply. Returns the reply length or an error. */
static int gsa_tz_xchg(struct gsa_tz *tz, const void *req, size_t req_len,
		       void *rsp, size_t rsp_size)
{
	struct tipc_msg_buf *txbuf;
	bool retried = false;
	int ret;

	mutex_lock(&tz->req_lock);
again:
	if (!tz->chan) {
		ret = gsa_tz_connect(tz);
		if (ret) {
			dev_err(tz->dev, "TZ: connect failed: %d\n", ret);
			goto out;
		}
	}

	txbuf = tipc_chan_get_txbuf_timeout(tz->chan,
					    msecs_to_jiffies(TZ_BUF_TIMEOUT_MS));
	if (IS_ERR(txbuf)) {
		ret = PTR_ERR(txbuf);
		goto out;
	}
	memcpy(mb_put_data(txbuf, req_len), req, req_len);

	mutex_lock(&tz->rsp_lock);
	tz->rsp_buf = rsp;
	tz->rsp_size = rsp_size;
	tz->rsp_res = -ENOMSG;
	mutex_unlock(&tz->rsp_lock);

	reinit_completion(&tz->reply);
	ret = tipc_chan_queue_msg(tz->chan, txbuf);
	if (ret < 0) {
		tipc_chan_put_txbuf(tz->chan, txbuf);
		gsa_tz_close(tz);
		/* The service may have closed the connection: retry once */
		if (!retried) {
			retried = true;
			goto again;
		}
		goto detach;
	}

	if (!wait_for_completion_timeout(&tz->reply,
					 msecs_to_jiffies(TZ_MSG_TIMEOUT_MS))) {
		dev_err(tz->dev, "TZ: no reply\n");
		gsa_tz_close(tz);
		ret = -ETIMEDOUT;
		goto detach;
	}
	ret = 0;
detach:
	mutex_lock(&tz->rsp_lock);
	if (!ret)
		ret = tz->rsp_res;
	tz->rsp_buf = NULL;
	tz->rsp_size = 0;
	mutex_unlock(&tz->rsp_lock);
out:
	mutex_unlock(&tz->req_lock);
	return ret;
}

static int gsa_tz_hwmgr_cmd(struct gsa *gsa, struct gsa_tz *tz,
			    const char *name, u32 cmd)
{
	struct {
		u32 hdr_cmd;
		u32 cmd;
	} req = { HWMGR_CMD_STATE_CMD, cmd };
	struct {
		u32 hdr_cmd;
		u32 err;
		u32 state;
	} rsp = { };
	int ret;

	ret = gsa_tz_xchg(tz, &req, sizeof(req), &rsp, sizeof(rsp));
	if (ret < 0)
		return ret;
	if (ret != sizeof(rsp) || rsp.hdr_cmd != (req.hdr_cmd | HWMGR_CMD_RESP)) {
		dev_err(gsa->dev, "TZ: bad reply (len %d, cmd 0x%08x)\n",
			ret, rsp.hdr_cmd);
		return -EIO;
	}
	if (rsp.err) {
		dev_err(gsa->dev, "TZ: %s cmd %u: err %u\n", name, cmd, rsp.err);
		return -EIO;
	}
	return rsp.state;
}

static int gsa_tz_hwmgr_unload(struct gsa *gsa, struct gsa_tz *tz)
{
	u32 req = HWMGR_CMD_UNLOAD_IMG;
	struct {
		u32 hdr_cmd;
		u32 err;
	} rsp = { };
	int ret;

	ret = gsa_tz_xchg(tz, &req, sizeof(req), &rsp, sizeof(rsp));
	if (ret < 0)
		return ret;
	if (ret != sizeof(rsp) || rsp.hdr_cmd != (req | HWMGR_CMD_RESP) || rsp.err)
		return -EIO;
	return 0;
}

static inline struct gsa_tz *gsa_aoc_tz(struct gsa *gsa)
{
	return &gsa->tz_aoc;
}

static inline struct gsa_tz *gsa_tpu_tz(struct gsa *gsa)
{
	return &gsa->tz_tpu;
}
#else
static inline struct gsa_tz *gsa_aoc_tz(struct gsa *gsa)
{
	return NULL;
}

static inline struct gsa_tz *gsa_tpu_tz(struct gsa *gsa)
{
	return NULL;
}

static int gsa_tz_hwmgr_cmd(struct gsa *gsa, struct gsa_tz *tz,
			    const char *name, u32 cmd)
{
	return -EOPNOTSUPP;
}

static int gsa_tz_hwmgr_unload(struct gsa *gsa, struct gsa_tz *tz)
{
	return -EOPNOTSUPP;
}
#endif

static int gsa_send_state_cmd(struct device *dev, struct gsa_tz *tz,
			      bool use_mbox, u32 mb_cmd, const char *name,
			      u32 cmd)
{
	struct gsa *gsa = dev_get_drvdata(dev);
	int ret;

	if (use_mbox)
		ret = gsa_mbox_state_cmd(gsa, mb_cmd, cmd);
	else
		ret = gsa_tz_hwmgr_cmd(gsa, tz, name, cmd);

	dev_info(dev, "%s cmd %u (%s): %d\n", name, cmd,
		 use_mbox ? "mailbox" : "Trusty", ret);
	return ret;
}

static int gsa_unload_fw_image(struct device *dev, struct gsa_tz *tz,
			       bool use_mbox, u32 mb_cmd, const char *name)
{
	struct gsa *gsa = dev_get_drvdata(dev);
	u32 err;
	int ret;

	if (use_mbox) {
		ret = gsa_cmd(gsa, mb_cmd, NULL, 0, NULL, 0, &err);
		if (ret > 0)
			ret = 0;
	} else {
		ret = gsa_tz_hwmgr_unload(gsa, tz);
	}
	dev_info(dev, "%s unload (%s): %d\n", name,
		 use_mbox ? "mailbox" : "Trusty", ret);
	return ret;
}

/**
 * gsa_send_aoc_cmd() - run an AoC management command (GSA_AOC_*)
 * @dev: the GSA device, from gsa_get_device()
 * @cmd: the command
 *
 * Return: the new AoC state (GSA_AOC_STATE_*) or a negative error.
 */
int gsa_send_aoc_cmd(struct device *dev, u32 cmd)
{
	struct gsa *gsa = dev_get_drvdata(dev);

	return gsa_send_state_cmd(dev, gsa_aoc_tz(gsa), aoc_cmd_mbox,
				  GSA_MB_CMD_AOC_CMD, "AoC", cmd);
}
EXPORT_SYMBOL_GPL(gsa_send_aoc_cmd);

/**
 * gsa_unload_aoc_fw_image() - unlock and unload the AoC image
 * @dev: the GSA device, from gsa_get_device()
 *
 * The AoC must be shut down first (GSA_AOC_SHUTDOWN). Afterwards the GSA no
 * longer protects the carve-out and the AoC state is "inactive".
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_unload_aoc_fw_image(struct device *dev)
{
	struct gsa *gsa = dev_get_drvdata(dev);

	return gsa_unload_fw_image(dev, gsa_aoc_tz(gsa), aoc_cmd_mbox,
				   GSA_MB_CMD_UNLOAD_AOC_FW_IMG, "AoC");
}
EXPORT_SYMBOL_GPL(gsa_unload_aoc_fw_image);

/**
 * gsa_aoc_start() - start an authenticated AoC firmware image
 * @dev: the GSA device, from gsa_get_device()
 *
 * Mirrors the Laguna EdgeTPU Trusty/MBA lifecycle: verifies the AoC is in the
 * loaded state, issues GSA_AOC_START via Trusty (com.android.trusty.gsa.hwmgr.aoc)
 * or the mailbox fallback when aoc_cmd_mbox is set, and confirms RUNNING.
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_aoc_start(struct device *dev)
{
	int state;

	state = gsa_send_aoc_cmd(dev, GSA_AOC_GET_STATE);
	if (state < 0)
		return state;

	if (state == GSA_AOC_STATE_RUNNING)
		return 0;

	if (state != GSA_AOC_STATE_LOADED) {
		dev_err(dev, "AoC cannot start from state %d\n", state);
		return -EINVAL;
	}

	state = gsa_send_aoc_cmd(dev, GSA_AOC_START);
	if (state < 0)
		return state;

	if (state != GSA_AOC_STATE_RUNNING) {
		dev_err(dev, "AoC start ended in unexpected state %d\n", state);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gsa_aoc_start);

/**
 * gsa_aoc_stop() - shut down and unload an AoC firmware image
 * @dev: the GSA device, from gsa_get_device()
 *
 * Transitions a running/suspended AoC through GSA_AOC_SHUTDOWN and unlocks the
 * carve-out via gsa_unload_aoc_fw_image(). Safe to call when already inactive.
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_aoc_stop(struct device *dev)
{
	int state, ret;

	state = gsa_send_aoc_cmd(dev, GSA_AOC_GET_STATE);
	if (state < 0)
		return state;

	if (state == GSA_AOC_STATE_INACTIVE)
		return 0;

	if (state == GSA_AOC_STATE_RUNNING ||
	    state == GSA_AOC_STATE_SUSPENDED) {
		ret = gsa_send_aoc_cmd(dev, GSA_AOC_SHUTDOWN);
		if (ret < 0)
			return ret;
	}

	return gsa_unload_aoc_fw_image(dev);
}
EXPORT_SYMBOL_GPL(gsa_aoc_stop);

/**
 * gsa_send_tpu_cmd() - run a TPU management command (enum gsa_tpu_cmd)
 * @dev: the GSA device, from gsa_get_device()
 * @cmd: the command
 *
 * Return: the new TPU state (enum gsa_tpu_state) or a negative error.
 */
int gsa_send_tpu_cmd(struct device *dev, enum gsa_tpu_cmd cmd)
{
	struct gsa *gsa = dev_get_drvdata(dev);

	return gsa_send_state_cmd(dev, gsa_tpu_tz(gsa), tpu_cmd_mbox,
				  GSA_MB_CMD_TPU_CMD, "TPU", cmd);
}
EXPORT_SYMBOL_GPL(gsa_send_tpu_cmd);

/**
 * gsa_unload_tpu_fw_image() - unlock and unload the TPU image
 * @dev: the GSA device, from gsa_get_device()
 *
 * Return: 0 on success, negative error otherwise.
 */
int gsa_unload_tpu_fw_image(struct device *dev)
{
	struct gsa *gsa = dev_get_drvdata(dev);

	return gsa_unload_fw_image(dev, gsa_tpu_tz(gsa), tpu_cmd_mbox,
				   GSA_MB_CMD_UNLOAD_TPU_FW_IMG, "TPU");
}
EXPORT_SYMBOL_GPL(gsa_unload_tpu_fw_image);

/**
 * gsa_get_device() - find the GSA a client's "google,gsa" or "gsa-device" property names
 * @client: the client device
 *
 * Return: the GSA device with a reference held (drop it with
 * gsa_put_device()), or an ERR_PTR(); -EPROBE_DEFER until the GSA has
 * probed.
 */
struct device *gsa_get_device(struct device *client)
{
	struct platform_device *pdev;
	struct device_node *np;

	np = of_parse_phandle(client->of_node, "google,gsa", 0);
	if (!np)
		np = of_parse_phandle(client->of_node, "gsa-device", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	if (!platform_get_drvdata(pdev)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return &pdev->dev;
}
EXPORT_SYMBOL_GPL(gsa_get_device);

void gsa_put_device(struct device *gsa)
{
	put_device(gsa);
}
EXPORT_SYMBOL_GPL(gsa_put_device);

static int gsa_sjtag_status_show(struct seq_file *s, void *unused)
{
	struct gsa *gsa = s->private;
	u32 rsp[3] = { };
	u32 err;
	int ret;

	ret = gsa_cmd(gsa, GSA_MB_CMD_SJTAG_GET_STATUS, NULL, 0,
		      rsp, ARRAY_SIZE(rsp), &err);
	if (ret < 0) {
		seq_printf(s, "error %d (gsa err %u)\n", ret, err);
		return 0;
	}

	seq_printf(s, "args %d: debug_allowed 0x%08x hw_status 0x%08x debug_time 0x%08x\n",
		   ret, rsp[0], rsp[1], rsp[2]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gsa_sjtag_status);

static const char *gsa_aoc_state_name(int state)
{
	static const char * const names[] = {
		"inactive", "loaded", "running", "suspended"
	};

	return state >= 0 && state < ARRAY_SIZE(names) ? names[state] : "?";
}

static int gsa_aoc_state_show(struct seq_file *s, void *unused)
{
	struct gsa *gsa = s->private;
	u32 req = GSA_AOC_GET_STATE;
	u32 rsp[4] = { };
	u32 err;
	int ret;

	ret = gsa_cmd(gsa, GSA_MB_CMD_AOC_CMD, &req, 1, rsp, ARRAY_SIZE(rsp), &err);
	if (ret < 0) {
		seq_printf(s, "error %d (gsa err %u)\n", ret, err);
		return 0;
	}

	seq_printf(s, "args %d: 0x%08x 0x%08x 0x%08x 0x%08x", ret,
		   rsp[0], rsp[1], rsp[2], rsp[3]);
	if (ret >= 1)
		seq_printf(s, " (state %s)", gsa_aoc_state_name(rsp[0]));
	seq_putc(s, '\n');
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gsa_aoc_state);

static int gsa_aoc_state_tz_show(struct seq_file *s, void *unused)
{
	struct gsa *gsa = s->private;
	int ret = gsa_tz_hwmgr_cmd(gsa, gsa_aoc_tz(gsa), "AoC",
				   GSA_AOC_GET_STATE);

	if (ret < 0)
		seq_printf(s, "error %d\n", ret);
	else
		seq_printf(s, "state %d (%s)\n", ret, gsa_aoc_state_name(ret));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gsa_aoc_state_tz);

static const char *gsa_tpu_state_name(int state)
{
	static const char * const names[] = {
		"inactive", "loaded", "running", "suspended"
	};

	return state >= 0 && state < ARRAY_SIZE(names) ? names[state] : "?";
}

static int gsa_tpu_state_tz_show(struct seq_file *s, void *unused)
{
	struct gsa *gsa = s->private;
	int ret = gsa_tz_hwmgr_cmd(gsa, gsa_tpu_tz(gsa), "TPU",
				   GSA_TPU_GET_STATE);

	if (ret < 0)
		seq_printf(s, "error %d\n", ret);
	else
		seq_printf(s, "state %d (%s)\n", ret, gsa_tpu_state_name(ret));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gsa_tpu_state_tz);

static int gsa_s2mpu_ctrl0_show(struct seq_file *s, void *unused)
{
	struct gsa *gsa = s->private;
	int ret;

	mutex_lock(&gsa->wake_lock);
	ret = gsa_wake_get(gsa);
	if (!ret) {
		seq_printf(s, "CTRL0 0x%08x\n", readl(gsa->s2mpu + S2MPU_CTRL0));
		gsa_wake_put(gsa);
	} else {
		seq_printf(s, "error %d (no wakelock, not read)\n", ret);
	}
	mutex_unlock(&gsa->wake_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gsa_s2mpu_ctrl0);

static int gsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gsa *gsa;
	int ret;

	gsa = devm_kzalloc(dev, sizeof(*gsa), GFP_KERNEL);
	if (!gsa)
		return -ENOMEM;

	gsa->dev = dev;
	spin_lock_init(&gsa->irq_lock);
	mutex_init(&gsa->cmd_lock);
	mutex_init(&gsa->wake_lock);
	init_completion(&gsa->rsp);
#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
	gsa_tz_init(&gsa->tz_aoc, dev, HWMGR_AOC_PORT);
	gsa_tz_init(&gsa->tz_tpu, dev, HWMGR_TPU_PORT);
#endif

	if (of_property_present(dev->of_node, "mboxes")) {
		if (!IS_ENABLED(CONFIG_MAILBOX))
			return dev_err_probe(dev, -ENODEV,
					     "CONFIG_MAILBOX disabled\n");
#if IS_ENABLED(CONFIG_MAILBOX)
		gsa->use_mbox = true;
		gsa->mbox_client.dev = dev;
		gsa->mbox_client.tx_block = true;
		gsa->mbox_client.tx_tout = GSA_CMD_TIMEOUT_MS;
		gsa->mbox_client.rx_callback = gsa_mbox_rx_callback;
		gsa->mbox_chan = mbox_request_channel(&gsa->mbox_client, 0);
		if (IS_ERR(gsa->mbox_chan))
			return dev_err_probe(dev, PTR_ERR(gsa->mbox_chan),
					     "failed to request mailbox channel\n");
#endif
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
		if (ret) {
			ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
			if (ret)
				goto err_free_mbox;
		}
	} else {
		struct resource *res;
		int irq;

		gsa->base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(gsa->base))
			return PTR_ERR(gsa->base);

		/* Optional second range: mapped, but only touched under a wakelock */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res) {
			gsa->s2mpu = devm_ioremap_resource(dev, res);
			if (IS_ERR(gsa->s2mpu))
				return PTR_ERR(gsa->s2mpu);
		}

		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;

		/* Image headers for the GSA are kept below 4 GiB, to be safe */
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			return ret;

		/* Known state: all 16 GSA -> AP interrupts masked, as Google does */
		writel(0xffff, gsa->base + MBOX_INTMR0);

		ret = devm_request_irq(dev, irq, gsa_irq, 0, dev_name(dev), gsa);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request IRQ\n");
	}

	gsa->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("sjtag_status", 0400, gsa->debugfs, gsa,
			    &gsa_sjtag_status_fops);
	if (aoc_query)
		debugfs_create_file("aoc_state", 0400, gsa->debugfs, gsa,
				    &gsa_aoc_state_fops);
	if (IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)) {
		debugfs_create_file("aoc_state_tz", 0400, gsa->debugfs, gsa,
				    &gsa_aoc_state_tz_fops);
		debugfs_create_file("tpu_state_tz", 0400, gsa->debugfs, gsa,
				    &gsa_tpu_state_tz_fops);
	}
	if (s2mpu_probe && gsa->s2mpu)
		debugfs_create_file("s2mpu_ctrl0", 0400, gsa->debugfs, gsa,
				    &gsa_s2mpu_ctrl0_fops);

	/* Last: gsa_get_device() treats drvdata as "probed" */
	platform_set_drvdata(pdev, gsa);

	dev_info(dev, "mailbox ready (%s%s%s%s)\n",
		 gsa->use_mbox ? "mbox framework" : "mmio",
		 gsa->s2mpu ? ", S2MPU" : "",
		 IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY) ? ", Trusty" : "",
		 aoc_query ? ", aoc_state query enabled" : "");
	return 0;

err_free_mbox:
#if IS_ENABLED(CONFIG_MAILBOX)
	if (gsa->use_mbox && !IS_ERR_OR_NULL(gsa->mbox_chan))
		mbox_free_channel(gsa->mbox_chan);
#endif
	return ret;
}

static void gsa_remove(struct platform_device *pdev)
{
	struct gsa *gsa = platform_get_drvdata(pdev);

	debugfs_remove_recursive(gsa->debugfs);
#if IS_ENABLED(CONFIG_GOOGLE_GSA_TRUSTY)
	mutex_lock(&gsa->tz_aoc.req_lock);
	gsa_tz_close(&gsa->tz_aoc);
	mutex_unlock(&gsa->tz_aoc.req_lock);

	mutex_lock(&gsa->tz_tpu.req_lock);
	gsa_tz_close(&gsa->tz_tpu);
	mutex_unlock(&gsa->tz_tpu.req_lock);
#endif
#if IS_ENABLED(CONFIG_MAILBOX)
	if (gsa->use_mbox) {
		mbox_free_channel(gsa->mbox_chan);
		return;
	}
#endif
	writel(0xffff, gsa->base + MBOX_INTMR0);
}

static const struct of_device_id gsa_of_match[] = {
	{ .compatible = "google,gs201-gsa" },
	{ .compatible = "google,zumapro-gsa" },
	{ .compatible = "google,gs101-gsa-v1" },
	{ }
};
MODULE_DEVICE_TABLE(of, gsa_of_match);

static struct platform_driver gsa_driver = {
	.probe = gsa_probe,
	.remove = gsa_remove,
	.driver = {
		.name = "google-gsa",
		.of_match_table = gsa_of_match,
	},
};
module_platform_driver(gsa_driver);

MODULE_DESCRIPTION("Google GSA mailbox");
MODULE_LICENSE("GPL");
