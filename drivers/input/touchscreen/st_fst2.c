// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics FST2 (FingerTip S / Angsana) SPI touchscreen driver
 *
 * Clean-room implementation targeting SPI mode-0 touch controllers using
 * hardware chip ID 0x46 0x54 ("FT"). Handles dual-rail sequencing (VDD 1.8V
 * before AVDD 3.3V while reset is asserted), pre-rev-0x08 length preambles,
 * SysInfo discovery via HDM framebuffer read, and batched FIFO pointer events
 * over either an edge/level interrupt or a 10 ms fallback poller.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define FST2_CMD_REG_SPI_R		0xb1
#define FST2_CMD_REG_SPI_W		0xb2
#define FST2_CMD_HDM_SPI_R		0xb7
#define FST2_CMD_FIFO_SPI_R		0xed
#define FST2_CMD_HW_REG_W		0xfa
#define FST2_CMD_HW_REG_R		0xfb

#define FST2_ADDR_DCHIP_ID		0x2001c000
#define FST2_ADDR_DATA_LEN		0x2001c012
#define FST2_ADDR_SPI4_CONFIG		0x2001f300
#define FST2_SPI4_CFG_VAL		0x60

#define FST2_CHIP_ID_0			0x46
#define FST2_CHIP_ID_1			0x54
#define FST2_REV_NO_LEN_CMD		0x08

#define FST2_FW_REG_SCAN_MODE		0x0010
#define FST2_FW_REG_HDM_REQ		0x0023
#define FST2_FW_REG_FIFO_CNT		0x005f
#define FST2_FW_REG_FIFO_SINGLE		0x0060
#define FST2_HDM_SYSINFO_ADDR		0x0000

#define FST2_SCAN_MODE_HIBERNATE	0x00
#define FST2_SCAN_MODE_ACTIVE		0x01
#define FST2_HDM_REQ_SYS_INFO		0x01

#define FST2_SYSINFO_SIZE		256
#define FST2_SYSINFO_OFF_API_MINOR	6
#define FST2_SYSINFO_OFF_API_MAJOR	7
#define FST2_SYSINFO_OFF_SCR_X		24
#define FST2_SYSINFO_OFF_SCR_Y		26

#define FST2_EVT_NOEVENT		0x00
#define FST2_EVT_READY			0x03
#define FST2_EVT_ENTER_POINT		0x13
#define FST2_EVT_MOTION_POINT		0x23
#define FST2_EVT_LEAVE_POINT		0x33
#define FST2_EVT_TOUCH_HEADER		0x63

#define FST2_FIFO_8B_SIZE		8
#define FST2_FIFO_16B_SIZE		16
#define FST2_FIFO_MAX_EVENTS		32
#define FST2_MAX_SLOTS			10

#define FST2_CS_DELAY_US		10
#define FST2_DEFAULT_SPEED_HZ		20000000
#define FST2_VDD_SETTLE_US		1500
#define FST2_AVDD_SETTLE_US		15000
#define FST2_RESET_PULSE_MS		20
#define FST2_POST_RESET_MS		100
#define FST2_READY_TIMEOUT_MS		500
#define FST2_HDM_TIMEOUT_MS		500
#define FST2_POLL_INTERVAL_MS		10
#define FST2_MAX_XFER_BUF		(FST2_FIFO_MAX_EVENTS * FST2_FIFO_16B_SIZE + 2)

struct fst2_ts {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct regulator *avdd;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct delayed_work poll_work;
	struct mutex bus_lock;

	u8 chip_rev;
	bool need_len_cmd;
	u8 api_major;
	u8 api_minor;
	bool group_fifo;
	u8 evt_size;
	u16 scr_x;
	u16 scr_y;
	u32 size_x;
	u32 size_y;

	u8 tx_buf[32] __aligned(ARCH_DMA_MINALIGN);
	u8 zeros_tx[FST2_MAX_XFER_BUF] __aligned(ARCH_DMA_MINALIGN);
	u8 rx_buf[FST2_MAX_XFER_BUF] __aligned(ARCH_DMA_MINALIGN);
};

static int fst2_spi_write_raw(struct fst2_ts *ts, const u8 *buf, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.len = len,
		.delay = {
			.value = FST2_CS_DELAY_US,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	if (len > sizeof(ts->tx_buf))
		return -EINVAL;

	memcpy(ts->tx_buf, buf, len);
	return spi_sync_transfer(ts->spi, &xfer, 1);
}

static int fst2_send_len_preamble(struct fst2_ts *ts, size_t data_len)
{
	u16 enc;
	u8 pkt[7];

	if (!data_len || data_len > 0x10000)
		return -EINVAL;

	enc = (u16)(data_len - 1);
	pkt[0] = FST2_CMD_HW_REG_W;
	pkt[1] = (FST2_ADDR_DATA_LEN >> 24) & 0xff;
	pkt[2] = (FST2_ADDR_DATA_LEN >> 16) & 0xff;
	pkt[3] = (FST2_ADDR_DATA_LEN >> 8) & 0xff;
	pkt[4] = FST2_ADDR_DATA_LEN & 0xff;
	pkt[5] = enc & 0xff;
	pkt[6] = (enc >> 8) & 0xff;

	return fst2_spi_write_raw(ts, pkt, sizeof(pkt));
}

static int fst2_spi_read_raw(struct fst2_ts *ts, const u8 *cmd, size_t cmd_len,
			     u8 *out, size_t data_len, bool with_len_cmd)
{
	struct spi_transfer xfer[2] = {
		{
			.tx_buf = ts->tx_buf,
			.len = cmd_len,
		},
		{
			.tx_buf = ts->zeros_tx,
			.rx_buf = ts->rx_buf,
			.len = data_len + 1,
			.delay = {
				.value = FST2_CS_DELAY_US,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		},
	};
	int ret;

	if (cmd_len > sizeof(ts->tx_buf) || data_len + 1 > sizeof(ts->rx_buf))
		return -EINVAL;

	if (with_len_cmd && ts->need_len_cmd) {
		ret = fst2_send_len_preamble(ts, data_len);
		if (ret)
			return ret;
	}

	memcpy(ts->tx_buf, cmd, cmd_len);
	ret = spi_sync_transfer(ts->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	/* First RX byte aligns with SPI dummy turnaround cycle. */
	memcpy(out, &ts->rx_buf[1], data_len);
	return 0;
}

static int fst2_write_hw(struct fst2_ts *ts, u32 addr, const u8 *data, size_t len)
{
	u8 pkt[16];
	int ret;

	if (5 + len > sizeof(pkt))
		return -EINVAL;

	pkt[0] = FST2_CMD_HW_REG_W;
	pkt[1] = (addr >> 24) & 0xff;
	pkt[2] = (addr >> 16) & 0xff;
	pkt[3] = (addr >> 8) & 0xff;
	pkt[4] = addr & 0xff;
	memcpy(&pkt[5], data, len);

	mutex_lock(&ts->bus_lock);
	ret = fst2_spi_write_raw(ts, pkt, 5 + len);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int fst2_read_hw(struct fst2_ts *ts, u32 addr, u8 *out, size_t len)
{
	u8 cmd[5] = {
		FST2_CMD_HW_REG_R,
		(addr >> 24) & 0xff,
		(addr >> 16) & 0xff,
		(addr >> 8) & 0xff,
		addr & 0xff,
	};
	int ret;

	mutex_lock(&ts->bus_lock);
	ret = fst2_spi_read_raw(ts, cmd, sizeof(cmd), out, len, false);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int fst2_write_fw(struct fst2_ts *ts, u16 addr, const u8 *data, size_t len)
{
	u8 pkt[16];
	int ret;

	if (3 + len > sizeof(pkt))
		return -EINVAL;

	pkt[0] = FST2_CMD_REG_SPI_W;
	pkt[1] = (addr >> 8) & 0xff;
	pkt[2] = addr & 0xff;
	memcpy(&pkt[3], data, len);

	mutex_lock(&ts->bus_lock);
	if (ts->need_len_cmd) {
		ret = fst2_send_len_preamble(ts, len);
		if (ret) {
			mutex_unlock(&ts->bus_lock);
			return ret;
		}
	}
	ret = fst2_spi_write_raw(ts, pkt, 3 + len);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int fst2_read_fw(struct fst2_ts *ts, u16 addr, u8 *out, size_t len)
{
	u8 cmd[3] = {
		FST2_CMD_REG_SPI_R,
		(addr >> 8) & 0xff,
		addr & 0xff,
	};
	int ret;

	mutex_lock(&ts->bus_lock);
	ret = fst2_spi_read_raw(ts, cmd, sizeof(cmd), out, len, true);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int fst2_read_hdm(struct fst2_ts *ts, u16 addr, u8 *out, size_t len)
{
	u8 cmd[3] = {
		FST2_CMD_HDM_SPI_R,
		(addr >> 8) & 0xff,
		addr & 0xff,
	};
	int ret;

	mutex_lock(&ts->bus_lock);
	ret = fst2_spi_read_raw(ts, cmd, sizeof(cmd), out, len, false);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int fst2_read_fifo_group(struct fst2_ts *ts, u8 *out, size_t len)
{
	u8 cmd[1] = { FST2_CMD_FIFO_SPI_R };
	int ret;

	mutex_lock(&ts->bus_lock);
	ret = fst2_spi_read_raw(ts, cmd, sizeof(cmd), out, len, true);
	mutex_unlock(&ts->bus_lock);
	return ret;
}

static void fst2_pulse_reset(struct fst2_ts *ts, unsigned int post_delay_ms)
{
	if (!ts->reset_gpio)
		return;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(FST2_RESET_PULSE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(post_delay_ms);
}

static void fst2_power_off(void *data)
{
	struct fst2_ts *ts = data;

	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 1);
	if (ts->avdd)
		regulator_disable(ts->avdd);
	if (ts->vdd)
		regulator_disable(ts->vdd);
}

static int fst2_power_on(struct fst2_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	int ret;

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ts->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		if (PTR_ERR(ts->vdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->vdd),
					     "failed to get vdd regulator\n");
		ts->vdd = NULL;
	}

	ts->avdd = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		if (PTR_ERR(ts->avdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->avdd),
					     "failed to get avdd regulator\n");
		ts->avdd = NULL;
	}

	if (ts->vdd) {
		ret = regulator_enable(ts->vdd);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vdd\n");
		usleep_range(FST2_VDD_SETTLE_US, FST2_VDD_SETTLE_US + 500);
	}

	if (ts->avdd) {
		ret = regulator_enable(ts->avdd);
		if (ret) {
			if (ts->vdd)
				regulator_disable(ts->vdd);
			return dev_err_probe(dev, ret, "failed to enable avdd\n");
		}
		usleep_range(FST2_AVDD_SETTLE_US, FST2_AVDD_SETTLE_US + 1000);
	}

	ret = devm_add_action_or_reset(dev, fst2_power_off, ts);
	if (ret)
		return ret;

	if (ts->reset_gpio) {
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
		msleep(FST2_RESET_PULSE_MS);
	}

	return 0;
}

static int fst2_wait_ready(struct fst2_ts *ts)
{
	u8 evt[FST2_FIFO_8B_SIZE];
	ktime_t deadline;
	int ret;

	deadline = ktime_add_ms(ktime_get(), FST2_READY_TIMEOUT_MS);
	do {
		ret = fst2_read_fw(ts, FST2_FW_REG_FIFO_SINGLE, evt, sizeof(evt));
		if (ret)
			return ret;

		if (evt[0] == FST2_EVT_READY)
			return 0;

		usleep_range(10000, 11000);
	} while (ktime_before(ktime_get(), deadline));

	dev_dbg(&ts->spi->dev, "controller ready event not seen before timeout\n");
	return 0;
}

static int fst2_read_sysinfo(struct fst2_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	u8 info[FST2_SYSINFO_SIZE];
	u8 req = FST2_HDM_REQ_SYS_INFO;
	u8 status = 0xff;
	ktime_t deadline;
	int ret;

	ret = fst2_write_fw(ts, FST2_FW_REG_HDM_REQ, &req, 1);
	if (ret)
		return ret;

	deadline = ktime_add_ms(ktime_get(), FST2_HDM_TIMEOUT_MS);
	do {
		ret = fst2_read_fw(ts, FST2_FW_REG_HDM_REQ, &status, 1);
		if (ret)
			return ret;
		if (!status)
			break;
		usleep_range(10000, 11000);
	} while (ktime_before(ktime_get(), deadline));

	if (status)
		return dev_err_probe(dev, -ETIMEDOUT,
				     "SysInfo HDM request stalled (0x%02x)\n",
				     status);

	ret = fst2_read_hdm(ts, FST2_HDM_SYSINFO_ADDR, info, sizeof(info));
	if (ret)
		return ret;

	if (info[0] != FST2_HDM_REQ_SYS_INFO)
		return dev_err_probe(dev, -EIO,
				     "unexpected SysInfo header 0x%02x\n",
				     info[0]);

	ts->api_minor = info[FST2_SYSINFO_OFF_API_MINOR];
	ts->api_major = info[FST2_SYSINFO_OFF_API_MAJOR];
	ts->scr_x = get_unaligned_le16(&info[FST2_SYSINFO_OFF_SCR_X]);
	ts->scr_y = get_unaligned_le16(&info[FST2_SYSINFO_OFF_SCR_Y]);

	ts->group_fifo = ts->api_major > 3 ||
			 (ts->api_major == 3 && ts->api_minor >= 1);
	ts->evt_size = (ts->api_major > 3 ||
			(ts->api_major == 3 && ts->api_minor >= 2)) ?
		       FST2_FIFO_16B_SIZE : FST2_FIFO_8B_SIZE;

	return 0;
}

static bool fst2_process_event(struct fst2_ts *ts, const u8 *evt)
{
	u8 type = evt[0];
	unsigned int tid = evt[1] >> 4;
	u16 x, y, z;
	u8 major, minor;

	switch (type) {
	case FST2_EVT_NOEVENT:
	case FST2_EVT_TOUCH_HEADER:
		return false;
	case FST2_EVT_ENTER_POINT:
	case FST2_EVT_MOTION_POINT:
		if (tid >= FST2_MAX_SLOTS)
			return false;
		if (ts->evt_size == FST2_FIFO_16B_SIZE) {
			x = get_unaligned_le16(&evt[2]);
			y = get_unaligned_le16(&evt[4]);
			z = get_unaligned_le16(&evt[6]);
			minor = evt[8];
			major = evt[9];
		} else {
			x = ((evt[3] & 0x0f) << 8) | evt[2];
			y = (evt[4] << 4) | (evt[3] >> 4);
			z = evt[5];
			minor = evt[6];
			major = evt[7];
		}
		if (x >= ts->size_x)
			x = ts->size_x - 1;
		if (y >= ts->size_y)
			y = ts->size_y - 1;

		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
		return true;
	case FST2_EVT_LEAVE_POINT:
		if (tid >= FST2_MAX_SLOTS)
			return false;
		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		return true;
	default:
		dev_dbg(&ts->spi->dev, "unhandled event 0x%02x\n", type);
		return false;
	}
}

static int fst2_drain_events(struct fst2_ts *ts)
{
	u8 buf[FST2_FIFO_MAX_EVENTS * FST2_FIFO_16B_SIZE];
	u8 cnt;
	unsigned int i;
	bool synced = false;
	int ret;

	if (ts->group_fifo) {
		ret = fst2_read_fw(ts, FST2_FW_REG_FIFO_CNT, &cnt, 1);
		if (ret)
			return ret;
		if (!cnt)
			return 0;
		if (cnt > FST2_FIFO_MAX_EVENTS)
			cnt = FST2_FIFO_MAX_EVENTS;

		ret = fst2_read_fifo_group(ts, buf, cnt * ts->evt_size);
		if (ret)
			return ret;

		for (i = 0; i < cnt; i++) {
			const u8 *evt = &buf[i * ts->evt_size];

			if (evt[0] == FST2_EVT_NOEVENT)
				break;
			synced |= fst2_process_event(ts, evt);
		}
	} else {
		for (i = 0; i < FST2_FIFO_MAX_EVENTS; i++) {
			ret = fst2_read_fw(ts, FST2_FW_REG_FIFO_SINGLE, buf,
					   ts->evt_size);
			if (ret)
				return ret;
			if (buf[0] == FST2_EVT_NOEVENT)
				break;
			synced |= fst2_process_event(ts, buf);
		}
	}

	if (synced) {
		input_mt_sync_frame(ts->input);
		input_sync(ts->input);
	}

	return 0;
}

static irqreturn_t fst2_irq_handler(int irq, void *data)
{
	struct fst2_ts *ts = data;
	int ret;

	ret = fst2_drain_events(ts);
	if (ret)
		dev_err_ratelimited(&ts->spi->dev,
				    "failed to read FIFO events: %d\n", ret);

	return IRQ_HANDLED;
}

static void fst2_poll_work(struct work_struct *work)
{
	struct fst2_ts *ts = container_of(work, struct fst2_ts, poll_work.work);

	fst2_drain_events(ts);
	queue_delayed_work(system_freezable_wq, &ts->poll_work,
			   msecs_to_jiffies(FST2_POLL_INTERVAL_MS));
}

static void fst2_stop_poll(void *data)
{
	struct fst2_ts *ts = data;

	disable_delayed_work_sync(&ts->poll_work);
}

static int fst2_register_input(struct fst2_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	u32 pressure_max;
	int ret;

	if (ts->scr_x >= 2 && ts->scr_y >= 2) {
		ts->size_x = ts->scr_x;
		ts->size_y = ts->scr_y;
	} else {
		device_property_read_u32(dev, "touchscreen-size-x", &ts->size_x);
		device_property_read_u32(dev, "touchscreen-size-y", &ts->size_y);
	}

	if (ts->size_x < 2 || ts->size_y < 2)
		return dev_err_probe(dev, -EINVAL,
				     "invalid touchscreen geometry %ux%u\n",
				     ts->size_x, ts->size_y);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "STMicroelectronics FST2 Touchscreen";
	input->id.bustype = BUS_SPI;

	pressure_max = ts->evt_size == FST2_FIFO_16B_SIZE ? 0xffff : 0xff;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->size_x - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->size_y - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, pressure_max, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);
	if (ts->scr_x >= 2 && ts->scr_y >= 2) {
		unsigned int x_max = ts->scr_x - 1;
		unsigned int y_max = ts->scr_y - 1;

		ts->prop.max_x = x_max;
		ts->prop.max_y = y_max;
		if (ts->prop.swap_x_y)
			swap(x_max, y_max);
		input_abs_set_max(input, ABS_MT_POSITION_X, x_max);
		input_abs_set_max(input, ABS_MT_POSITION_Y, y_max);
	}

	ret = input_mt_init_slots(input, FST2_MAX_SLOTS, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ts->input = input;
	return input_register_device(input);
}

static int fst2_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct fst2_ts *ts;
	u8 spi4_cfg = FST2_SPI4_CFG_VAL;
	u8 scan_mode = FST2_SCAN_MODE_ACTIVE;
	u8 id[3];
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->evt_size = FST2_FIFO_8B_SIZE;
	mutex_init(&ts->bus_lock);
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = FST2_DEFAULT_SPEED_HZ;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ret = fst2_power_on(ts);
	if (ret)
		return ret;

	ret = fst2_write_hw(ts, FST2_ADDR_SPI4_CONFIG, &spi4_cfg, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure SPI4 mode\n");

	ret = fst2_read_hw(ts, FST2_ADDR_DCHIP_ID, id, sizeof(id));
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	if (id[0] != FST2_CHIP_ID_0 || id[1] != FST2_CHIP_ID_1)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected chip ID %02x %02x rev %02x\n",
				     id[0], id[1], id[2]);

	ts->chip_rev = id[2];
	ts->need_len_cmd = ts->chip_rev < FST2_REV_NO_LEN_CMD;

	fst2_pulse_reset(ts, FST2_POST_RESET_MS);

	ret = fst2_wait_ready(ts);
	if (ret)
		return ret;

	ret = fst2_read_sysinfo(ts);
	if (ret)
		return ret;

	ret = fst2_register_input(ts);
	if (ret)
		return ret;

	ret = fst2_write_fw(ts, FST2_FW_REG_SCAN_MODE, &scan_mode, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable active scan\n");

	dev_info(dev,
		 "FST2 chip %02x%02x rev 0x%02x API %u.%u (scr=%ux%u, %ux%u, %uB evt)\n",
		 id[0], id[1], ts->chip_rev, ts->api_major, ts->api_minor,
		 ts->scr_x, ts->scr_y, ts->size_x, ts->size_y, ts->evt_size);

	if (spi->irq > 0) {
		ret = devm_request_threaded_irq(dev, spi->irq, NULL,
						fst2_irq_handler,
						IRQF_ONESHOT,
						dev_name(dev), ts);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request IRQ %d\n",
					     spi->irq);
		return 0;
	}

	INIT_DELAYED_WORK(&ts->poll_work, fst2_poll_work);
	ret = devm_add_action_or_reset(dev, fst2_stop_poll, ts);
	if (ret)
		return ret;

	queue_delayed_work(system_freezable_wq, &ts->poll_work, 0);
	return 0;
}

static int fst2_suspend(struct device *dev)
{
	struct fst2_ts *ts = spi_get_drvdata(to_spi_device(dev));
	u8 scan_mode = FST2_SCAN_MODE_HIBERNATE;
	unsigned int slot;

	if (ts->spi->irq > 0)
		disable_irq(ts->spi->irq);
	else
		disable_delayed_work_sync(&ts->poll_work);

	fst2_write_fw(ts, FST2_FW_REG_SCAN_MODE, &scan_mode, 1);

	for (slot = 0; slot < FST2_MAX_SLOTS; slot++) {
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return 0;
}

static int fst2_resume(struct device *dev)
{
	struct fst2_ts *ts = spi_get_drvdata(to_spi_device(dev));
	u8 scan_mode = FST2_SCAN_MODE_ACTIVE;
	int ret;

	fst2_pulse_reset(ts, FST2_POST_RESET_MS);
	ret = fst2_write_fw(ts, FST2_FW_REG_SCAN_MODE, &scan_mode, 1);
	if (ret)
		dev_warn(dev, "failed to restore active scan on resume: %d\n",
			 ret);

	if (ts->spi->irq > 0) {
		enable_irq(ts->spi->irq);
	} else {
		enable_delayed_work(&ts->poll_work);
		queue_delayed_work(system_freezable_wq, &ts->poll_work, 0);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fst2_pm_ops, fst2_suspend, fst2_resume);

static const struct of_device_id fst2_of_match[] = {
	{ .compatible = "st,fst2" },
	{ }
};
MODULE_DEVICE_TABLE(of, fst2_of_match);

static const struct spi_device_id fst2_spi_ids[] = {
	{ "st_fst2" },
	{ "fst2" },
	{ }
};
MODULE_DEVICE_TABLE(spi, fst2_spi_ids);

static struct spi_driver fst2_driver = {
	.driver = {
		.name = "st-fst2",
		.of_match_table = fst2_of_match,
		.pm = pm_sleep_ptr(&fst2_pm_ops),
	},
	.probe = fst2_probe,
	.id_table = fst2_spi_ids,
};
module_spi_driver(fst2_driver);

MODULE_DESCRIPTION("STMicroelectronics FST2 SPI touchscreen driver");
MODULE_LICENSE("GPL");
