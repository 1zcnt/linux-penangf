// SPDX-License-Identifier: GPL-2.0-only
/*
 * Omnivision (formerly Synaptics) TCM touch controller in TDDI chips such as
 * the TD4160, connected over SPI.
 *
 * These controllers have no firmware storage: after every reset they come up
 * in a ROM bootloader and the host has to download the application firmware
 * and its configuration ("host download") before touch reports start.
 *
 * Based on the downstream omnivision_tcm driver:
 *   Copyright (C) 2017-2018 Scott Lin <scott.lin@tw.synaptics.com>
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define TCM_MARKER			0xa5
#define TCM_HEADER_SIZE			4
#define TCM_RD_CHUNK			256

/* Commands */
#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_ENABLE_REPORT		0x05
#define TCM_CMD_RUN_APPLICATION_FW	0x14
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_SET_TOUCH_REPORT_CONFIG	0x26
#define TCM_CMD_DOWNLOAD_CONFIG		0x30
#define TCM_CMD_ROMBOOT_RUN_BOOTLOADER	0x42
#define TCM_CMD_ROMBOOT_DOWNLOAD	0x45

/* Status codes (responses) and report codes share the code byte */
#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_BUSY			0x02
#define TCM_STATUS_CONTINUED_READ	0x03
#define TCM_REPORT_FIRST		0x10
#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11
#define TCM_REPORT_STATUS		0x1b
#define TCM_REPORT_HDL_ROMBOOT		0xfd

#define TCM_MODE_APPLICATION		0x01
#define TCM_MODE_HOSTDOWNLOAD		0x02
#define TCM_MODE_ROMBOOTLOADER		0x04

/* Firmware status, from the status report and the config download response */
#define TCM_FW_NEED_DISP_CONFIG		BIT(1)
#define TCM_FW_NEED_APP_CONFIG		BIT(2)
#define TCM_FW_HDL_VERSION		GENMASK(6, 3)

#define TCM_HDL_TOUCH_CONFIG		1

/* Firmware image */
#define TCM_IMAGE_MAGIC			0x4818472b
#define TCM_AREA_MAGIC			0x7c05e516
#define TCM_AREA_HEADER_SIZE		36
#define TCM_ROMBOOT_RESERVED_BYTES	14

/* Touch report entities, each followed by its width in bits */
#define TCM_TOUCH_END			0
#define TCM_TOUCH_FOREACH_ACTIVE_OBJECT	1
#define TCM_TOUCH_FOREACH_END		3
#define TCM_TOUCH_OBJECT_N_INDEX	6
#define TCM_TOUCH_OBJECT_N_CLASS	7
#define TCM_TOUCH_OBJECT_N_X		8
#define TCM_TOUCH_OBJECT_N_Y		9
#define TCM_TOUCH_OBJECT_N_Z		10
#define TCM_TOUCH_OBJECT_N_X_WIDTH	11
#define TCM_TOUCH_OBJECT_N_Y_WIDTH	12

#define TCM_OBJECT_SIZE			11
#define TCM_OBJECT_LIFT			0
#define TCM_MAX_OBJECTS			16

#define TCM_RESPONSE_TIMEOUT_MS		3000
#define TCM_DOWNLOAD_TIMEOUT_MS		5000
#define TCM_CONFIG_RETRIES		10
/* The firmware file lives on the root filesystem, which may come up late */
#define TCM_FW_RETRY_MS			2000
#define TCM_FW_RETRIES			30

struct ovt_tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
	const char *fw_name;

	/* Only touched from the interrupt thread */
	u8 *in;
	size_t in_size;

	/* Serialises commands; the interrupt thread delivers the response */
	struct mutex cmd_lock;
	spinlock_t resp_lock;
	struct completion response;
	bool cmd_pending;
	bool accept_identify;
	u8 resp_code;
	u8 *resp;
	size_t resp_len;

	struct delayed_work setup_work;
	unsigned int fw_retries;

	u8 mode;
	u16 fw_status;
	bool have_fw_status;
	unsigned int max_objects;
};

static int ovt_tcm_read(struct ovt_tcm *ts, u8 *buf, size_t len)
{
	struct spi_transfer xfer = { .len = len };
	u8 *tx;
	int ret;

	tx = kmalloc(len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	/* The controller wants MOSI high while it is being read */
	memset(tx, 0xff, len);
	xfer.tx_buf = tx;
	xfer.rx_buf = buf;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	kfree(tx);

	return ret;
}

/*
 * Read one message into ts->in. Returns the payload length, with the code in
 * *code, or -ENOMSG if the controller had nothing to say.
 */
static int ovt_tcm_read_message(struct ovt_tcm *ts, u8 *code)
{
	u8 header[TCM_HEADER_SIZE];
	u8 chunk[TCM_RD_CHUNK];
	size_t length, remaining, offset = 0;
	int ret;

	ret = ovt_tcm_read(ts, header, sizeof(header));
	if (ret)
		return ret;

	if (header[0] != TCM_MARKER)
		return -ENOMSG;

	*code = header[1];
	length = get_unaligned_le16(&header[2]);
	if (!length)
		return 0;

	if (length > ts->in_size) {
		u8 *in = krealloc(ts->in, length, GFP_KERNEL);

		if (!in)
			return -ENOMEM;
		ts->in = in;
		ts->in_size = length;
	}

	/* The payload is followed by one padding byte */
	remaining = length + 1;
	while (remaining) {
		size_t n = min(remaining, sizeof(chunk) - 2);

		/* Nothing but the padding byte left: not worth a transfer */
		if (n == 1 && offset == length)
			break;

		ret = ovt_tcm_read(ts, chunk, n + 2);
		if (ret)
			return ret;

		if (chunk[0] != TCM_MARKER ||
		    chunk[1] != TCM_STATUS_CONTINUED_READ) {
			dev_err(&ts->spi->dev,
				"bad continued read: %02x %02x\n",
				chunk[0], chunk[1]);
			return -EIO;
		}

		memcpy(ts->in + offset, &chunk[2], min(n, length - offset));
		offset = min(offset + n, length);
		remaining -= n;
	}

	return length;
}

static int ovt_tcm_write_message(struct ovt_tcm *ts, u8 cmd,
				 const u8 *payload, size_t len)
{
	u8 *buf;
	int ret;

	buf = kvmalloc(len + 3, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = cmd;
	put_unaligned_le16(len, &buf[1]);
	if (len)
		memcpy(&buf[3], payload, len);

	ret = spi_write(ts->spi, buf, len + 3);
	kvfree(buf);

	return ret;
}

/*
 * Send a command and wait for its response. On success the response payload
 * is in ts->resp / ts->resp_len until the next command. Must not be called
 * from the interrupt thread.
 *
 * With @accept_identify an identify report also ends the wait: commands that
 * make the controller start different firmware are answered that way.
 */
static int ovt_tcm_command(struct ovt_tcm *ts, u8 cmd, const u8 *payload,
			   size_t len, unsigned int timeout_ms,
			   bool accept_identify)
{
	struct device *dev = &ts->spi->dev;
	int ret;

	lockdep_assert_held(&ts->cmd_lock);

	spin_lock_irq(&ts->resp_lock);
	reinit_completion(&ts->response);
	ts->cmd_pending = true;
	ts->accept_identify = accept_identify;
	ts->resp_code = TCM_STATUS_IDLE;
	spin_unlock_irq(&ts->resp_lock);

	ret = ovt_tcm_write_message(ts, cmd, payload, len);
	if (ret) {
		dev_err(dev, "command %#04x: write failed: %d\n", cmd, ret);
		goto out;
	}

	if (!wait_for_completion_timeout(&ts->response,
					 msecs_to_jiffies(timeout_ms))) {
		dev_err(dev, "command %#04x: no response\n", cmd);
		ret = -ETIMEDOUT;
		goto out;
	}

	if (ts->resp_code != TCM_STATUS_OK &&
	    ts->resp_code != TCM_REPORT_IDENTIFY) {
		dev_err(dev, "command %#04x: status %#04x\n", cmd,
			ts->resp_code);
		ret = -EIO;
	}

out:
	spin_lock_irq(&ts->resp_lock);
	ts->cmd_pending = false;
	spin_unlock_irq(&ts->resp_lock);

	return ret;
}

/* Hand a response (or identify report) to the waiting command, if any */
static bool ovt_tcm_deliver_response(struct ovt_tcm *ts, u8 code, size_t len)
{
	u8 *resp = NULL;
	bool taken = false;

	if (len) {
		resp = kmemdup(ts->in, len, GFP_KERNEL);
		if (!resp)
			return false;
	}

	spin_lock_irq(&ts->resp_lock);
	if (ts->cmd_pending &&
	    (code != TCM_REPORT_IDENTIFY || ts->accept_identify)) {
		swap(ts->resp, resp);
		ts->resp_len = len;
		ts->resp_code = code;
		ts->cmd_pending = false;
		taken = true;
		complete(&ts->response);
	}
	spin_unlock_irq(&ts->resp_lock);

	kfree(resp);

	return taken;
}

static void ovt_tcm_report_touch(struct ovt_tcm *ts, size_t len)
{
	const u8 *obj = ts->in;
	size_t i;

	if (!ts->input)
		return;

	/*
	 * The report holds the objects whose state changed or that are still
	 * down, laid out as programmed by ovt_tcm_setup_reports().
	 */
	for (i = 0; i + TCM_OBJECT_SIZE <= len; i += TCM_OBJECT_SIZE, obj += TCM_OBJECT_SIZE) {
		unsigned int index = obj[0] & 0x0f;
		unsigned int class = obj[0] >> 4;
		unsigned int x = get_unaligned_le16(&obj[1]);
		unsigned int y = get_unaligned_le16(&obj[3]);
		unsigned int wx = get_unaligned_le16(&obj[5]);
		unsigned int wy = get_unaligned_le16(&obj[7]);
		unsigned int z = get_unaligned_le16(&obj[9]);

		if (index >= ts->max_objects)
			continue;

		input_mt_slot(ts->input, index);
		if (!input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						class != TCM_OBJECT_LIFT))
			continue;

		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, max(wx, wy));
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, min(wx, wy));
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t ovt_tcm_irq(int irq, void *data)
{
	struct ovt_tcm *ts = data;
	struct device *dev = &ts->spi->dev;
	u8 code;
	int len;

	len = ovt_tcm_read_message(ts, &code);
	if (len < 0) {
		if (len != -ENOMSG)
			dev_err_ratelimited(dev, "read failed: %d\n", len);
		/*
		 * Nothing sensible on the bus although the interrupt line is
		 * low, as while the controller is in reset or unpowered. Do
		 * not spin on the level interrupt.
		 */
		usleep_range(5000, 10000);
		return IRQ_HANDLED;
	}

	switch (code) {
	case TCM_STATUS_IDLE:
	case TCM_STATUS_BUSY:
	case TCM_STATUS_CONTINUED_READ:
		break;
	case TCM_REPORT_TOUCH:
		ovt_tcm_report_touch(ts, len);
		break;
	case TCM_REPORT_IDENTIFY:
		if (len < 2)
			break;
		ts->mode = ts->in[1];
		dev_dbg(dev, "identify: mode %#04x, part %.16s\n", ts->mode,
			len >= 18 ? (const char *)&ts->in[2] : "");
		/*
		 * Unless a command was waiting for it, this means the
		 * controller was reset (the display going off does that, the
		 * two share their supplies): everything has to be loaded again.
		 */
		if (!ovt_tcm_deliver_response(ts, code, len) &&
		    ts->mode == TCM_MODE_ROMBOOTLOADER)
			schedule_delayed_work(&ts->setup_work, 0);
		break;
	case TCM_REPORT_HDL_ROMBOOT:
		schedule_delayed_work(&ts->setup_work, 0);
		break;
	case TCM_REPORT_STATUS:
		if (len >= 2) {
			ts->fw_status = get_unaligned_le16(ts->in);
			ts->have_fw_status = true;
			dev_dbg(dev, "firmware status %#06x\n", ts->fw_status);
		}
		break;
	default:
		if (code < TCM_REPORT_FIRST)
			ovt_tcm_deliver_response(ts, code, len);
		else
			dev_dbg(dev, "unhandled report %#04x (%d bytes)\n",
				code, len);
		break;
	}

	return IRQ_HANDLED;
}

struct ovt_tcm_area {
	const u8 *data;
	size_t size;
};

static int ovt_tcm_find_area(struct ovt_tcm *ts, const struct firmware *fw,
			     const char *id, struct ovt_tcm_area *area)
{
	struct device *dev = &ts->spi->dev;
	unsigned int i, count;

	if (fw->size < 8 || get_unaligned_le32(fw->data) != TCM_IMAGE_MAGIC) {
		dev_err(dev, "%s: not a firmware image\n", ts->fw_name);
		return -EINVAL;
	}

	count = get_unaligned_le32(fw->data + 4);
	if (count > (fw->size - 8) / 4)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		u32 addr = get_unaligned_le32(fw->data + 8 + 4 * i);
		const u8 *desc;
		u32 length;

		if (addr > fw->size || fw->size - addr < TCM_AREA_HEADER_SIZE)
			continue;

		desc = fw->data + addr;
		if (get_unaligned_le32(desc) != TCM_AREA_MAGIC)
			continue;

		/* 16 character identifier, padded with spaces */
		if (strncmp((const char *)desc + 4, id, strlen(id)) ||
		    (strlen(id) < 16 && desc[4 + strlen(id)] != ' '))
			continue;

		length = get_unaligned_le32(desc + 28);
		if (length > fw->size - addr - TCM_AREA_HEADER_SIZE)
			return -EINVAL;

		area->data = desc + TCM_AREA_HEADER_SIZE;
		area->size = length;

		if (get_unaligned_le32(desc + 32) !=
		    (crc32(~0, area->data, area->size) ^ ~0)) {
			dev_err(dev, "%s: bad checksum in %s\n", ts->fw_name,
				id);
			return -EINVAL;
		}

		return 0;
	}

	dev_err(dev, "%s: no %s area\n", ts->fw_name, id);

	return -ENOENT;
}

static int ovt_tcm_download_firmware(struct ovt_tcm *ts,
				     const struct firmware *fw)
{
	struct ovt_tcm_area code;
	u8 *buf;
	int ret;

	ret = ovt_tcm_find_area(ts, fw, "ROMBOOT_APP_CODE", &code);
	if (ret)
		return ret;

	buf = kvzalloc(TCM_ROMBOOT_RESERVED_BYTES + code.size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = code.size >> 16;
	memcpy(buf + TCM_ROMBOOT_RESERVED_BYTES, code.data, code.size);

	ts->have_fw_status = false;

	ret = ovt_tcm_command(ts, TCM_CMD_ROMBOOT_DOWNLOAD, buf,
			      TCM_ROMBOOT_RESERVED_BYTES + code.size,
			      TCM_DOWNLOAD_TIMEOUT_MS, true);
	kvfree(buf);
	if (ret)
		return ret;

	/* Start what was just downloaded; it announces itself when running */
	return ovt_tcm_command(ts, TCM_CMD_ROMBOOT_RUN_BOOTLOADER, NULL, 0,
			       TCM_RESPONSE_TIMEOUT_MS, true);
}

static int ovt_tcm_download_config(struct ovt_tcm *ts,
				   const struct firmware *fw)
{
	struct device *dev = &ts->spi->dev;
	struct ovt_tcm_area config;
	unsigned int tries;
	size_t len;
	u8 *buf;
	int ret;

	/* The firmware says what it needs in a status report once it runs */
	for (tries = 0; tries < 50 && !ts->have_fw_status; tries++)
		msleep(20);

	if (!ts->have_fw_status) {
		dev_dbg(dev, "no firmware status report, assuming none needed\n");
		return 0;
	}

	if (!(ts->fw_status & TCM_FW_NEED_APP_CONFIG))
		return 0;

	ret = ovt_tcm_find_area(ts, fw, "APP_CONFIG", &config);
	if (ret)
		return ret;

	/* Two byte header, then the config padded to a multiple of 8 bytes */
	len = 2 + round_up(config.size, 8);
	buf = kvzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = FIELD_GET(TCM_FW_HDL_VERSION, ts->fw_status) == 2 ? 2 : 1;
	buf[1] = TCM_HDL_TOUCH_CONFIG;
	memcpy(buf + 2, config.data, config.size);

	for (tries = 0; tries < TCM_CONFIG_RETRIES; tries++) {
		ret = ovt_tcm_command(ts, TCM_CMD_DOWNLOAD_CONFIG, buf, len,
				      TCM_RESPONSE_TIMEOUT_MS, false);
		if (ret)
			continue;

		if (ts->resp_len >= 2)
			ts->fw_status = get_unaligned_le16(ts->resp);

		if (!(ts->fw_status & TCM_FW_NEED_APP_CONFIG))
			break;

		ret = -EAGAIN;
	}

	kvfree(buf);

	if (ts->fw_status & TCM_FW_NEED_DISP_CONFIG)
		dev_warn(dev, "firmware asks for a display config\n");

	return ret;
}

static int ovt_tcm_register_input(struct ovt_tcm *ts, unsigned int max_x,
				  unsigned int max_y)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "Omnivision TCM Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, max(max_x, max_y),
			     0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, max(max_x, max_y),
			     0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, U8_MAX, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, ts->max_objects,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	ts->input = input;

	return 0;
}

static int ovt_tcm_setup_reports(struct ovt_tcm *ts)
{
	static const u8 report_config[] = {
		TCM_TOUCH_FOREACH_ACTIVE_OBJECT,
		TCM_TOUCH_OBJECT_N_INDEX, 4,
		TCM_TOUCH_OBJECT_N_CLASS, 4,
		TCM_TOUCH_OBJECT_N_X, 16,
		TCM_TOUCH_OBJECT_N_Y, 16,
		TCM_TOUCH_OBJECT_N_X_WIDTH, 16,
		TCM_TOUCH_OBJECT_N_Y_WIDTH, 16,
		TCM_TOUCH_OBJECT_N_Z, 16,
		TCM_TOUCH_FOREACH_END,
		TCM_TOUCH_END,
	};
	static const u8 touch_report = TCM_REPORT_TOUCH;
	struct device *dev = &ts->spi->dev;
	unsigned int max_x, max_y;
	int ret;

	/* Host-downloaded firmware is the application, under its own mode */
	if (ts->mode != TCM_MODE_APPLICATION &&
	    ts->mode != TCM_MODE_HOSTDOWNLOAD) {
		ret = ovt_tcm_command(ts, TCM_CMD_RUN_APPLICATION_FW, NULL, 0,
				      TCM_RESPONSE_TIMEOUT_MS, true);
		if (ret)
			return ret;
	}

	ret = ovt_tcm_command(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
			      TCM_RESPONSE_TIMEOUT_MS, false);
	if (ret)
		return ret;

	/* max_x, max_y and max_objects follow 32 bytes of other information */
	if (ts->resp_len < 38) {
		dev_err(dev, "short application info (%zu bytes)\n",
			ts->resp_len);
		return -EIO;
	}

	max_x = get_unaligned_le16(&ts->resp[32]);
	max_y = get_unaligned_le16(&ts->resp[34]);
	ts->max_objects = clamp_t(unsigned int,
				  get_unaligned_le16(&ts->resp[36]), 1,
				  TCM_MAX_OBJECTS);

	dev_info(dev, "application running: %ux%u, %u objects, status %#x\n",
		 max_x + 1, max_y + 1, ts->max_objects,
		 get_unaligned_le16(&ts->resp[2]));

	ret = ovt_tcm_command(ts, TCM_CMD_SET_TOUCH_REPORT_CONFIG,
			      report_config, sizeof(report_config),
			      TCM_RESPONSE_TIMEOUT_MS, false);
	if (ret)
		return ret;

	if (!ts->input) {
		ret = ovt_tcm_register_input(ts, max_x, max_y);
		if (ret)
			return ret;
	}

	return ovt_tcm_command(ts, TCM_CMD_ENABLE_REPORT, &touch_report, 1,
			       TCM_RESPONSE_TIMEOUT_MS, false);
}

static void ovt_tcm_setup_work(struct work_struct *work)
{
	struct ovt_tcm *ts = container_of(to_delayed_work(work), struct ovt_tcm,
					  setup_work);
	struct device *dev = &ts->spi->dev;
	const struct firmware *fw;
	int ret;

	guard(mutex)(&ts->cmd_lock);

	ret = firmware_request_nowarn(&fw, ts->fw_name, dev);
	if (ret) {
		if (ts->fw_retries++ < TCM_FW_RETRIES) {
			schedule_delayed_work(&ts->setup_work,
					      msecs_to_jiffies(TCM_FW_RETRY_MS));
		} else {
			dev_err(dev, "firmware %s not available: %d\n",
				ts->fw_name, ret);
		}
		return;
	}

	ts->fw_retries = 0;

	if (ts->mode == TCM_MODE_ROMBOOTLOADER) {
		ret = ovt_tcm_download_firmware(ts, fw);
		if (ret) {
			dev_err(dev, "firmware download failed: %d\n", ret);
			goto out;
		}
	}

	ret = ovt_tcm_download_config(ts, fw);
	if (ret) {
		dev_err(dev, "config download failed: %d\n", ret);
		goto out;
	}

	ret = ovt_tcm_setup_reports(ts);
	if (ret)
		dev_err(dev, "report setup failed: %d\n", ret);

out:
	release_firmware(fw);
}

static void ovt_tcm_cancel_work(void *data)
{
	struct ovt_tcm *ts = data;

	cancel_delayed_work_sync(&ts->setup_work);
}

static void ovt_tcm_free_buffers(void *data)
{
	struct ovt_tcm *ts = data;

	kfree(ts->in);
	kfree(ts->resp);
}

static int ovt_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ovt_tcm *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->max_objects = TCM_MAX_OBJECTS;
	mutex_init(&ts->cmd_lock);
	spin_lock_init(&ts->resp_lock);
	init_completion(&ts->response);
	INIT_DELAYED_WORK(&ts->setup_work, ovt_tcm_setup_work);

	ret = devm_add_action_or_reset(dev, ovt_tcm_free_buffers, ts);
	if (ret)
		return ret;

	ts->fw_name = "hdl_firmware.img";
	device_property_read_string(dev, "firmware-name", &ts->fw_name);

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "Failed to get reset-gpios\n");

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ovt_tcm_irq,
					IRQF_ONESHOT, dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	ret = devm_add_action_or_reset(dev, ovt_tcm_cancel_work, ts);
	if (ret)
		return ret;

	/* The controller answers the reset with an identify report */
	msleep(20);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);

	return 0;
}

static const struct of_device_id ovt_tcm_of_match[] = {
	{ .compatible = "omnivision,td4160-touch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match);

static const struct spi_device_id ovt_tcm_id[] = {
	{ "td4160-touch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_id);

static struct spi_driver ovt_tcm_driver = {
	.driver = {
		.name = "omnivision-tcm",
		.of_match_table = ovt_tcm_of_match,
	},
	.probe = ovt_tcm_probe,
	.id_table = ovt_tcm_id,
};
module_spi_driver(ovt_tcm_driver);

MODULE_DESCRIPTION("Omnivision TCM TDDI touchscreen driver");
MODULE_LICENSE("GPL");
