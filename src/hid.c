/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

static struct k_work report_send;

struct tracker_report {
	uint8_t data[15];
} __packed;

// Ringbuffer of reports
// Need to store potentially multiple packet types by all trackers 
// REPORTS_LENGTH should be divisible by 4 to allow clean wrapping when sending 4 reports at once
#define REPORTS_LENGTH (MAX_TRACKERS*4)
#define REPORT_SIZE sizeof(struct tracker_report)
#define MAX_REPORTS (REPORT_SIZE-4-1) // 4 as currently sending, 1 more for HEADER_SIZE of sending
#define HEADER_SIZE 4
uint8_t report_ringbuffer[HEADER_SIZE + REPORT_SIZE * REPORTS_LENGTH];
uint8_t *reports = report_ringbuffer+HEADER_SIZE;
uint32_t report_count = 0;
uint32_t report_sent = 0;

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void send_report_timer_1ms(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, send_report_timer_1ms, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
	HID_END_COLLECTION,
};
		/* HID_REPORT_SIZE(32),
		HID_REPORT_COUNT(1),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(120),
		HID_REPORT_COUNT(4), */

// Report 1: 4 Bytes HID Packet Header, 4 IMU Packets (15 Byte each), total 64 Bytes
//     HID Header:    |timestamp_last   |RESV             |
//     Data Packet:   |typ|id  |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |
//     IMU Packet:    |111|id  |q0               |q1               |q2               |a0               |a1               |a2               |timestamp_imu    |
// Report 2: 4 Bytes HID Packet Header, 12 Status Packets (5 Byte each), total 64 Bytes
//     HID Header:    |timestamp_last   |RESV             |
//     Status Packet: |000|id  |batt    |batt_v  |temp    |rssi    |

bool usb_enabled = false;
int64_t last_registration_sent = 0;
uint64_t tx_timestamp = 0;

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!stored_trackers) return;
	if (report_count == 0) return;
	int ret, wrote;

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {

		// Write 4-Byte header before first report 
		uint8_t *header = &reports[report_sent*REPORT_SIZE - HEADER_SIZE];
		// Timestamp of LAST HID packet sent, for timesync
		((uint16_t*)header)[0] = tx_timestamp & 0xFFFF;
		// Remaining two header bytes are unused

		// Submit header and 4 reports for HID to send
		ret = hid_int_ep_write(hdev, header, HEADER_SIZE+4*REPORT_SIZE, &wrote);

		for (int i = 0; i < 4; i++)
		{ // Invalidate packets after sending
			reports[(report_sent+i)*REPORT_SIZE+0] = 255;
			reports[(report_sent+i)*REPORT_SIZE+1] = 255;
		}

		// Update report ringbuffer values
		report_count = report_count > 4? report_count-4 : 0;
		report_sent += 4;
		if ((report_sent + 4)*REPORT_SIZE > REPORTS_LENGTH)
			report_sent = 0; // Wrap in ring buffer
		//assert(report_sent%4 == 0);
		if (report_sent%4 != 0)
		{
			LOG_ERR("report_sent %d not aligned anymore!", report_sent);
			report_sent += 4-(report_sent%4);
		}

		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			//LOG_DBG("Report submitted");
		}
	} else { // busy with what
		//LOG_DBG("HID IN endpoint busy");
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	tx_timestamp = k_ticks_to_us_floor64(k_uptime_ticks());
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to send_report_timer_1ms(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	if (report_count >= 4)
		k_work_submit(&report_send);
}

static void send_report_timer_1ms(struct k_timer *dummy)
{
	if (usb_enabled)
		k_work_submit(&report_send);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!configured) {
			int_in_ready_cb(hdev);
			configured = true;
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static int composite_pre_init()
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code");
	}

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void usb_init_thread(void)
{
	k_msleep(1000);
	usb_disable();
	k_msleep(1000); // Wait before enabling USB // TODO: why does it need to wait so long
	usb_enable(status_cb);
	k_work_init(&report_send, send_report);
	memset(reports, 0, sizeof(reports));
	for (int i = 0; i < REPORTS_LENGTH; i++)
	{ // Invalidate packet
		reports[i*REPORT_SIZE+0] = 255;
		reports[i*REPORT_SIZE+1] = 255;
	}
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

void hid_queue_tracker_report(uint8_t *data, uint8_t size)
{
	//assert(size <= REPORT_SIZE);
	for (int i = 0; i < report_count; i++)
	{ // Replace any existing queued report with same header (same type from same tracker)
		uint32_t index = ((report_sent+i)%REPORTS_LENGTH)*REPORT_SIZE;
		if (reports[index] == data[0])
		{ // Same packet type, same tracker, overwrite
			memcpy(&reports[index], data, size);
			// Zeroe remaining bytes (if any)
			memset(&reports[index+size], 0, REPORT_SIZE-size);
			break;
		}
	}
	if (report_count >= MAX_REPORTS) // overflow
		return; // Overflow - minus 5 to keep away from currently sending reports
	uint32_t index = ((report_sent+report_count)%REPORTS_LENGTH)*REPORT_SIZE;
	memcpy(&reports[index], data, size);
	// Zeroe remaining bytes (if any)
	memset(&reports[index+size], 0, REPORT_SIZE-size);
	report_count++;
}
