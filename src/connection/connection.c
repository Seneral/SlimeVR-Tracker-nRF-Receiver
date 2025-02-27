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
#include "connection.h"
#include "system/system.h"
#include "build_defines.h"
#include "hid.h"

#include <zephyr/kernel.h>
#include <esb.h>

uint8_t discovered_trackers[MAX_TRACKERS] = {0};

typedef struct tracker_info { uint8_t data[10]; } tracker_info_t;
tracker_info_t tracker_infos[MAX_TRACKERS] = {0};

typedef struct tracker_status { uint8_t data[4]; } tracker_status_t;
tracker_status_t tracker_status[MAX_TRACKERS] = {0};

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);


// Building blocks: Status (3B), Info (10B), Timestamps (3B), Data(1-14B) - e.g. IMU(12B)
// LEN:  |t:3|id:5|b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |b16     |b17     |b18     |
//    8: |00000000|Checksum|pairing adress                                       |
//   14: |001|id  |brd_id  |mcu_id  |RESV    |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |batt    |batt_v  |temp    |
// <=15: |XXX|id  |DATA (up to 14B)                                                                                                             |
//   16: |XXX|id  |DATA (12B)                                                                                                 |timestamp imu[12] last[12]|
//   19: |XXX|id  |DATA (12B)                                                                                                 |timestamp imu[12] last[12]|batt    |batt_v  |temp    |
// DATA for TYPE_IMU_CAYLEY (12B, compatible with SIZE_TIMESTAMPED and SIZE_TIMESTAMPED_STATUS):
//                |q0               |q1               |q2               |a0               |a1               |a2               |
// DATA for TYPE_GENERIC_HID (example using full 14B using SIZE_MAX_NORMAL):
//                |001     |Joystick X       |Joystick Y       |Trigger |Buttons |Capacitive Sensors (8x8B?)                                    |

// See  PACKET_HEADER_TYPE and PACKET_RESERVED_SIZES

void connection_handle_packet(uint8_t *data, uint8_t size, uint8_t rssi)
{
	uint8_t header = data[0];
	uint8_t type = header >> 5;
	uint8_t tracker_id = header & 0b11111;

	if (type == TYPE_INFO_STATUS)
	{ // Update into and status and then return
		if (size != 14)
		{
			LOG_ERR("Received info+status packet of size %d (!= 14)!", size);
			return;
		}
		memcpy(tracker_infos[tracker_id].data, data+1, 10);
		memcpy(tracker_status[tracker_id].data, data+11, 3);
		return;
	}

	if (discovered_trackers[tracker_id] < DETECTION_THRESHOLD)
	{ // garbage filtering of nonexistent tracker
		discovered_trackers[tracker_id]++;
		return;
	}

	if (size == SIZE_TIMESTAMPED_STATUS)
	{ // Update status
		memcpy(tracker_status[tracker_id].data, data+16, 3);
		size = SIZE_TIMESTAMPED;
	}

	// Update RSSI in status (signal strength)
	tracker_status[tracker_id].data[3] = rssi;

	if (size == SIZE_TIMESTAMPED)
	{ // Use two 12bit timestamps for time sync and append mapped timestamp
		uint8_t *ts = &data[13];
		uint16_t timestamp_imu = (ts[0] << 4) | (ts[1] >> 4);
		uint16_t timestamp_last_packet = ((ts[1]&0xF) << 4) | ts[2];
		// 2 LSB were dropped for both
		timestamp_imu <<= 2;
		timestamp_last_packet <<= 2;

		// TODO: Update time sync with timestamp_last_packet and rx_timestamp
		// TODO: Determine packet timestamp with time sync and timestamp_imu
		uint32_t packet_time_us = 0;

		// Write mapped timestamp after the 12 Bytes of data
		*(uint16_t*)&data[12] = packet_time_us;
		size = 12 + 2;
	}

	if (size > 15)
	{ // TODO: Support larger packets with their own HID report
		// So it would NOT use the 15-Byte report ringbuffer
		// Currently no usecase for it but might be useful
		return;
	}

	// Forward final packet data to server through HID
	hid_queue_tracker_report(data, size);
}

static void connection_packet_filter_thread(void);
K_THREAD_DEFINE(connection_packet_filter_thread_id, 1024, connection_packet_filter_thread, NULL, NULL, NULL, 6, 0, 0);

static void connection_packet_filter_thread(void)
{
	memset(discovered_trackers, 0, sizeof(discovered_trackers));
	while (true)
	{ // reset count if its not above threshold
		k_msleep(1000);

		for (int i = 0; i < MAX_TRACKERS; i++)
		{
			if (discovered_trackers[i] < DETECTION_THRESHOLD)
			{
				discovered_trackers[i] = 0;
				if (i < stored_trackers)
				{ // Stored tracker is not connected, but inform server it exists
					uint8_t report_register[7];
					report_register[0] = (TYPE_REGISTER << 5) | (i & 0b11111);
					memcpy(&report_register[1], &stored_tracker_addr[i], 6);
					hid_queue_tracker_report(report_register, sizeof(report_register));
				}
			}
		}
	}
}
