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
#include "esb.h"

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

#include "time_sync.h"

#include <zephyr/kernel.h>
#include <esb.h>

uint8_t discovered_trackers[MAX_TRACKERS] = {0};

typedef struct tracker_info { uint8_t data[10]; } tracker_info_t;
tracker_info_t tracker_infos[MAX_TRACKERS] = {0};

typedef struct tracker_status { uint8_t data[4]; } tracker_status_t;
tracker_status_t tracker_status[MAX_TRACKERS] = {0};

time_sync_t tracker_time_sync[MAX_TRACKERS] = {};
uint64_t last_tracker_rx[MAX_TRACKERS];


// Building blocks: Status (3B), Info (10B), Timestamps (3B), Data(1-14B) - e.g. IMU(12B)
// LEN:  |t:3|id:5|b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |b16     |b17     |b18     |
//    9: |00000000|Checksum|id      |pairing adress                                       |
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

	if (tracker_id >= MAX_TRACKERS)
	{
		LOG_WRN("Received packet from invalid tracker id %d!", tracker_id);
		return;
	}

	//LOG_INF("Received ESB packet %d from tracker %d!", type, tracker_id);
	if (type == TYPE_INFO_STATUS)
	{ // Update into and status and then return
		if (size != 14)
		{
			LOG_ERR("Received info+status packet of size %d (!= 14) from tracker %d!", size, tracker_id);
			return;
		}
		memcpy(tracker_infos[tracker_id].data, data+1, 10);
		memcpy(tracker_status[tracker_id].data, data+11, 3);
		LOG_DBG("Updating info and status!");
	}

	if (discovered_trackers[tracker_id] < DETECTION_THRESHOLD)
	{ // garbage filtering of nonexistent tracker
		if (discovered_trackers[tracker_id] == 0)
			LOG_INF("Received an initial packet from tracker %d!", tracker_id);
		discovered_trackers[tracker_id]++;
		if (discovered_trackers[tracker_id] >= DETECTION_THRESHOLD)
			LOG_INF("Connected to tracker %d!", tracker_id);
		last_tracker_rx[tracker_id] = rx_timestamp;
		return;
	}

	if (type == TYPE_INFO_STATUS)
	{ // Parsed already
		last_tracker_rx[tracker_id] = rx_timestamp;
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
		uint16_t timestamp_last_packet = ((ts[1]&0xF) << 8) | ts[2];
		// 2 LSB were dropped for both (so 14bit in total)
		timestamp_imu <<= 2;
		timestamp_last_packet <<= 2;

		// Initialise time sync for that tracker
		time_sync_t *timesync = &tracker_time_sync[tracker_id];
		if (timesync->measurements == 0)
		{
			LOG_INF("Initialising time sync for tracker %d", tracker_id);
			init_time_sync(timesync);
		}

		// Update timesync using timing of last packet
		uint64_t ts_last = rebase_timestamp_reference(timesync, timestamp_last_packet, 1<<14, last_tracker_rx[tracker_id]);
		update_time_synced(timesync, ts_last, last_tracker_rx[tracker_id]);

		// Determine time of this data packet using remote timestamp
		uint64_t ts_imu = rebase_timestamp_reference(timesync, timestamp_imu, 1<<14, rx_timestamp-4000);
		uint64_t packet_time_us = get_time_synced(timesync, ts_imu);
		LOG_DBG("Received IMU sample from tracker %d with %lldus latency! Raw timestamps are %u IMU %u last",
			tracker_id, (int64_t)rx_timestamp-(int64_t)packet_time_us, timestamp_imu, timestamp_last_packet);

		// Write mapped timestamp after the 12 Bytes of data
		*(uint16_t*)&data[13] = packet_time_us & 0xFFFF;
		size = SIZE_TIMESTAMPED - 3 + 2;
	}
	else
	{
		LOG_DBG("Received non-timesynced packet of size %d from tracker %d with measured time %llu (diff %lld!",
			size, tracker_id, rx_timestamp, (int64_t)rx_timestamp-(int64_t)last_tracker_rx[tracker_id]);
	}
	last_tracker_rx[tracker_id] = rx_timestamp;

	if (size > 15)
	{ // TODO: Support larger packets with their own HID report
		// So it would NOT use the 15-Byte report ringbuffer
		// Currently no usecase for it but might be useful
		LOG_WRN("Received an oversized packet %d of size %d from tracker %d!", type, size, tracker_id);
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
	int register_index = -1;
	while (true)
	{ // reset count if its not above threshold
		k_msleep(1000);

		// Also send registration packet of not connected trackers to server, but only one at a time
		bool sent_register = false;

		for (int i = 0; i < MAX_TRACKERS; i++)
		{
			if (discovered_trackers[i] < DETECTION_THRESHOLD)
			{
				discovered_trackers[i] = 0;
				if (i < stored_trackers && !sent_register && i > register_index)
				{ // Stored tracker is not connected, but inform server it exists
					uint8_t report_register[7];
					report_register[0] = (TYPE_REGISTER << 5) | (i & 0b11111);
					memcpy(&report_register[1], &stored_tracker_addr[i], 6);
					hid_queue_tracker_report(report_register, sizeof(report_register));
					register_index = i;
					sent_register = true;
					LOG_DBG("Sending register packet for non-connected tracker %d!", i);
				}
			}
		}

		if (!sent_register)
			register_index = -1;
	}
}
