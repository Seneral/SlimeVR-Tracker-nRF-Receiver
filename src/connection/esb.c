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
#include "system/system.h"
#include "connection.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>

#include "esb.h"

static struct esb_payload rx_payload;
//static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  HEADER_PAIR, 0, 0, 0, 0, 0, 0, 0, 0);
//static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0);

static bool esb_paired = false;
static bool esb_pairing = false;
static uint8_t pairing_buf[8] = {0};

uint64_t rx_timestamp = 0;

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:

		rx_timestamp = k_ticks_to_us_floor64(k_uptime_ticks());

		if (!esb_read_rx_payload(&rx_payload)) // zero, rx success
		{
			if (rx_payload.data[0] == HEADER_PAIR)
			{
				if (rx_payload.length != 9)
					LOG_ERR("Received malformed pairing packet of length %d", rx_payload.length);
				else if (!esb_pairing) // This is fine, unpaired tracker closeby
					LOG_WRN("Received pairing packet outside of pairing mode");
				else if (stored_trackers >= MAX_TRACKERS)
					LOG_INF("Ignoring pairing request as %d/%d trackers are already stored!", stored_trackers, MAX_TRACKERS);
				else if (rx_payload.data[1] == tx_payload_pair.data[1])
				{ // Already processed previously, send as response
					LOG_INF("Answering pairing request!");
					esb_write_payload(&tx_payload_pair);
				}
				else
				{ // Copy for other thread to process, could do here as well...
					LOG_INF("Processing pairing request...");
					memcpy(pairing_buf, rx_payload.data+1, 8);
					// Answer with invalid pairing packet to signal we are open to receiving requests
					//tx_payload_pair.data[1] = 0; // Invalidate pairing packet
					// Note that multiple trackers trying to pair at once could conflict
					// TODO: Move processing from pairing thread to here to fix multiple trackers pairing at once conflicting
					esb_write_payload(&tx_payload_pair);
				}
			}
			else
			{
				connection_handle_packet(rx_payload.data, rx_payload.length, rx_payload.rssi);
			}
		}
		else
		{
			LOG_ERR("Error while reading rx packet");
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = buf[5] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

void esb_pair(void)
{
	esb_pairing = true;
	if (stored_trackers >= MAX_TRACKERS)
	{
		LOG_ERR("Cannot pair more than %d/%d trackers!", stored_trackers, MAX_TRACKERS);
		while (true)
		{ // Run indefinitely (User must reset/unplug dongle)
			k_msleep(100);
		}
	}
	LOG_INF("Pairing");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	tx_payload_pair.data[0] = 0; // Signal response as pairing packet
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	*(uint64_t*)&rx_payload.data[CONFIG_ESB_MAX_PAYLOAD_LENGTH-8] = *addr;
	memcpy(&tx_payload_pair.data[3], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	while (true)
	{ // Run indefinitely (User must reset/unplug dongle)
		k_msleep(100);
		if (pairing_buf[0] == 0)
			continue; // No pairing request pending
		uint64_t found_addr = (*(uint64_t *)pairing_buf >> 16) & 0xFFFFFFFFFFFF;
		LOG_INF("Processing pairing request by tracker %012llX", found_addr);
		uint16_t send_tracker_id = stored_trackers; // Use new tracker id
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
			{
				//LOG_INF("Found device linked to id %d with address %012llX", i, found_addr);
				send_tracker_id = i;
			}
		}
		if (send_tracker_id >= MAX_TRACKERS)
		{
			LOG_WRN("Cannot pair more than %d trackers!", MAX_TRACKERS);
			tx_payload_pair.data[1] = 0; // Invalidate pairing packet
			continue;
		}
		uint8_t checksum = crc8_ccitt(0x07, &pairing_buf[2], 6); // make sure the packet is valid
		if (checksum == 0)
			checksum = 8;
		if (checksum != pairing_buf[0])
		{
			LOG_WRN("Pairing request checksum check failed! %d != %d!", checksum, pairing_buf[0]);
			tx_payload_pair.data[1] = 0; // Invalidate pairing packet
			continue;
		}
		if (send_tracker_id == stored_trackers)
		{ // New device, add to NVS
			LOG_INF("Added device on id %d with address %012llX", stored_trackers, found_addr);
			stored_tracker_addr[stored_trackers] = found_addr;
			sys_write(STORED_ADDR_0+stored_trackers, NULL, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
			stored_trackers++;
			sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		}
		// Valid 9-byte response to pairing packet: 0, non-zero checksum, tracker_id, receiver address
		tx_payload_pair.data[1] = checksum;
		tx_payload_pair.data[2] = send_tracker_id;
		// tx_payload_pair will be sent as response to the next request
		memset(pairing_buf, 0, sizeof(pairing_buf));
	}
}

// TODO:
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload_sync.noack = false;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	esb_write_payload(&tx_payload_sync);
}

// TODO:
void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}