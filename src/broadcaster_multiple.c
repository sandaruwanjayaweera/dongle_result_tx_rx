/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/settings/settings.h>
#include "cam.h"

#include <zephyr/drivers/uart.h>

#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)		// default 50 ms
#define UART_RX_TIMEOUT 50

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

uint8_t  mac[BT_ADDR_SIZE] = { 0x46, 0xb3, 0x1d, 0xc2, 0x45, 0xf2 };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static struct bt_le_ext_adv *adv;

uint8_t ble_data_received(uint8_t *data, uint16_t len)
{
	int err;

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			// printk("Not able to allocate UART send data buffer");
			return 1;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}

	return 1;
}


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		// printk("UART_TX_DONE\n");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			// printk("Failed to send data over UART\n");
		}

		break;

	case UART_RX_RDY:
		// printk("UART_RX_RDY\n");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		// if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		//     (evt->data.rx.buf[buf->len - 1] == '\r')) {
		if (buf->len >= UART_DATA_SIZE) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		// printk("UART_RX_DISABLED\n");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			// printk("Not able to allocate UART receive buffer. size = %d\n", sizeof(*buf));
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
									100);
			    //    UART_RX_TIMEOUT);

		break;

	case UART_RX_BUF_REQUEST:
		// printk("UART_RX_BUF_REQUEST\n");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			// printk("UART_RX_BUF_REQUEST Not able to allocate UART receive buffer size %d\n", sizeof(*buf));
		}

		break;

	case UART_RX_BUF_RELEASED:
		// printk("UART_RX_BUF_RELEASED\n");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		// printk("UART_TX_ABORTED\n");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		// printk("uart_work_handler: Not able to allocate UART receive buffer\n");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

static int uart_init(void)
{
	int err;
	struct uart_data_t *rx;

	if (!device_is_ready(uart)) {
		// printk("uart_init: UART device not ready\n");
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data),
			      UART_RX_TIMEOUT);
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	// char scan_adr = "F2:45:C2:1D:B3:46";

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
}


uint8_t pkt_count = 0;
union pdu_src_station_id_t pdu_src_station_id_r = { .bit_32 = 0 };
static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];
	uint8_t data_status;
	uint16_t data_len;

	(void)memset(name, 0, sizeof(name));

	data_len = buf->len;
	// bt_data_parse(buf, data_cb, name);

	data_status = BT_HCI_LE_ADV_EVT_TYPE_DATA_STATUS(info->adv_props);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	if(info->addr->a.val[5] == 242 && info->addr->a.val[4] == 69 && info->addr->a.val[3] == 194 && info->addr->a.val[2] == 29 && info->addr->a.val[1] == 179 && info->addr->a.val[0] == 70){	
		// if(buf->data[5] == 1 && buf->data[6] == 2){ 	// pdu_proto_version pdu_message_id
		if(!(pdu_src_station_id_r.bit_32 == ((buf->data[7]<<24) + (buf->data[8]<<16) + (buf->data[9]<<8) + buf->data[10]) && buf->data[4] == pkt_count)){ 	// pkt_count
			pkt_count 	= buf->data[4];
			pdu_src_station_id_r.bit_8[3]	= buf->data[7];
			pdu_src_station_id_r.bit_8[2]	= buf->data[8];
			pdu_src_station_id_r.bit_8[1] 	= buf->data[9];
			pdu_src_station_id_r.bit_8[0] 	= buf->data[10];
			uint8_t data[BLE_ARRAY_MAX];
			data[0] = 0xff;
			data[1] = 0x00;
			data[2] = 0xff;
			data[3] = CAM_DATA_SIZE + 6;
			uint8_t crc = data[3];
			for (size_t i = 0; i < CAM_DATA_SIZE; i++) {
				data[i+4] = buf->data[i+5];
				crc = crc ^ data[i+4];
			}
			data[CAM_DATA_SIZE + 4] = crc;
			data[CAM_DATA_SIZE + 5] = 0xff;
			ble_data_received(data, CAM_DATA_SIZE+6);
			// printk("scan success\n");
		}
		
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

void ble_set_mac(bt_addr_t* addr) {
	struct net_buf *buf;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_BD_ADDR, sizeof(*addr));
	if(!buf) {
		// REPORT_ERROR(-ENOBUFS);
	}

	net_buf_add_mem(buf, addr, sizeof(*addr));

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_BD_ADDR, buf, NULL);
	if(err) {
		// REPORT_ERROR(err);
	}
}

int broadcaster_multiple(void)
{
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U, /* Supply unique SID when creating advertising set */
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_DISABLE_CHAN_38 | BT_LE_ADV_OPT_DISABLE_CHAN_39),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_1, //BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_1, //BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = 0x0030, //BT_GAP_SCAN_FAST_INTERVAL,
		.window     = 0x0030, //BT_GAP_SCAN_FAST_WINDOW,
		.timeout 	= 0x0000, //BT_GAP_PER_ADV_MAX_TIMEOUT, 								// How long the scanner will run before stopping automatically.
	};
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

/* CONFIG_BT_EXT_ADV */
	bt_le_scan_cb_register(&scan_callbacks);
	// printk("Registered scan callbacks\n");
/* CONFIG_BT_EXT_ADV */

	ble_set_mac(&mac);
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		printk("Settings loaded (info %d)\n", err);
	}

	err = uart_init();
	if (err) {
		printk("Uart Init failed (err %d)\n", err);
		return err;
	}

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Start scanning failed (err %d)\n", err);
		return err;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return err;
	}
	// printk("Created advertising set.\n");

	int start_flag = 0, adv_offset = 0, end_flag = 0;
	int cur_state = 0;
	int uart_len = 0, uart_count = 0;
	uint8_t pkt_count = 0;
	for (;;) {
//_________________________________________________(Tx)_________________________________________________________________
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
		if(buf){
			printk("UART data received. %d %d\n", buf->len, adv_offset);
			// if(buf->len < 10){
			// 	for (size_t i = 0; i < buf->len; i++)
			// 	{
			// 		printk("junk data. %d\n", buf->data[i]);
			// 	}
				
			// }
			int offset = 0;
			if(false){
				k_free(buf);
			} else {

				while(offset < (buf->len)){

					if(cur_state == 6 && buf->data[offset] == 0xff){
						cur_state 	= 0;
						end_flag  	= 1;
						adv_offset 	= 0;

						if(start_flag){
							bt_le_ext_adv_stop(adv);
							if (err) {
								printk("Advertising failed to stop (err %d)\n", err);
							}
						}

						err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
						if (err) {
							printk("Failed to set advertising data for set (err %d)\n", err);
						}

						err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
						if (err) {
							printk("Failed to start extended advertising set (err %d)\n", err);
						}
						// k_sleep(K_MSEC(10));
						start_flag 	= 1;
						// printk("state 6 buflen %d offset %d\n", buf->len, offset);
					}
					if(cur_state == 5 && buf->data[offset] == 0xff){
						cur_state 	= 6;
						// printk("state 5 buflen %d offset %d\n", buf->len, offset);
					}
					if(cur_state == 4){
						mfg_data[adv_offset+3] = buf->data[offset];

						if(/*adv_offset == 1 || adv_offset == 2 || adv_offset == 3  || adv_offset == 4 || adv_offset == 5 ||*/ adv_offset == 10 /*|| adv_offset == 30 || adv_offset == 31 || adv_offset == 32 || adv_offset == 33 || adv_offset == 34 || adv_offset == 35 || adv_offset == 36 || adv_offset == 37 || adv_offset == 38 || adv_offset == 39*/){
							printk("state 4 miss %d %d %d offset %d buflen %d uartcnt %d\n", buf->data[offset], buf->data[offset+1], buf->data[offset+2], offset, buf->len, uart_count);
						}

						uart_count++;
						adv_offset++;

						if(uart_count + 6 >= uart_len){
							cur_state 	= 5;
							uart_count 	= 0;
							adv_offset 	= 0;
							// printk("state 4 buflen %d offset %d\n", buf->len, offset);
						}
						// printk("state 4 buflen %d uartlen %d offset %d adv %d\n", buf->len, uart_len, offset, adv_offset);
					}
					if(cur_state == 3){
						cur_state 	= 4;
						uart_len 	= buf->data[offset];
						for(int i = 0; i<BLE_ARRAY_MAX; i++){
							mfg_data[i] = 0x00;
						}
						mfg_data[2] = pkt_count;
						pkt_count++;
						// printk("state 3 buflen %d offset %d\n", buf->len, offset);
					}
					if(cur_state == 2){
						if(buf->data[offset] == 0xff){
							cur_state 	= 3;
						} else {
							cur_state 	= 0;
						}
						// printk("state 2 buflen %d offset %d\n", buf->len, offset);
					}
					if(cur_state == 1){
						if(buf->data[offset] == 0x00){
							cur_state 	= 2;
						} else{
							cur_state 	= 0;
						}
						// printk("state 1 buflen %d offset %d\n", buf->len, offset);
					}
					if(end_flag == 1){
						end_flag = 0;
						offset += 1;
						continue;
					}
					if(cur_state == 0 && buf->data[offset] == 0xff){
						cur_state 	= 1;
						printk("state 0 buflen %d offset %d\n", buf->len, offset);
					}

					offset += 1;
				}
				k_free(buf);

			}
		}
	}

	return 0;
}
