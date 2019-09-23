/*
++- * tag_dev.c
 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */


#include <stdio.h>
#include <string.h>
#include <time.h>
#include "stm32l4xx_hal.h"
#include "deca_device_api.h"
#include "device.h"
#include "tag_dev.h"
#include "LoRadriver/LoRadriver.h"

extern UART_HandleTypeDef huart1;
static anch_dist_t anch_dist[4];
#if LPL_MODE
static uint8 state = ASK_SERVER;
#else
static uint8 state = ASK_SERVER;
#endif

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

static uint8 frame_seq_nb = 0;
static int frame_seq_nb2 = 0;
static int sample = 0;
static uint8 ranging_anch = 0;

static uint8 poll_msg[] = {0x01, 0x02, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 resp_msg[] = {0x03, 0x04, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0, 0, 0};
static uint8 final_msg[] = {0x05, 0x06, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 wus_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'X', 'T', 'X', 'R', 0xE1, 0, 0, 0, 0};

static int wu_ok[4] = {0, 0, 0, 0};

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
#define RXBUFFERSIZE 30
uint8 aRxBuffer[RXBUFFERSIZE];
static int nb_try = 0;
int tag_dev(void) {
	dw_device_t * dev = get_device();
	char debug[50];
	int random_wait_time = 0;
	uint8 askmsg[3];
	askmsg[0] = dev->add >> 8;
	askmsg[1] = dev->add;
	askmsg[2] = 0x01;
	uint64 final_tx_time;
	double frame_counter = dev->delay.wuFrame_nb; //WUS_FRAME_NB;
	int ret;

	dwt_setrxaftertxdelay((uint32)dev->delay.tagRespRxDelay_sy);
	dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy);

	dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	double start, end;
	int delay;
	/* Loop forever initiating ranging exchanges. */
	while (1) {
		switch (state) {
			case ASK_SERVER:
				nb_try = 0;
				serverOk = 0;
#if !LORA
#if LPL_MODE
				state = WAKE_UP_ANCHOR;
#else
				state = SEND_POLL;
#endif
				//state = SEND_POLL;
#else
				ret = send_cmsghex_lora(askmsg, 3);
				if (ret == 1) {
					start = get_systime_ms();
					println("\r\n\r\n\r\n[LORA] --- Ask server");
					state = WAIT_SERVER_RESP;
				} else {
					sprintf(debug, "--- CAN'T ASK, err: %d ---", ret);
					println(debug);
					Sleep(1000);
				}
#endif
				break;

			case WAIT_SERVER_RESP:
				state = check_server_resp();
				if (state == WAKE_UP_ANCHOR) {
					end = get_systime_ms();
					if (end < start) {
						end += 17207;
					}
					delay = (int) (end - start);
					sprintf(debug, "[LORA] Answer duration: %.3fs -> wait %.3fs", (float) delay/1000, (float) (serverOk * 1000 - delay) / 1000);
					println(debug);
					if (serverOk * 1000 - delay > 0) {
						Sleep(serverOk * 1000 - delay);
					} else {
						sprintf(debug, "[LORA] Answer duration: %.3fs -> Time passed, retry", (float) delay/1000);
						println(debug);
						state = ASK_SERVER;
						Sleep(5000);
					}
					frame_counter = dev->delay.wuFrame_nb;
				}
				break;

			case WAKE_UP_ANCHOR:
				if (frame_counter == dev->delay.wuFrame_nb) {
					start = get_systime_ms();
					println("\r\n[-wake up sequence started-]");
				}

				state = send_wu_frames(frame_counter--);
				//if (frame_counter == 0) {
					end = get_systime_ms();
					set_ranging_exchange_config();
					sprintf(debug, " -wake up duration: %.2fms-", (end > start) ? (end - start) : (end + 17207 - start));
					println(debug);
				//}
				break;

			case SEND_POLL:
				if (sample == 0) {
					println("\r\n[UWB] --- Start ranging exchange");
					start = get_systime_ms();
				}
				//println("-> poll");
				init_and_send_poll();
				state = WAIT_POLL_TX_CONF;
				break;

			case WAIT_POLL_TX_CONF:
				if (irq_status != IRQ_TX_OK) {
					break;
				} else {
					final_msg_set_ts(&final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
					irq_status = IRQ_NONE;
					state = READ_RESP;
				}
				break;

			case READ_RESP:
				if (irq_status != IRQ_NONE) {
					state = read_response();
				}
				break;

			case SEND_FINAL:
				final_tx_time = dwt_readtxtimestamphi32() + dev->delay.pollTx2FinalTxDelay;
				dwt_setdelayedtrxtime(final_tx_time);
				final_msg[8] = dev->add & 0xFF;
				final_msg[7] = (dev->add >> 8) & 0xFF;
				final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + dev->ant_dly;
				final_msg_set_ts(&final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
				final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

				sample++;
				if (sample == NB_SAMPLES) {
					final_msg[9] = TWR_END;
				} else if (sample == 1) {
					final_msg[9] = TWR_BEGIN;
				} else {
					final_msg[9] = TWR_RUN;
				}
				dwt_writetxdata(sizeof(final_msg), final_msg, 0);
				dwt_writetxfctrl(sizeof(final_msg), 0, 1);

				ret = dwt_starttx(DWT_START_TX_DELAYED);

				if (ret == DWT_SUCCESS)
				{
					state = WAIT_FINAL_TX_CONF;
				} else {
					println("can't send final");
					state = WAIT_NEXT_RANGING;
				}
				break;

			case WAIT_FINAL_TX_CONF:
				if (irq_status != IRQ_TX_OK) {
					break;
				} else {
					irq_status = IRQ_NONE;
					state = CHECK_RESULT;
				}
				break;

			case CHECK_RESULT:
				if (sample == NB_SAMPLES) {
					sample = 0;
					state = WAIT_NEXT_RANGING;
				} else {
					if (dev->config.dataRate == DWT_BR_6M8) {
						Sleep(20);
					} else {
						Sleep(10);
					}
					state = SEND_POLL;
				}
				break;

			case WAIT_NEXT_RANGING:
				// final_msg[9] = 0x23;

				state = ASK_SERVER;
				serverOk = 0;
				end = get_systime_ms();
				frame_seq_nb2++;
				send_result();
				// clear_lora_pending_message();
#if LPL_MODE
				set_lowpowerlistening_config();
				frame_counter = dev->delay.wuFrame_nb; //WUS_FRAME_NB
#endif
				state = ASK_SERVER;
#if LORA
				random_wait_time = 1000; // WAIT_TIME + 1000 * (((int)get_systime_ms()) % 5);
#else
				random_wait_time = 2000;
#endif
				if (end < start) {
					end = end + 17207 - start;
				} else {
					end = end - start;
				}
				sprintf(debug, "[UWB] --- Ranging exchange [%d] finished [duration: %fms]----", frame_seq_nb2, end);
				println(debug);
				frame_seq_nb++;
				Sleep(random_wait_time);
				serverOk = 0;
				break;
		}
	}
}

uint8 check_server_resp() {

	if (wait_lora_status(LW_RX, LORA_TIMEOUT)) {
		println("[LORA] --- Server answered");
		disable_loraIRQ();
#if LPL_MODE
		return WAKE_UP_ANCHOR;
#else
		return SEND_POLL;
#endif
	} else {
		//clear_lora_pending_message();
		send_str_lora("AT+MSG\r\n");
		Sleep(5000);
		println("NO RX, TRY AGAIN");
		return ASK_SERVER;
	}

	/*
	char debug[30];
	uint8 state = WAIT_SERVER_RESP;
	int timeout;
	int rx = 0, ack = 0;
	if (wait_lora_status(LW_ACK, 15)) {
		if (wait_lora_status(LW_DONE, 5)) {
			if (wait_lora_status(LW_RX, LORA_TIMEOUT)) {
				println("[LORA] --- Server answered");
				disable_loraIRQ();
#if LPL_MODE
				return WAKE_UP_ANCHOR;
#else
				return SEND_POLL;
#endif
			} else {
				println("NO RX, TRY AGAIN");
				return ASK_SERVER;
			}
		} else {
			println("NO DONE, TRY AGAIN");
			return ASK_SERVER;
		}
	} else {
		println("NO ACK, TRY AGAIN");
		return ASK_SERVER;
	}*/
}

uint8 send_wu_frames(uint16 frame_counter) {

	while (frame_counter--) {
		wus_msg[WU_MSG_CNTDWN_IDX] = frame_counter & 0xFF;
		wus_msg[WU_MSG_CNTDWN_IDX + 1] = frame_counter >> 8;

		dwt_writetxdata(sizeof(wus_msg), wus_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(wus_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

		dwt_starttx(DWT_START_TX_IMMEDIATE);
		while(irq_status != IRQ_TX_OK)
		{ };
		irq_status = IRQ_NONE;
	}
	//if (frame_counter == 0) {
	println("[-wake up sequence finished-]");
	Sleep(50);
	return SEND_POLL;
	//}


	//wus_msg[ALL_MSG_SN_IDX]++;
	//return WAKE_UP_ANCHOR;
}

void init_and_send_poll() {
	dw_device_t * dev = get_device();
	poll_tx_ts = 0;
	poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
	dwt_writetxfctrl(sizeof(poll_msg), 0, 1);
	poll_msg[8] = dev->add & 0xFF;
	poll_msg[7] = (dev->add >> 8) & 0xFF;
	dwt_setrxaftertxdelay((uint32)dev->delay.tagRespRxDelay_sy);
	dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}
uint8 read_response() {
	char debug[50];
	dw_device_t * dev = get_device();
	uint16 anch_address;
	uint8 final_msg_resp_pos, anch_id;

	if (irq_status == IRQ_RX_OK) {
		anch_address = (rx_buffer[7] << 8 | rx_buffer[8]);
		anch_id = anch_address & 0x03;
		if (is_ranging_anchor(anch_address) == 1) {
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, resp_msg, ALL_MSG_COMMON_LEN) == 0) {
				final_msg_resp_pos = 14 + 4 * anch_id;
				if (sample == 0) {
					final_msg_set_ts(&final_msg[final_msg_resp_pos], anch_dist[anch_id].address); //anch_address);
					//anch_dist[anch_id].address = anch_address;
				} else {
					resp_rx_ts = get_rx_timestamp_u64();
					final_msg_set_ts(&final_msg[final_msg_resp_pos], resp_rx_ts);
					anch_dist[anch_id].dist[sample] = get_previous_dist(&rx_buffer[9]);
				}
			}
		}
	}
	irq_status = IRQ_NONE;

	if (ranging_anch == 3) {
		ranging_anch = 0;
		return SEND_FINAL;
	} else {
		ranging_anch++;
		uint32 next_rx_time;
		next_rx_time = dwt_readtxtimestamphi32() + (ranging_anch + 1) * dev->delay.fixedReplyDelayAnc32h;
		enable_rx(next_rx_time);
		return READ_RESP;
	}
}

void enable_rx(uint32 dlyTime) {
	dw_device_t * dev = get_device();
	dwt_setdelayedtrxtime(dlyTime - dev->delay.preambleDuration32h) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
	{
		dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
		dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy*2); //reconfigure the timeout before enable
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setpreambledetecttimeout(PRE_TIMEOUT); //configure preamble timeout
		dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy); //restore the timeout for next RX enable
	} else {
		//println("delayed");
	}

}

float get_previous_dist(uint8 * dist_field) {
	int offset = 0;
	float res = 0;
	for (int i = 0; i < 4; i++) {
		*((uint8 *) &res + offset) = dist_field[i];
		offset++;
	}
	return res;
}

int is_ranging_anchor(uint16 address) {
	if ((anch_dist[address & 0x03].address == address) || (anch_dist[address & 0x03].address == 0x0000) ) {
		if (anch_dist[address & 0x03].address == 0x0000) {
			anch_dist[address & 0x03].address = address;
		}
		return 1;
	} else {
		return 0;
	}
}


void send_result() {
	dw_device_t * dev = get_device();
	char debug[30];
	uint8 data[35]; // = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	data[0] = dev->add >> 8;
	data[1] = dev->add;
	data[2] = 0x02;
	uint dist_d = 0;
	sprintf(debug, "%04X", dev->add);
	println(debug);
	float total = 0;
	int len = 3;
	for (int i = 0; i < 4; i++) {
		total = get_average_distance(i);
		dist_d = (uint) (total * 1000) % 1000000;
#if CALIBRATE
		if (i == 0) {
			// calibrate_antenna_delay(total);
		}
#endif
		data[len + ADD0] = (anch_dist[i].address >> 8) & 0xFF;
		data[len + ADD1] = anch_dist[i].address & 0xFF;
		sprintf(data + len + DIST, "%0*d", 6, dist_d);
		len += ANCH_LEN;

		sprintf(debug, "anch[%04X] : %2.3fm   [%d] %s (%d)",
				anch_dist[i].address,
				(float)((float)dist_d / 1000.0),
				total != 0 ? ++wu_ok[i] : wu_ok[i],
				total != 0 ? "+" : "-",
				wu_ok[i] == frame_seq_nb2 ? 0 : wu_ok[i] - frame_seq_nb2);
		println(debug);
		anch_dist[i].address = 0x0000;
	}
#if LORA
	for (int i = 0; i < 10; i++) {
		sprintf(debug, "%02X ", data[i]);
		print(debug);
	}

	enable_loraIRQ();
	int ret = 0;
	for (int i = 0; i < 1; i++) {
		ret = send_cmsghex_lora(data, len);
		if(ret != 1) {
			println("data not send");
		} else {
			//if(wait_lora_status(LW_ACK, LORA_TIMEOUT)) {
			if(wait_lora_status(LW_DONE, LORA_TIMEOUT)) {
				lora_status = LW_NONE;
				return;
			} else {
				println("try again");
			}
		}
	}
	println("has not been send");
	lora_status = LW_NONE;
#endif
}

float get_average_distance(int anch_id) {
	float tot = 0;
	int nb_valid = 0;
	for (int i = 0; i < NB_SAMPLES; i++) {
		if ((anch_dist[anch_id].dist[i] > 0) && (anch_dist[anch_id].dist[i] < 2000)) {
			tot += anch_dist[anch_id].dist[i];
			nb_valid++;
			anch_dist[anch_id].dist[i] = 0;
		}
	}
	return (tot / (nb_valid ? nb_valid : 1));
}

void final_msg_set_ts(uint8 *ts_field, uint64 ts) {
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

void rx_ok_tag(const dwt_cb_data_t *cb_data)
{
    int i;
    uint16 anch_address;
    for (i = 0 ; i < RX_BUF_LEN; i++ )
    {
        rx_buffer[i] = 0;
    }
    if (cb_data->datalength <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        anch_address = (rx_buffer[7] << 8 | rx_buffer[8]);
        if (is_ranging_anchor(anch_address)) {
            irq_status = IRQ_RX_OK;
        } else {
            irq_status = IRQ_RX_OK;
        }
    }
}

void rx_to_tag(const dwt_cb_data_t *cb_data) {
    irq_status = IRQ_RX_TO;
}

void rx_err_tag(const dwt_cb_data_t *cb_data) {
    irq_status = IRQ_RX_ERR;
}

void tx_conf_tag(const dwt_cb_data_t *cb_data) {
    irq_status = IRQ_TX_OK;
    poll_tx_ts = get_tx_timestamp_u64();
 }
