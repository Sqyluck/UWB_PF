#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "device.h"
#include "anch_dev.h"
#include "main.h"

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);
typedef signed long long int64;

#define ANCH_DEBUG 1

#if LPL_MODE
static uint8 state = WAIT_WAKE_UP;
#else
static uint8 state = WAIT_POLL;
#endif

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static float distance32 = 0;
static uint8 frame_seq_nb = 0;

static int count = 0;
static double total = 0;
static uint8 poll_msg[] = {0x01, 0x02, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 resp_msg[] = {0x03, 0x04, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 final_msg[] = {0x05, 0x06, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 wus_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'X', 'T', 'X', 'R', 0xE1, 0, 0, 0, 0};

static int ts = 0, te = 0;
static int timeout = -1;
static int sleep_time = 0;
static uint16 anch_chosen;

int anch_dev(int init) {
	char debug[50];
	dw_device_t * dev = get_device();
	int ret = 0;
	uint8 val = 0;
	state = WAIT_WAKE_UP;
#if LPL_MODE
	if (init == 1) {
		put_dev_to_sleep();
		//rx_reenable_no_timeout();
	} else {
		//lpl_status = ASLEEP;
		lpl_status = AWAKE;
		//HAL_Delay(50);
		//port_EnableEXT_IRQ();
		uint32 status = port_GetEXT_IRQStatus();
		sprintf(debug, "%d", status);
		println(debug);
		rx_reenable_no_timeout();
	}
#else
	state = WAIT_POLL;
	rx_reenable_no_timeout();
#endif

    while (1)
    {
    	switch(state) {
    		case WAIT_WAKE_UP:
    			if (irq_status != IRQ_NONE) {
    				timeout = -1;
    				state = wake_up();
    				irq_status = IRQ_NONE;
    				if (state != SEND_RESP) {
    					rx_reenable_no_timeout();
    					// break;
    				} else {
    					//println("don't reenable rx");
    				}
    			} else {
        			if (lpl_status == AWAKE) {
        				if (check_timeout(LONG_SLEEP_TIME_MS + 5000) == 0) {
        					println("******************");
							println("**--WU TIMEOUT--**");
							println("******************");
							timeout = -1;
							state = PUT_TO_SLEEP;
        				}
        			}

    			}
    			break;
    		case WAIT_POLL:
    			if (irq_status != IRQ_NONE) {
    				timeout = -1;
    				state = prepare_resp();
    				irq_status = IRQ_NONE;
    			} else {
    				if (check_timeout(500) == 0) {
        				println("********************");
        				println("**--POLL TIMEOUT--**");
        				println("********************");
        				timeout = -1;
						state = PUT_TO_SLEEP;
					}
    			}
    			break;

    		case SEND_RESP:
    			resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
				dwt_writetxdata(sizeof(resp_msg), resp_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED);// | DWT_RESPONSE_EXPECTED);

				if (ret == DWT_ERROR)
				{
					println("\nResp can't be send");
					state = REENABLE_RX;
				} else {
					state = WAIT_RESP_TX_CONF;
					break;
				}
				break;

    		case WAIT_RESP_TX_CONF:
    			if (irq_status == IRQ_TX_OK) {
    				irq_status = IRQ_NONE;
    				dwt_setrxtimeout(dev->delay.fwtoTime_sy);
					dwt_setdelayedtrxtime((get_rx_timestamp_u64() >> 8) + dev->delay.pollTx2FinalTxDelay - dev->delay.preambleDuration32h) ;

					if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) {
						println("can't reenable delayed");
						dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
						dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy*2); //reconfigure the timeout before enable
						dwt_rxenable(DWT_START_RX_IMMEDIATE);
						dwt_setpreambledetecttimeout(PRE_TIMEOUT); //configure preamble timeout
						dwt_setrxtimeout((uint16)dev->delay.fwto4RespFrame_sy); //restore the timeout for next RX enable
					}
					state = WAIT_FINAL;
    			}
    			break;

    		case WAIT_FINAL:
    			if (irq_status != IRQ_NONE) {
    				state = read_final_msg();
    				irq_status = IRQ_NONE;
    			}
    			break;

    		case REENABLE_RX:
				state = WAIT_POLL;
				irq_status = IRQ_NONE;
				rx_reenable_no_timeout();
    			break;

    		case PUT_TO_SLEEP:
    			put_dev_to_sleep();
				state = WAIT_WAKE_UP;
				irq_status = IRQ_NONE;
    			break;
    	}
     }
}

uint8 wake_up() {
    if (irq_status == IRQ_RX_OK) {
    	rx_buffer[ALL_MSG_SN_IDX] = 0;
		dw_device_t * dev = get_device();
		if (memcmp(rx_buffer, wus_msg, ALL_MSG_COMMON_LEN) == 0) {
			uint16 wus_end_frame_nb;
			uint32 wus_end_time_ms;

			// Calculate the time to wait
			wus_end_frame_nb = (rx_buffer[WU_MSG_CNTDWN_IDX + 1] << 8) + rx_buffer[WU_MSG_CNTDWN_IDX];
			wus_end_time_ms = ((wus_end_frame_nb) * (dev->delay.wuFrameTime_sy + 32)) / 1000;
			anch_chosen = 0;

			int tmp_to;
			tmp_to = 60000;
			//dwt_setrxtimeout(tmp_to);
			//dwt_setpreambledetecttimeout(PRE_TIMEOUT);

			print("W");
			//dwt_configuresleep(DWT_CONFIG | DWT_RX_EN, DWT_WAKE_CS | DWT_WAKE_WK | DWT_SLP_EN);
			//dwt_entersleep();

			// Wait and wake up the device
			Sleep(wus_end_time_ms + 50);
#if ANCH_DEBUG
			sleep_time = wus_end_time_ms;
#endif
            //dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
			//port_wakeup_dw1000_fast();
			//dwt_softreset();
			//set_RFconfiguration();
			//set_ranging_exchange_config();
			// Set interrupt and antenna delay
			dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

			dwt_setrxantennadelay(dev->ant_dly);
			dwt_settxantennadelay(dev->ant_dly);

		    port_EnableEXT_IRQ();
			println("U");
		    //dwt_setrxtimeout(tmp_to);
			//dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	        //dwt_rxenable(DWT_START_RX_IMMEDIATE);
			//rx_reenable_no_timeout();
#if ANCH_DEBUG
			ts = portGetTickCnt();
#endif
			return WAIT_POLL;
		}

		if (memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0) {
			dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

			dwt_setrxantennadelay(dev->ant_dly);
			dwt_settxantennadelay(dev->ant_dly);
			print("D");
			return prepare_resp();
		}
    } else if (irq_status == IRQ_RX_ERR) {
		println("wu err");
	    //rx_reenable_no_timeout();
		//return PUT_TO_SLEEP;
	}
    return WAIT_WAKE_UP;
}

uint8 prepare_resp() {
	dw_device_t * dev = get_device();

	rx_buffer[ALL_MSG_SN_IDX] = 0;
	if (irq_status == IRQ_RX_OK) {
		if (memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0) {
#if ANCH_DEBUG
			if (te == 0) {
				te = portGetTickCnt();
			}
#endif
			uint32 resp_tx_time;
			poll_rx_ts = get_rx_timestamp_u64();
			resp_tx_time = dwt_readrxtimestamphi32() + dev->delay.fixedReplyDelayAnc32h * (dev->id + 1);
			dwt_setdelayedtrxtime((resp_tx_time & 0xFFFFFFFE));
			resp_msg[8] = dev->add & 0xFF;
			resp_msg[7] = (dev->add >> 8) & 0xFF;
			*((float*) &resp_msg[9]) = distance32;
			return SEND_RESP;
		}
	} else if (irq_status == IRQ_RX_ERR) {
		//println("poll err");
	}
	irq_status = IRQ_NONE;
	rx_reenable_no_timeout();
	return WAIT_POLL;
}

uint8 read_final_msg() {
	dw_device_t * dev = get_device();

    double distance_to_correct, distance, tof;
	char debug[30];
	uint8 frame_id;

    if (irq_status == IRQ_RX_OK) {
    	frame_id = rx_buffer[ALL_MSG_SN_IDX];
    	rx_buffer[ALL_MSG_SN_IDX] = 0;
    	if (memcmp(rx_buffer, final_msg, ALL_MSG_COMMON_LEN) == 0) {
    		uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    		uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    		double Ra, Rb, Da, Db;
    		int64 tof_dtu, idx;
    		resp_tx_ts = get_tx_timestamp_u64();
    		final_rx_ts = get_rx_timestamp_u64();
    		uint16 tag_address;

    		idx = FINAL_MSG_RESP_RX_TS_ID0 + FINAL_MSG_TS_LEN * dev->id;
    		final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    		final_msg_get_ts(&rx_buffer[idx], &resp_rx_ts);
    		final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
    		tag_address = (rx_buffer[7] << 8 | rx_buffer[8]);
    		if (rx_buffer[9] == TWR_BEGIN) {
    			count++;
        		anch_chosen = (rx_buffer[idx + 1] << 8 | rx_buffer[idx]);
    			if (dev->add != anch_chosen) {
					sprintf(debug, "Go back to sleep, %04X chosen, id: [%d]", anch_chosen, frame_id);
					println(debug);
#if LPL_MODE
					return PUT_TO_SLEEP;
#else
					Sleep(1000);
					//rx_reenable_no_timeout();
					return REENABLE_RX;
#endif
				} else {
					print("+");
				}
    		} else {
    			poll_rx_ts_32 = (uint32)poll_rx_ts;
				resp_tx_ts_32 = (uint32)resp_tx_ts;
				final_rx_ts_32 = (uint32)final_rx_ts;
				Ra = (double)(resp_rx_ts - poll_tx_ts);
				Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
				Da = (double)(final_tx_ts - resp_rx_ts);
				Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
				tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

				int32 tofi;
				tofi = (int32) tof_dtu ; // make it signed
				if (tofi > 0x7FFFFFFF)  // close up TOF may be negative
				{
				   tofi -= 0x80000000 ;  //
				}

				tof = tofi * DWT_TIME_UNITS;

				distance = tof * SPEED_OF_LIGHT;
				distance_to_correct = distance;
				distance = distance - dwt_getrangebias(dev->config.chan, (float) distance_to_correct, dev->config.prf);

				if ((distance > 0) && (distance < 2000)) {
					distance32 = (float) distance;
#if ANCH_DEBUG == 2
					sprintf(debug, "%2.3fm", distance);
					println(debug);
#endif
					count ++;
					total += (distance32);
				}
				if (rx_buffer[9] == TWR_END) {
					sprintf(debug, "DIST: %2.3fm %04X with %04X, [%d], id: [%d]", (total / (count - 1)), anch_chosen, tag_address, count, frame_id);
					println(debug);
					//calibrate_antenna_delay((double)(total/count));
#if ANCH_DEBUG
					sprintf(debug, "wu --> poll : %dms, -> wait : %dms", te - ts, sleep_time);
					println(debug);
					/*if (sleep_time < 1100) {
						println("S");
					}*/
#endif
					te = 0;
					count = 0;
					total = 0;
#if LPL_MODE
					return PUT_TO_SLEEP;
#else
					//rx_reenable_no_timeout();
					return REENABLE_RX;
#endif
				}
    		}
    	} else {
        	println("j3");
        }
    } else if (irq_status == IRQ_RX_TO) {
    	println("Final timeout");
    } else if (irq_status == IRQ_RX_ERR) {
    	println("Final err");
    } else {
    	println("j2");
    }
    return REENABLE_RX;
}

int check_timeout(int t) {
	if (timeout == -1) {
		timeout = portGetTickCnt() + t;
	} else {
		if (portGetTickCnt() > timeout) {
			return 0;
		}
	}
	return 1;
}

void final_msg_get_ts(const uint8 *ts_field, uint32 *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

void rx_reenable_no_timeout () {
	dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_ok_anch(const dwt_cb_data_t *cb_data) {
    int i;
    char debug[50];
    for (i = 0 ; i < RX_BUF_LEN; i++ ) {
        rx_buffer[i] = 0;
    }

    if (cb_data->datalength <= RX_BUF_LEN){
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    if ((rx_buffer[0] == 0x41) && (rx_buffer[1] == 0x88)) {
		if (lpl_status == ASLEEP) {
			println("wu");
			lpl_status = AWAKE;
		} else {
			print("w");
		}
	    irq_status = IRQ_RX_OK;
    } else {
    	if (lpl_status == ASLEEP) {
    		if ( (rx_buffer[0] == 0x01) || (rx_buffer[0] == 0x05) ) {
    			println("rx but sleeping");
    			put_dev_to_sleep();
    		}
    		println("srx");
    	} else {
    		if (rx_buffer[0] == 0x01) {
    			//print("p");
			} else if (rx_buffer[0] == 0x05) {
				//print("f");
			} else if (rx_buffer[0] == 0x03) {
				//print("e");
			}
    	    irq_status = IRQ_RX_OK;
    	}
    }
}

void rx_to_anch(const dwt_cb_data_t *cb_data) {
	if (lpl_status == ASLEEP) {
		print(":");
		rx_reenable_no_timeout();
	} else {
	    irq_status = IRQ_RX_TO;
	}
}

void rx_err_anch(const dwt_cb_data_t *cb_data) {
	print("r");
    irq_status = IRQ_RX_ERR;
}

void tx_conf_anch(const dwt_cb_data_t *cb_data) {
    irq_status = IRQ_TX_OK;
}
