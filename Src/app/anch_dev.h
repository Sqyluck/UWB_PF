/*
 * anch_dev.h
 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */

#ifndef APP_ANCH_DEV_H_
#define APP_ANCH_DEV_H_

#define WAIT_WAKE_UP       0x01
#define WAIT_POLL          0x02
#define SEND_RESP          0x03
#define WAIT_RESP_TX_CONF  0x04
#define WAIT_FINAL         0x05
#define REENABLE_RX        0x06
#define PUT_TO_SLEEP       0x07


#define SPEED_OF_LIGHT 299702547

int anch_dev(int init);
void rx_reenable_no_timeout ();
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
uint8 read_final_msg();
void init_lpl_mode();
uint8 prepare_resp();
uint8 wake_up();
int check_timeout(int t);


#endif /* APP_ANCH_DEV_H_ */
