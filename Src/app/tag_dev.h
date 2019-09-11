/*
 * tag_dev.h
 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */

#ifndef APP_TAG_DEV_H_
#define APP_TAG_DEV_H_

#define NB_SAMPLES 20
#define WAIT_TIME 5000
#define WUS_FRAME_NB 5614


#define WAKE_UP_ANCHOR     0x01
#define SEND_POLL          0x02
#define READ_RESP          0x03
#define SEND_FINAL         0x04
#define WAIT_NEXT_RANGING  0x05
#define CHECK_RESULT       0x06
#define WAIT_POLL_TX_CONF  0x07
#define WAIT_FINAL_TX_CONF 0x08
#define ASK_SERVER 		   0x09
#define WAIT_SERVER_RESP   0x0a

typedef struct {
	float dist[NB_SAMPLES];
	uint16 address;
} anch_dist_t;

int tag_dev(void);
void init_and_send_poll();
void enable_rx(uint32 dlyTime);
uint8 check_server_resp();
uint8 send_wu_frames(uint16 frame_counter);
uint8 read_response();
float get_previous_dist(uint8 * dist_field);
float get_average_distance(int anch_id);
int is_ranging_anchor(uint16 address);
void send_result();
void final_msg_set_ts(uint8 *ts_field, uint64 ts);


#endif /* APP_TAG_DEV_H_ */
