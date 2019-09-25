/*
 * device.h
 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */

#ifndef APP_DEVICE_H_
#define APP_DEVICE_H_

#include "deca_device_api.h"
#include "deca_types.h"
/* @fn
 * @brief
 * @param
 *
 * @return
 * */
#define TAG    0
#define ANCHOR 1
#define LPL_MODE 1
#define CALIBRATE 0
#define LORA 1
#define STAND_BY 0
#define STOP_MODE 1
#define SWITCH_CONFIG 0

#define TEST_ADD 0x00

#define RX_RESPONSE_TURNAROUND 300
#define DW_RX_ON_DELAY 16
#define POLL_MSG_LEN   14
#define RESP_MSG_LEN   17
#define FINAL_MSG_LEN  36
#define WU_MSG_LEN     14

#define ADD0 0
#define ADD1 1
#define DIST 2
#define DIST_LEN 4
#define ANCH_LEN 8

#define TWR_BEGIN 0x52
#define TWR_RUN   0x21
#define TWR_END   0x25

#define TREK_ANTDLY_1  (0xD)
#define TREK_ANTDLY_2  (0xE)
#define TREK_ANTDLY_3  (0xF)
#define TREK_ANTDLY_4  (0x1D)

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)

#define LPL_SHORT_SLEEP_SNOOZE_TIME 4
#define LPL_RX_SNIFF_TIME 2

#define PRE_TIMEOUT 8

#define PARTID_ADDRESS (0x06)

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_ID0 14
#define FINAL_MSG_FINAL_TX_TS_IDX 30
#define FINAL_MSG_TS_LEN 4
#define WU_MSG_CNTDWN_IDX 10

#define DWT_LNA_PA_DISABLE     0x00
#define DWT_LNA_ENABLE         0x01
#define DWT_PA_ENABLE          0x02

#define XTAL_FREQ_HZ 38400000
#define LONG_SLEEP_TIME_MS 1500

#define DUMMY_BUFFER_LEN 700
#define RX_BUF_LEN 36

typedef unsigned long long uint64;

typedef struct
{
	uint64 pollTx2FinalTxDelay;
	uint32 preambleDuration32h;
	int fwtoTime_sy;
	int fwto4RespFrame_sy;
	int wuFrameTime_sy;
	uint32 fixedReplyDelayAnc32h;
	uint32 tagRespRxDelay_sy;
	uint32 wuFrame_nb;
	int respFrame;
} delay_config_t;

typedef struct {
	uint16 add;
	uint16 ant_dly;
	uint8 role;
	uint8 id;
	uint8 eui64[8];
	dwt_config_t config;
	delay_config_t delay;
} dw_device_t;


#define NB_CALIB 6
typedef struct {
	uint16 devId;
	uint16 ant_dly;
}dev_cfg_t;

/* Declaration of interrupt callback */
#define IRQ_TX_OK  0x01
#define IRQ_RX_OK  0x02
#define IRQ_RX_TO  0x03
#define IRQ_RX_ERR 0x04
#define IRQ_NONE   0x05

#define AWAKE  0x01
#define ASLEEP 0x02
uint8 irq_status;
uint8 lpl_status;

static uint8 poll_msg[];
static uint8 resp_msg[];
static uint8 final_msg[];
static uint8 wus_msg[];
uint8 dummy_buffer[];

static uint8 rx_buffer[RX_BUF_LEN];


// function declaration
int convert_devtimeu_to_usec (uint64 devtimeu);
void set_delays(dwt_config_t * config, delay_config_t * delay_config);
void init_config(int role, int dataRate, int init);
void set_interrupt();
void set_local_config();
void enter_stop_mode();
void put_dev_to_sleep();
void set_RFconfiguration();
void hardreset_DW1000();
void set_ranging_exchange_config();
void set_lowpowerlistening_config();
dw_device_t * get_device();
uint64 get_rx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
double get_systime_ms();
double get_systime_us();
double get_systime_s();

void rx_ok_anch(const dwt_cb_data_t *cb_data);
void rx_to_anch(const dwt_cb_data_t *cb_data);
void rx_err_anch(const dwt_cb_data_t *cb_data);
void tx_conf_anch(const dwt_cb_data_t *cb_data);

void rx_ok_tag(const dwt_cb_data_t *cb_data);
void rx_to_tag(const dwt_cb_data_t *cb_data);
void rx_err_tag(const dwt_cb_data_t *cb_data);
void tx_conf_tag(const dwt_cb_data_t *cb_data);

#endif /* APP_DEVICE_H_ */
