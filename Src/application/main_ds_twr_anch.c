/*#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
//#include "lcd.h"
#include "deca_spi.h"
#include "port.h"
#include "device.h"

#define APP_NAME "DS TWR ANCHOR"

#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)

#define HERTZ_TO_PPM_MULTIPLIER_CHAN_1     (-1.0e6/3494.4e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_3     (-1.0e6/4492.8e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);


#define MODE_110k 1

#if MODE_110k
static dwt_config_t config = {
    2,               // Channel number.
    DWT_PRF_64M,     // Pulse repetition frequency.
    DWT_PLEN_1024,   // Preamble length. Used in TX only.
    DWT_PAC32,       // Preamble acquisition chunk size. Used in RX only.
    9,               // TX preamble code. Used in TX only.
    9,               // RX preamble code. Used in RX only.
    1,               // 0 to use standard SFD, 1 to use non-standard SFD.
    DWT_BR_110K,     // Data rate.
    DWT_PHRMODE_STD, // PHY header mode.
    (1025 + 64 - 32) // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};
#else
static dwt_config_t config = {
    2,               // Channel number.
    DWT_PRF_64M,     // Pulse repetition frequency.
    DWT_PLEN_128,   // Preamble length. Used in TX only.
    DWT_PAC8,       // Preamble acquisition chunk size. Used in RX only.
    9,               // TX preamble code. Used in TX only.
    9,               // RX preamble code. Used in RX only.
    0,               // 0 to use standard SFD, 1 to use non-standard SFD.
    DWT_BR_6M8,     // Data rate.
    DWT_PHRMODE_STD, // PHY header mode.
    (128 + 1 + 8 - 8) // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};
#endif

static delay_config_t delay_config = {
		0,
		0,
		0,
		0,
		0
};


#define TX_ANT_DLY 16472
#define RX_ANT_DLY 16472

//static uint16 antenna_delay = 16400;

static uint8 rx_poll_msg[] = {0x01, 0x02, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x03, 0x04, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x05, 0x06, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_ID0 14
#define FINAL_MSG_RESP_RX_TS_ID1 18
#define FINAL_MSG_RESP_RX_TS_ID2 22
#define FINAL_MSG_RESP_RX_TS_ID3 26
#define FINAL_MSG_FINAL_TX_TS_IDX 30
#define FINAL_MSG_TS_LEN 4
static uint8 frame_seq_nb = 0;

#define RX_BUF_LEN 36
static uint8 rx_buffer[RX_BUF_LEN];

static uint32 status_reg = 0;

#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS  2750 // 2750
#define RESP_TX_TO_FINAL_RX_DLY_UUS 1000  // 500
#define FINAL_RX_TIMEOUT_UUS        4300 // 3300
#define PRE_TIMEOUT 8
#define NB_SAMPLES 32

#define WAIT_POLL          0x01
#define SEND_RESP          0x02
#define WAIT_RESP_TX_CONF  0x03
#define WAIT_FINAL         0x04
#define REENABLE_RX        0x05

#define IRQ_TX_OK  0x01
#define IRQ_RX_OK  0x02
#define IRQ_RX_TO  0x03
#define IRQ_RX_ERR 0x04
#define IRQ_NONE   0x05

typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;
static int count = 0;
static double total = 0;
static float distance32 = 0;
static uint8 state = WAIT_POLL;
// static uint8 irq_status = IRQ_NONE;
static uint64 rx_timestamp = 0;

static uint32 txPower[16] = { 0x00000000, 0x67676767, 0x67676767, 0x8B8B8B8B, 0x9A9A9A9A, 0x85858585, 0x00000000 ,0xD1D1D1D1,
							  0x00000000, 0x07274767, 0x07274767, 0x2B4B6B8B, 0x3A5A7A9A, 0x25456585, 0x00000000 ,0x5171B1D1};
static uint8 PGdelay[8] = { 0x00, 0xC9, 0xC2, 0xC5, 0x95, 0xC0, 0x00, 0x93 };
static dwt_txconfig_t txconfig;

#define SPEED_OF_LIGHT 299702547

static double tof;
static double distance;
static uint8 address;
static int final_msg_resp_ts_id;

char lcd_str[16] = {0};
char lcd_str2[16] = {0};
static uint32 start_ts;
static uint32 resp_tx;

static uint16 ant_dly = 16400;
static int ant_dly_fixed = 0;

static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static uint8 prepare_resp();
static void read_final_msg();
static void set_antenna_delays(dwt_config_t * config, int mode);
void rx_reenable_no_timeout();

static void rx_ok_anch(const dwt_cb_data_t *cb_data);
static void rx_to_anch(const dwt_cb_data_t *cb_data);
static void rx_err_anch(const dwt_cb_data_t *cb_data);
static void tx_conf_anch(const dwt_cb_data_t *cb_data);
int main_ds_twr_anch(void)
{
	char debug[50];
	int ret = 0;
    uint8 smartPower = 0;
    port_DisableEXT_IRQ();
    uint32 currenttime;

	address = 0 << 1 | 0;
	final_msg_resp_ts_id = 14 + 4 * address;

	sprintf(debug, "add: %d, pos: %d", address, final_msg_resp_ts_id);
	println(debug);

    reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        { };
    }
    port_set_dw1000_fastrate();

    init_delays(&config, &delay_config);
    print_config(&delay_config);
    dwt_configure(&config);

    txconfig.power = txPower[8 * smartPower + config.chan];
	txconfig.PGdly = PGdelay[config.chan];
	dwt_configuretxrf(&txconfig);

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
	dwt_setcallbacks(&tx_conf_anch, &rx_ok_anch, &rx_to_anch, &rx_err_anch);

    port_EnableEXT_IRQ();

    //dwt_setrxantennadelay(RX_ANT_DLY);
    //dwt_settxantennadelay(TX_ANT_DLY);
    //set_antenna_delays(&config, 1);
    dwt_setrxantennadelay(ant_dly);
	dwt_settxantennadelay(ant_dly);

    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    rx_reenable_no_timeout();

    while (1)
    {
    	switch(state) {
    		case WAIT_POLL:
    			if (irq_status != IRQ_NONE) {
    				state = prepare_resp();
    				irq_status = IRQ_NONE;
    			}
    			break;

    		case SEND_RESP:
    			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
				ret = dwt_starttx(DWT_START_TX_DELAYED);// | DWT_RESPONSE_EXPECTED);

				if (ret == DWT_ERROR)
				{
					println("\nResp can't be send");
					state = REENABLE_RX;
				} else {
					state = WAIT_RESP_TX_CONF;
					break;
				}
				currenttime = dwt_readsystimestamphi32();
				sprintf(debug, "sys time1: %08X, rx: %08X, diff: %08X", start_ts, dwt_readrxtimestamphi32(), dwt_readrxtimestamphi32() - start_ts);
				println(debug);
				sprintf(debug, "resp tx ts: %08X, diff: %08X, diff2: %08X", resp_tx, resp_tx - start_ts, dwt_readrxtimestamphi32() - start_ts);
				println(debug);
				sprintf(debug, "ct: %08X, diff: %08X", currenttime, resp_tx - currenttime);
				println(debug);
				break;

    		case WAIT_RESP_TX_CONF:
    			if (irq_status == IRQ_TX_OK) {
    				irq_status = IRQ_NONE;
    				dwt_setrxtimeout(delay_config.fwtoTime_sy);
					dwt_setdelayedtrxtime((get_rx_timestamp_u64() >> 8) + delay_config.pollTx2FinalTxDelay - delay_config.preambleDuration32h) ;

					if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) {
						println("can't reenable delayed");
						dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
						dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy*2); //reconfigure the timeout before enable
						dwt_rxenable(DWT_START_RX_IMMEDIATE);
						dwt_setpreambledetecttimeout(PRE_TIMEOUT); //configure preamble timeout
						dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy); //restore the timeout for next RX enable
					}
					state = WAIT_FINAL;
    			}
    			break;

    		case WAIT_FINAL:
    			if (irq_status != IRQ_NONE) {
    				read_final_msg();
    				irq_status = IRQ_NONE;
    				state = REENABLE_RX;
    			}
    			break;

    		case REENABLE_RX:
    			Sleep(10);
				state = WAIT_POLL;
				irq_status = IRQ_NONE;
    			rx_reenable_no_timeout();
    			break;
    	}
     }
}

static uint8 prepare_resp() {
	rx_buffer[ALL_MSG_SN_IDX] = 0;
	if (irq_status == IRQ_RX_OK) {
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
			uint32 resp_tx_time;
			poll_rx_ts = get_rx_timestamp_u64();
			resp_tx_time = dwt_readrxtimestamphi32() + delay_config.fixedReplyDelayAnc32h * (address + 1);
			resp_tx = resp_tx_time;
			start_ts = dwt_readsystimestamphi32();
			dwt_setdelayedtrxtime((resp_tx_time & 0xFFFFFFFE));
			tx_resp_msg[8] = address;
			*((float*) &tx_resp_msg[9]) = distance32;
			return SEND_RESP;
		}
	}
	irq_status = IRQ_NONE;
	rx_reenable_no_timeout();
	return WAIT_POLL;
}

static void read_final_msg() {
    double distance_to_correct;
	char debug[30];

    if (irq_status == IRQ_RX_OK) {
    	rx_buffer[ALL_MSG_SN_IDX] = 0;
    	if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
    		uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    		uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    		double Ra, Rb, Da, Db;
    		int64 tof_dtu;
    		resp_tx_ts = get_tx_timestamp_u64();
    		final_rx_ts = get_rx_timestamp_u64();

    		final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    		final_msg_get_ts(&rx_buffer[final_msg_resp_ts_id], &resp_rx_ts);
    		final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

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

            distance = distance - dwt_getrangebias(config.chan, (float) distance_to_correct, config.prf);

    		if ((distance > 0) && (distance < 2000)) {
    			distance32 = (float) distance;
    			count ++;
    			total += (distance);
    		}
    		if (rx_buffer[9] == 0x25) {
				sprintf(debug, "DIST: %2.3fm", (total / count));
				println(debug);

				if (ant_dly_fixed == 0) {
					if (total/count > 5.05) {
						ant_dly++;
					} else if (total/count < 4.95) {
						ant_dly--;
					} else {
						ant_dly_fixed = 1;
					}
					sprintf(debug, "antenna delay: %d %s", ant_dly, (ant_dly_fixed == 1 ? "FIX" : "..."));
					println(debug);
					dwt_setrxantennadelay(ant_dly);
					dwt_settxantennadelay(ant_dly);
				}
				count = 0;
				total = 0;
			}
    	}
    } else if (irq_status == IRQ_RX_TO) {
    	println("Final timeout");
    } else if (irq_status == IRQ_RX_ERR) {
    	println("Final err");
    }

}

#define TREK_ANTDLY_1  (0xD)
#define TREK_ANTDLY_2  (0xE)
#define TREK_ANTDLY_3  (0xF)
#define TREK_ANTDLY_4  (0x1D)

static void set_antenna_delays(dwt_config_t * config, int mode) {
	uint16 txAntennaDelay = 0;
	uint8 chanindex = 0;
	uint32 dly = 0;
	const uint16 rfDelaysTREK[2] = {
			(uint16) ((514.83f/ 2.0) * 1e-9 / DWT_TIME_UNITS),//channel 2
			(uint16) ((514.65f/ 2.0) * 1e-9 / DWT_TIME_UNITS) //channel 5
	};

    port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

	switch(config->chan)
	{
		case 2:
			if(config->dataRate == DWT_BR_6M8)
				dwt_otpread(TREK_ANTDLY_1, &dly, 1);
			else if(config->dataRate == DWT_BR_110K)
				dwt_otpread(TREK_ANTDLY_2, &dly, 1);
			break;
		case 5:
			if(config->dataRate == DWT_BR_6M8)
				dwt_otpread(TREK_ANTDLY_3, &dly, 1);
			else if(config->dataRate == DWT_BR_110K)
				dwt_otpread(TREK_ANTDLY_4, &dly, 1);
			break;
		default:
			dly = 0;
			break;
	}

	port_set_dw1000_fastrate(); //increase SPI to max

	// if nothing was actually programmed then set a reasonable value anyway
	if ((dly == 0)
			|| (dly == 0xffffffff))
	{
		if(config->chan == 5)
		{
			chanindex = 1;
		}

		txAntennaDelay = rfDelaysTREK[chanindex];
		println("nothing programed");
	}
	else
	{
		txAntennaDelay = (dly >> (16*(mode & 0x1))) & 0xFFFF;
		println("something programed");
	}

	char debug[50];
	sprintf(debug, "%d, %d", txAntennaDelay, txAntennaDelay);
	println(debug);

	dwt_setrxantennadelay(txAntennaDelay);
	dwt_settxantennadelay(txAntennaDelay);
}


static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}


static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}


static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

void rx_reenable_no_timeout () {
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_ok_anch(const dwt_cb_data_t *cb_data)
{
    int i;
    rx_timestamp = get_rx_timestamp_u64();
    irq_status = IRQ_RX_OK;
    for (i = 0 ; i < RX_BUF_LEN; i++ )
    {
        rx_buffer[i] = 0;
    }
    if (cb_data->datalength <= RX_BUF_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }
}

void rx_to_anch(const dwt_cb_data_t *cb_data)
{
    irq_status = IRQ_RX_TO;
}

void rx_err_anch(const dwt_cb_data_t *cb_data)
{
    irq_status = IRQ_RX_ERR;
}

void tx_conf_anch(const dwt_cb_data_t *cb_data)
{
    irq_status = IRQ_TX_OK;
}*/
