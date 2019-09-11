#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"
#include "device.h"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
/*
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

#define NB_SAMPLES 32

typedef struct {
	float dist[NB_SAMPLES];
	uint16 address;
} anch_dist_t;

static anch_dist_t anch_dist[4];

#define TX_ANT_DLY 16472
#define RX_ANT_DLY 16472

#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  // The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

static uint8 tx_poll_msg[] = {0x01, 0x02, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x03, 0x04, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x05, 0x06, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2
#define RESP_LAST_DIST_VAL 11
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_ID0 14
#define FINAL_MSG_RESP_RX_TS_ID1 18
#define FINAL_MSG_RESP_RX_TS_ID2 22
#define FINAL_MSG_RESP_RX_TS_ID3 26
#define FINAL_MSG_FINAL_TX_TS_IDX 30
#define FINAL_MSG_TS_LEN 4

static uint8 frame_seq_nb = 0;
static int nb_poll = 0;

#define RX_BUF_LEN 20
//static uint8 rx_buffer[RX_BUF_LEN];

static uint32 status_reg = 0;

#define UUS_TO_DWT_TIME 65536

#define POLL_TX_TO_RESP_RX_DLY_UUS  500   // 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100  // 3100
#define RESP_RX_TIMEOUT_UUS         3000  // 2700
#define PRE_TIMEOUT 8

#define SEND_POLL          0x01
#define READ_RESP          0x02
#define SEND_FINAL         0x03
#define WAIT_NEXT_RANGING  0x04
#define CHECK_RESULT       0x05
#define WAIT_POLL_TX_CONF  0x06
#define WAIT_FINAL_TX_CONF 0x07

#define IRQ_TX_OK  0x01
#define IRQ_RX_OK  0x02
#define IRQ_RX_TO  0x03
#define IRQ_RX_ERR 0x04
#define IRQ_NONE   0x05

static uint32 txPower[16] = { 0x00000000, 0x67676767, 0x67676767, 0x8B8B8B8B, 0x9A9A9A9A, 0x85858585, 0x00000000 ,0xD1D1D1D1,
							  0x00000000, 0x07274767, 0x07274767, 0x2B4B6B8B, 0x3A5A7A9A, 0x25456585, 0x00000000 ,0x5171B1D1};
static uint8 PGdelay[8] = { 0x00, 0xC9, 0xC2, 0xC5, 0x95, 0xC0, 0x00, 0x93 };
static dwt_txconfig_t txconfig;

typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static uint32 final_tx_time;
static uint8 state = SEND_POLL;
static uint8 ranging_anch = 0;
static uint8 sample = 0;
// static uint8 irq_status = IRQ_NONE;

void init_and_send_poll();
static uint8 read_response();
void enable_rx(uint32 dlyTime);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
float get_previous_dist(uint8 * dist_field);
void print_result();
//float get_average_distance(int anch_id);
//static void set_antenna_delays(dwt_config_t * config, int mode);

static void rx_ok_tag(const dwt_cb_data_t *cb_data);
static void rx_to_tag(const dwt_cb_data_t *cb_data);
static void rx_err_tag(const dwt_cb_data_t *cb_data);
static void tx_conf_tag(const dwt_cb_data_t *cb_data);

int main_ds_twr_tag(void)
{
	int ret;
    uint8 smartPower = 0;

    port_DisableEXT_IRQ();
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

    dwt_setcallbacks(&tx_conf_tag, &rx_ok_tag, &rx_to_tag, &rx_err_tag);

    port_EnableEXT_IRQ();
    //dwt_setrxantennadelay(RX_ANT_DLY);
    //dwt_settxantennadelay(TX_ANT_DLY);

    set_antenna_delays(&config, 0);

    //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

	dwt_setrxaftertxdelay((uint32)delay_config.tagRespRxDelay_sy);
	dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy);

    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	uint64 tagCalculatedFinalTxTime;
	uint64 poll_tx_time;

	char debug[30];
    while (1)
    {
    	switch (state) {
			case SEND_POLL:
				init_and_send_poll();
				state = WAIT_POLL_TX_CONF;
				break;

			case WAIT_POLL_TX_CONF:
				if (irq_status != IRQ_TX_OK) {
					break;
				} else {
					final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
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
				poll_tx_time = (get_tx_timestamp_u64() >> 8);
				final_tx_time = poll_tx_time + delay_config.pollTx2FinalTxDelay;
				dwt_setdelayedtrxtime(final_tx_time);
				sample++;


				final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
				tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				if (sample == NB_SAMPLES) {
					tx_final_msg[9] = 0x25;
				}
				dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
				dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

				ret = dwt_starttx(DWT_START_TX_DELAYED);

				if (ret == DWT_SUCCESS)
				{
					state = WAIT_FINAL_TX_CONF;
					frame_seq_nb++;
				} else {
					println("can't send");
					state = WAIT_NEXT_RANGING;
				}
				break;

			case WAIT_FINAL_TX_CONF:
				if (irq_status != IRQ_TX_OK) {
					break;
				} else {
					//println("Final send");
					irq_status = IRQ_NONE;
					state = CHECK_RESULT;
				}
				break;

			case CHECK_RESULT:
				if (sample == NB_SAMPLES) {
					sample = 0;
					state = WAIT_NEXT_RANGING;
				} else {
					Sleep(20);
					state = SEND_POLL;
				}
				break;

			case WAIT_NEXT_RANGING:
				tx_final_msg[9] = 0x23;
				state = SEND_POLL;
				print_result();
				Sleep(1000);
				break;
			}
    }
}

void init_and_send_poll() {
	poll_tx_ts = 0;
	final_tx_time = 0;
	tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
	dwt_setrxaftertxdelay((uint32)delay_config.tagRespRxDelay_sy);
	dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT);

	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}


static uint8 read_response() {
	char debug[50];
	uint16 anch_address;
	uint8 final_msg_resp_pos, anch_id;
	frame_seq_nb++;

	if (irq_status == IRQ_RX_OK) {
		//println("rx");
		anch_address = (rx_buffer[7] << 8 | rx_buffer[8]);
		anch_id = anch_address & 0x03;

		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
			resp_rx_ts = get_rx_timestamp_u64();
			final_msg_resp_pos = 14 + 4 * anch_id;
			final_msg_set_ts(&tx_final_msg[final_msg_resp_pos], resp_rx_ts);
			anch_dist[anch_id].dist[sample] = get_previous_dist(&rx_buffer[9]);
			anch_dist[anch_id].address = anch_address;
		}
	} else {
		//println("--");
	}
	irq_status = IRQ_NONE;

	if (ranging_anch == 3) {
		ranging_anch = 0;
		return SEND_FINAL;
	} else {
		ranging_anch++;
		uint32 next_rx_time;
		uint64 start_ranging_time;
		next_rx_time = (get_tx_timestamp_u64() >> 8) + (ranging_anch + 1) * delay_config.fixedReplyDelayAnc32h;
		enable_rx(next_rx_time);
		return READ_RESP;
	}
}
/*
void enable_rx(uint32 dlyTime)
{
	dwt_setdelayedtrxtime(dlyTime - delay_config.preambleDuration32h) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
	{
		dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
		dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy*2); //reconfigure the timeout before enable
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setpreambledetecttimeout(PRE_TIMEOUT); //configure preamble timeout
		dwt_setrxtimeout((uint16)delay_config.fwto4RespFrame_sy); //restore the timeout for next RX enable
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

void print_result() {
	char debug[50];
	println("--- RANGING EXCHANGE --- ");
	for (int i = 0; i < 4; i++) {
		sprintf(debug, "anch[%d] : %2.3fm", anch_dist[i].address, get_average_distance(i));
		println(debug);
		anch_dist[i].address = 0;
	}
	println("");
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


static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
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

void rx_to_tag(const dwt_cb_data_t *cb_data)
{
    irq_status = IRQ_RX_TO;
    //println("rx_to_cb");
}

void rx_err_tag(const dwt_cb_data_t *cb_data)
{
    irq_status = IRQ_RX_ERR;
    println("rx_err_cb");
}

void tx_conf_tag(const dwt_cb_data_t *cb_data)
{
	//println("tx_conf_cb");
    irq_status = IRQ_TX_OK;
    poll_tx_ts = get_tx_timestamp_u64();
	//poll_tx_ts = portGetTickCnt() - TX_ANT_DLY;
}
*/
