/*
 * initial_config.c
 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */

#include "deca_device_api.h"
#include "port.h"
#include "device.h"
#include "main.h"

static dwt_config_t config [4] = {
	{
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
	},
	{
		2,               // Channel number.
		DWT_PRF_16M,     // Pulse repetition frequency.
		DWT_PLEN_1024, //DWT_PLEN_128,   // Preamble length. Used in TX only.
		DWT_PAC16, //DWT_PAC8,       // Preamble acquisition chunk size. Used in RX only.
		3,               // TX preamble code. Used in TX only.
		3,               // RX preamble code. Used in RX only.
		0,               // 0 to use standard SFD, 1 to use non-standard SFD.
		DWT_BR_6M8,     // Data rate.
		DWT_PHRMODE_STD, // PHY header mode.
		(1024 + 1 + 8 - 16) //(128 + 1 + 8 - 8) // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
	}, {
		2,               // Channel number.
		DWT_PRF_64M,     // Pulse repetition frequency.
		DWT_PLEN_256, //DWT_PLEN_128,   // Preamble length. Used in TX only.
		DWT_PAC16, //DWT_PAC8,       // Preamble acquisition chunk size. Used in RX only.
		9,               // TX preamble code. Used in TX only.
		9,               // RX preamble code. Used in RX only.
		0,               // 0 to use standard SFD, 1 to use non-standard SFD.
		DWT_BR_6M8,     // Data rate.
		DWT_PHRMODE_STD, // PHY header mode.
		(256 + 1 + 8 - 16) //(128 + 1 + 8 - 8) // SFD timeout (preamble length + 1 + SFD length - PAC size)
	}, {
		2,               // Channel number.
		DWT_PRF_16M,     // Pulse repetition frequency.
		DWT_PLEN_128, //DWT_PLEN_128,   // Preamble length. Used in TX only.
		DWT_PAC8, //DWT_PAC8,       // Preamble acquisition chunk size. Used in RX only.
		4,               // TX preamble code. Used in TX only.
		4,               // RX preamble code. Used in RX only.
		0,               // 0 to use standard SFD, 1 to use non-standard SFD.
		DWT_BR_6M8,     // Data rate.
		DWT_PHRMODE_STD, // PHY header mode.
		(128 + 1 + 8 - 8) //(128 + 1 + 8 - 8) // SFD timeout (preamble length + 1 + SFD length - PAC size)
	}
};

static uint32 txPower[16] = { 0x00000000, 0x67676767, 0x67676767, 0x8B8B8B8B, 0x9A9A9A9A, 0x85858585, 0x00000000 ,0xD1D1D1D1,
							  0x00000000, 0x07274767, 0x07274767, 0x2B4B6B8B, 0x3A5A7A9A, 0x25456585, 0x00000000 ,0x5171B1D1};
static uint8 PGdelay[8] = { 0x00, 0xC9, 0xC2, 0xC5, 0x95, 0xC0, 0x00, 0x93 };

static dev_cfg_t dev_cfg[NB_CALIB] = {
		{
				0x11CC,
				16473
		},
		{
				0x11CB,
				16466
		},
		{
				0x11D5,
				16441
		},
		{
				0x11DE,
				16444
		},
		{
				0x1813,
				16429
		},
		{
				0x11B6,
				16439
		}
};

static dw_device_t device;
static dwt_txconfig_t txconfig;

//uint8 irq_status = IRQ_NONE;
uint8 dummy_buffer[DUMMY_BUFFER_LEN];

void init_config(int role, int dataRate, int init) {
	uint8 smart_power = 0;
	uint32 devID;
	char debug[30];
	irq_status = IRQ_NONE;
	device.role = (uint8) role;

	device.config = config[(dataRate == DWT_BR_110K) ? 0 : 1];

	if (dataRate == DWT_BR_6M8) {
		smart_power = 1;
	}

	port_DisableEXT_IRQ();

	port_set_dw1000_slowrate();

	devID = dwt_readdevid() ;
	if(DWT_DEVICE_ID != devID) {
		println("*******NEED WAKE UP*********");
		port_wakeup_dw1000();

		devID = dwt_readdevid() ;
		// SPI not working or Unsupported Device ID
		if(DWT_DEVICE_ID != devID){
			println("DWT_DEVICE_ID Failed");
			Sleep(5000);
			hardreset_DW1000();
			devID = dwt_readdevid() ;
			sprintf(debug, "%04X", devID);
			println(debug);
			//put_dev_to_sleep();

		}
		dwt_softreset();
	}

	//reset the DW1000 by driving the RSTn line low


	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		println("Initialization Failed, abandon");
		while (1)
		{ };
	}
	port_set_dw1000_fastrate();

	// Initialize main delays for the TWR
	set_delays(&device.config, &device.delay);
	dwt_configure(&device.config);

	// Configure the tx power
	txconfig.power = txPower[8 * smart_power + device.config.chan];
	txconfig.PGdly = PGdelay[device.config.chan];
	dwt_configuretxrf(&txconfig);

	dwt_setsmarttxpower(smart_power);

	// Configure the interruptions
	set_interrupt();
	//Sleep(2);
	port_EnableEXT_IRQ();
	//Sleep(2);
	set_local_config();
	if (init == 1) {
		sprintf(debug, " - ant delay: %d", device.ant_dly);
		println(debug);
		sprintf(debug, "********* [%04X] *********", device.add);
		println(debug);
	}
}

dw_device_t * get_device() {
	return &device;
}


uint64 convert_usec_to_devtimeu (double microsecu) {
	uint64 dt;
    long double dtime;
    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;
    dt =  (uint64) (dtime) ;
    return dt;
}

int convert_devtimeu_to_usec (uint64 devtimeu) {
	long double utime;
    utime = (int64) (devtimeu << 8) * (double) DWT_TIME_UNITS * 1e6;
    int ret = (int) utime + 1;
    return ret;
}

float calculate_length_data(float msgdatalen, uint8 dataRate) {
	int x = 0;
	x = (int) ceil(msgdatalen*8/330.0f);
	msgdatalen = msgdatalen*8 + x*48;

	//assume PHR length is 172308ns for 110k and 21539ns for 850k/6.81M
	if(dataRate == DWT_BR_110K)	{
		msgdatalen *= 8205.13f;
		msgdatalen += 172308; // PHR length in nanoseconds

	} else if(dataRate == DWT_BR_850K) {
		msgdatalen *= 1025.64f;
		msgdatalen += 21539; // PHR length in nanoseconds
	} else {
		msgdatalen *= 128.21f;
		msgdatalen += 21539; // PHR length in nanoseconds
	}
	return msgdatalen ;
}

void set_delays(dwt_config_t * config, delay_config_t * delay_config) {
    int margin = 3000; //2000 symbols
    int ifs;
    int respframe = 0, finalframe = 0, wuframe = 0, respframe_sy = 0, pollframe_sy = 0, finalframe_sy = 0, wuframe_sy = 0, sfdlen = 0;
	float msgdatalen_resp = 0, msgdatalen_poll = 0, msgdatalen_final = 0, msgdatalen_wu = 0, preamblelen = 0, msg_corpse = 0;

	msgdatalen_resp = calculate_length_data(RESP_MSG_LEN, config->dataRate);
	msgdatalen_poll = calculate_length_data(POLL_MSG_LEN, config->dataRate);
	msgdatalen_final = calculate_length_data(FINAL_MSG_LEN, config->dataRate);
	msgdatalen_wu = calculate_length_data(WU_MSG_LEN, config->dataRate);

	if (config->dataRate == DWT_BR_110K) {
		sfdlen = 64;
		ifs = 150;
	} else {
		sfdlen = 8;
		ifs = 91;
	}

	switch (config->txPreambLength) {
	    case DWT_PLEN_4096 : preamblelen = 4096.0f; break;
	    case DWT_PLEN_2048 : preamblelen = 2048.0f; break;
	    case DWT_PLEN_1536 : preamblelen = 1536.0f; break;
	    case DWT_PLEN_1024 : preamblelen = 1024.0f; break;
	    case DWT_PLEN_512  : preamblelen = 512.0f; break;
	    case DWT_PLEN_256  : preamblelen = 256.0f; break;
	    case DWT_PLEN_128  : preamblelen = 128.0f; break;
	    case DWT_PLEN_64   : preamblelen = 64.0f; break;
	}

	if(config->prf == DWT_PRF_16M) {
		preamblelen = (sfdlen + preamblelen) * 0.99359f;
		msg_corpse = sfdlen * 0.99359f;
	} else {
		preamblelen = (sfdlen + preamblelen) * 1.01763f;
		msg_corpse = sfdlen * 1.01763f;
	}

	respframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_resp + margin)/1000.0))/ 1.0256)) ;
	pollframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_poll + margin)/1000.0))/ 1.0256)) ;
	finalframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_final + margin)/1000.0))/ 1.0256)) ;
	wuframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_wu + margin)/1000.0))/ 1.0256));
	msg_corpse = msg_corpse + (int)((msgdatalen_wu / 1000) / 1.0256);

	respframe = (int)(preamblelen + (msgdatalen_resp/1000.0)); //length of response frame (micro seconds)
	finalframe = (int)(preamblelen + (msgdatalen_final/1000.0));
	wuframe = (int)(preamblelen + (msgdatalen_wu / 1000.0));

	if(config->dataRate == DWT_BR_110K)	{
		delay_config->fwtoTime_sy = finalframe_sy + RX_RESPONSE_TURNAROUND + 400; //add some margin because of the resp to resp RX turn on time
		delay_config->preambleDuration32h = (uint32) (((uint64) convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY; //preamble duration + 16 us for RX on
		msg_corpse += 172.308;
	} else {
		delay_config->fwtoTime_sy = finalframe_sy + RX_RESPONSE_TURNAROUND; //add some margin because of the resp to resp RX turn on time
		delay_config->preambleDuration32h = (uint32) (((uint64) convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY; //preamble duration + 16 us for RX on
		msg_corpse += 21.539;
	}
	delay_config->tagRespRxDelay_sy = RX_RESPONSE_TURNAROUND + respframe_sy - pollframe_sy;
	delay_config->fixedReplyDelayAnc32h = ((uint64)convert_usec_to_devtimeu (respframe + RX_RESPONSE_TURNAROUND) >> 8);


	delay_config->pollTx2FinalTxDelay = ((uint64)convert_usec_to_devtimeu (4 * (respframe + RX_RESPONSE_TURNAROUND) + finalframe + RX_RESPONSE_TURNAROUND * 2) >> 8);
	delay_config->fwto4RespFrame_sy = respframe_sy;
	delay_config->wuFrameTime_sy = wuframe_sy + ifs;
	delay_config->wuFrame_nb = ((LONG_SLEEP_TIME_MS + 2500) * 1e3) / delay_config->wuFrameTime_sy + 1;
	delay_config->respFrame = respframe;
	char debug[50];
	/*sprintf(debug, "msg corpse: %f", msg_corpse);
	println(debug);
	msg_corpse = (((uint64) convert_usec_to_devtimeu (msg_corpse)) >> 8);
	sprintf(debug, "tagRespRx : %d", delay_config->tagRespRxDelay_sy);
	println(debug);
	sprintf(debug, "replyDelay: %d", respframe + RX_RESPONSE_TURNAROUND);
	println(debug);
	sprintf(debug, "poll2final: %d", 4 * (respframe + RX_RESPONSE_TURNAROUND) + finalframe + RX_RESPONSE_TURNAROUND);
	println(debug);
	sprintf(debug, "fwto4Resp : %d", respframe_sy);
	println(debug);
	sprintf(debug, "preamble  : %f", preamblelen);
	println(debug);
	sprintf(debug, "respFrame : %d", respframe);
	println(debug);
	sprintf(debug, "finalFrame: %d", finalframe);
	println(debug);
	sprintf(debug, "fwtoTime : %d", delay_config->fwtoTime_sy);
	println(debug);
	sprintf(debug, "wuFrameTime : %d", wuframe_sy + ifs);
	println(debug);*/
	//sprintf(debug, "replyDelA: %d");
}

void calibrate_antenna_delay(double dist) {
	double threshold = 1.04;
	uint8 change_ant_dly = 0;
	char debug[30];
	if (dist < threshold - 0.10) {
		change_ant_dly = 1;
		device.ant_dly -= 10;
		sprintf(debug, " --- ant delay: %d", device.ant_dly);
	} else if (threshold + 0.10 < dist) {
		change_ant_dly = 1;
		device.ant_dly += 10;
		sprintf(debug, " +++ ant delay: %d", device.ant_dly);
	} else if (dist < threshold - 0.05) {
		change_ant_dly = 1;
		device.ant_dly -= 5;
		sprintf(debug, " -- ant delay: %d", device.ant_dly);
	} else if (threshold + 0.05 < dist) {
		change_ant_dly = 1;
		device.ant_dly += 5;
		sprintf(debug, " ++ ant delay: %d", device.ant_dly);
	} else if (dist < threshold - 0.02) {
		change_ant_dly = 1;
		device.ant_dly -= 1;
		sprintf(debug, " - ant delay: %d", device.ant_dly);
	} else if (threshold + 0.02 < dist) {
		change_ant_dly = 1;
		device.ant_dly += 1;
		sprintf(debug, " + ant delay: %d", device.ant_dly);
	}

	if (change_ant_dly == 1) {
		println("");
		print("###");
		dwt_setrxantennadelay(device.ant_dly);
		dwt_settxantennadelay(device.ant_dly);
		println(debug);
	}
}

void set_ranging_exchange_config() {
	dwt_forcetrxoff();
	dwt_rxreset();
	if (device.config.dataRate == DWT_BR_6M8) {
		if (device.config.txPreambLength != config[2].txPreambLength) {
			device.config = config[2];
		}
	}
	set_delays(&device.config, &device.delay);
	dwt_configure(&device.config);

}

void set_lowpowerlistening_config() {
	dwt_forcetrxoff();
	dwt_rxreset();
	if (device.config.dataRate == DWT_BR_6M8) {
		if (device.config.txPreambLength != config[1].txPreambLength) {
			device.config = config[1];
		}
	}
	set_delays(&device.config, &device.delay);
	dwt_configure(&device.config);
}

void set_interrupt() {
	if (device.role == TAG) {
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
		dwt_setcallbacks(&tx_conf_tag, &rx_ok_tag, &rx_to_tag, &rx_err_tag);
	} else {
#if LPL_MODE
		dwt_setinterrupt(DWT_INT_RFCG, 1);
#else
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
#endif
		dwt_setcallbacks(&tx_conf_anch, &rx_ok_anch, &rx_to_anch, &rx_err_anch);
	}
}

void set_local_config () {
	uint16 txAntennaDelay = 0;
	uint8 chanindex = 0;
	uint32 dly = 0, devId = 0;
	uint16 rfDelays[2] = {
			(uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
			(uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)
	};

    port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

	switch(device.config.chan) {
		case 2:
			if(device.config.dataRate == DWT_BR_6M8)
				dwt_otpread(TREK_ANTDLY_1, &dly, 1);
			else if(device.config.dataRate == DWT_BR_110K)
				dwt_otpread(TREK_ANTDLY_2, &dly, 1);
			break;
		case 5:
			if(device.config.dataRate == DWT_BR_6M8)
				dwt_otpread(TREK_ANTDLY_3, &dly, 1);
			else if(device.config.dataRate == DWT_BR_110K)
				dwt_otpread(TREK_ANTDLY_4, &dly, 1);
			break;
		default:
			dly = 0;
			break;
	}

	port_set_dw1000_fastrate(); //increase SPI to max

	// if nothing was actually programmed then set a reasonable value anyway
	if ((dly == 0) || (dly == 0xffffffff)) {
		if(device.config.chan == 5)	{
			chanindex = 1;
		}
		txAntennaDelay = rfDelays[chanindex];
	} else {
		println("Antenna calibrated by DECAWAVE");
		txAntennaDelay = (dly >> (16*(device.role & 0x1))) & 0xFFFF;
	}
    dwt_geteui(device.eui64);
    char debug[30];
    devId = _dwt_otpread(PARTID_ADDRESS);
    device.add = devId & 0xFFFF;
    if (device.role == ANCHOR) {
        //device.add = device.add & 0xFFFC;
        //device.add = device.add | TEST_ADD;
    }
    device.id = device.add & 0x03;
    for (int i = 0; i < NB_CALIB; i++) {
    	if ((devId & 0xFFFF) == dev_cfg[i].devId) {
    		device.ant_dly = dev_cfg[i].ant_dly;
    	}
    }
	if (device.ant_dly == 0) {
		device.ant_dly = txAntennaDelay;
	}
	dwt_setrxantennadelay(device.ant_dly);
	dwt_settxantennadelay(device.ant_dly);
}


void put_dev_to_sleep() {
	uint32 lp_osc_freq, sleep_cnt;
	set_lowpowerlistening_config();
	dwt_forcetrxoff();
	dwt_rxreset();
	port_DisableEXT_IRQ();
	port_set_dw1000_slowrate();
	lp_osc_freq = (XTAL_FREQ_HZ / 2) / dwt_calibratesleepcnt();
	sleep_cnt = ((1500 / 2 * lp_osc_freq) / 1000) >> 12;
	dwt_configuresleepcnt(sleep_cnt);
	port_set_dw1000_fastrate();
	dwt_setinterrupt(0xFFFFFFFF, 0);
	Sleep(5);
	dwt_setinterrupt(DWT_INT_RFCG, 1);
    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_RX_EN, DWT_WAKE_SLPCNT | DWT_SLP_EN);
    port_EnableEXT_IRQ();
	dwt_setsnoozetime(LPL_SHORT_SLEEP_SNOOZE_TIME);
	dwt_setpreambledetecttimeout(LPL_RX_SNIFF_TIME);
	dwt_setlowpowerlistening(1);
	lpl_status = ASLEEP;
	dwt_entersleep();

	uint32 devID = dwt_readdevid();
	if (devID == DWT_DEVICE_ID) {
		println("!!!!!! SLEEP FAILED !!!!!!");
	} else {
		//println("[STM32] Put to sleep\r\n");

		HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_0);
		HAL_PWREx_EnablePullUpPullDownConfig();

		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);

		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);

		HAL_PWR_EnterSTANDBYMode();
	}
}

void hardreset_DW1000() {
	HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

	usleep(1);

	HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
}

uint64 get_rx_timestamp_u64(void) {
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

uint64 get_tx_timestamp_u64(void) {
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

double get_systime_ms() {
	uint32 ts = dwt_readsystimestamphi32() >> 1;
	double coef = 512/(499.2e6*128);
	return (double) (coef * ts) * 1e3;
}

double get_systime_us() {
	uint32 ts = dwt_readsystimestamphi32() >> 1;
	double coef = 512/(499.2e6*128);
	return (double) (coef * ts) * 1e6;
}

double get_systime_s() {
	uint32 ts = dwt_readsystimestamphi32() >> 1;
	double coef = 512/(499.2e6*128);
	return (double) (coef * ts);
}
