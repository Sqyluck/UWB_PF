/*
 * LoRadriver.c
 *
 *  Created on: 29 juil. 2019
 *      Author: Noolitic
 */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include <string.h>
#include "main.h"
#include "LoRadriver.h"
#include "device.h"
#include "port.h"

#define DEBUG 3

extern UART_HandleTypeDef huart1;

uart_data lora_data;
at_config lora_config[NB_CONFIG];
int serverOk = 0;
uint8_t lora_status = LW_NONE;
int answer_ok = 0;
static int cfg_id = 0;
int phase = AT;
static int IRQ_enabled;

void lora_usart_callback(unsigned char c) {
	char debug[30];
#if DEBUG == 3
	sprintf(debug, "%c", c);
	print(debug);
#endif
	lora_data.data[lora_data.len++] = c;
	if ((lora_data.len > MAX_LENGTH) ||
		((lora_data.data[lora_data.len - 2] == '\r') &&
		 (lora_data.data[lora_data.len - 1] == '\n'))) {
		clear_uart_buf();
	}
	if (phase == AT) {
		if ( (lora_data.data[lora_data.len - 2] == 'O') &&
			 (lora_data.data[lora_data.len - 1] == 'K') ) {
			answer_ok = 1;
			clear_uart_buf();
			println("[LORA]---AT OK");
		}

	} else if (phase == CLASS) {
		if( (lora_data.data[lora_data.len-4] == 'C') &&
			(lora_data.data[lora_data.len-3] == 'L') &&
			(lora_data.data[lora_data.len-2] == 'A') &&
			(lora_data.data[lora_data.len-1] == 'S') ) {
			answer_ok = 1;
			println("[LORA]---Answer Class OK");
			clear_uart_buf();
		}

	} else if (phase == PGM) {
		if ( (lora_data.data[lora_data.len-5] == 'S') &&
			(lora_data.data[lora_data.len-4] == 't') &&
			(lora_data.data[lora_data.len-3] == 'a') &&
			(lora_data.data[lora_data.len-2] == 'r') &&
			(lora_data.data[lora_data.len-1] == 't')) {
#if DEBUG == 2
				println("\r\n**START**");
#endif
				lora_status = LW_START;

		} else 	if ( (lora_data.data[lora_data.len-7] == 'R') &&
			(lora_data.data[lora_data.len-6] == 'X') &&
			(lora_data.data[lora_data.len-5] == ':') ) {
#if DEBUG == 2
			println("***RX***");
#endif
			lora_status = LW_RX;
			serverOk = getByteFromStrResp(lora_data.data + lora_data.len - 2);

		} else if ( (lora_data.data[lora_data.len-6] == 'A') &&
				(lora_data.data[lora_data.len-5] == 'C') &&
				(lora_data.data[lora_data.len-4] == 'K') &&
				(lora_data.data[lora_data.len-2] == 'R') &&
				(lora_data.data[lora_data.len-1] == 'e') ) {
#if DEBUG == 2
			println("***ACK***");
#endif
			lora_status = LW_ACK;

		} else if ( (lora_data.data[lora_data.len-4] == 'D') &&
				(lora_data.data[lora_data.len-3] == 'o') &&
				(lora_data.data[lora_data.len-2] == 'n') &&
				(lora_data.data[lora_data.len-1] == 'e') ) {
#if DEBUG == 2
			println("****DONE****\r\n");
#endif
			lora_status = LW_DONE;

		} else if ( (lora_data.data[lora_data.len-4] == 'b') &&
				(lora_data.data[lora_data.len-3] == 'u') &&
				(lora_data.data[lora_data.len-2] == 's') &&
				(lora_data.data[lora_data.len-1] == 'y') ) {
#ifdef DEBUG
			println("**BUSY**");
#endif
			lora_status = LW_BUSY;

		} else if ( (lora_data.data[lora_data.len-5] == 'N') &&
				(lora_data.data[lora_data.len-4] == 'o') &&
				(lora_data.data[lora_data.len-2] == 'f') &&
				(lora_data.data[lora_data.len-1] == 'r') ) {
#ifdef DEBUG
			println("***NOCHANN***");
#endif
			lora_status = LW_NO_CHAN;

		}
	} else {
		sprintf(debug, "%c", c);
		print(debug);
	}
}

void clear_uart_buf() {
    memset(lora_data.data, '\0', sizeof(char) * MAX_LENGTH);
    lora_data.len = 0;
}

int check_resp(uint8_t * resp) {
	return (memcmp(&lora_config[cfg_id].exp_resp, resp, lora_config[cfg_id].resp_len) == 0 ? 1 : 0);
}

void send_cmd_lora(char * cmd, int size) {
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, size, 0xFFFF);
}

int isDistFields(int field_id) {
	for (int i = 0; i < 4; i++) {
		if ((field_id >= 3 + 2 + 8 * i) && (field_id < 3 + 8 * (i + 1))) {
			return 1;
		}
	}
	return 0;
}

int getByteFromStrResp(char* str) {
	uint8_t res = 0x00;
	char c;
	for (int i = 0; i < 2; i++) {
		c = str[i];
		if ('0' <= c && c <= '9'){
			c = (c - '0');
		}
		if ('A' <= c && c <= 'F') {
			c = (c - 'A' + 10);
		}
		if ('a' <= c && c <= 'f') {
			c = (c - 'a' + 10);
		}
		res = res | (c);
		res = res << (4 * (1 - i));
	}
	return res;
}

int isLoraConnected() {
	phase = AT;
	int nb_try = 0;
	answer_ok = 0;
	while ( (!answer_ok) && (nb_try < 5) ) {
		send_str_lora("AT\r\n");
		HAL_Delay(500);
		nb_try++;
	}
	return answer_ok;
}

int send_cmsghex_lora(unsigned char * hex, int size) {
	char msghex[60] = "";
	char debug[30];
	int currentsize = 10;
	if (IRQ_enabled == 0) {
		enable_loraIRQ();
		println("need to enable IRQ");
	}

	sprintf(msghex, "AT+MSGHEX=");
	for (int i = 0; i < size; i++) {
		if (size == 35) {
			if (isDistFields(i)) {
				sprintf(msghex + currentsize, "%c", hex[i]);
				currentsize++;
			} else {
				sprintf(msghex + currentsize, "%02X", hex[i]);
				currentsize+=2;
			}
		} else {
			sprintf(msghex + currentsize, "%02X", hex[i]);
			currentsize+=2;
		}
		 // interval
	}
	int final_len = strlen(msghex);
	msghex[strlen(msghex)] = '\r';
	msghex[strlen(msghex)] = '\n';
	HAL_UART_Transmit(&huart1, (uint8_t*)msghex, strlen(msghex), 0xFFFF);
	serverOk = 0;
	int ack = 0;
	lora_status = LW_NONE;
	int c = 0;
	while (lora_status == LW_NONE){
		HAL_Delay(1);
	}
	if ((lora_status == LW_NO_CHAN) || (lora_status == LW_BUSY)) {
		Sleep(1000);
		return 0;
	} else if (lora_status == LW_START){
		return 1;
	} else {
		return -1;
	}
}

int wait_lora_status(int status, int timeout) {
	int start_ts;
	start_ts = portGetTickCnt();
	while (lora_status != status) {
		if (start_ts + timeout < portGetTickCnt()) {
			return 0;
		}
	}
	if (lora_status == status) {
		lora_status = LW_NONE;
		return 1;
	} else {
		println("TIMEOUT, EXPECTED STATUS NOT RECEIVED");
		return 0;
	}

	/*char debug[30];
	double sending_time, current_time, previous_time = 0, global_time, save_time = 0;
	current_time = get_systime_s();
	sending_time = current_time;
	global_time = current_time;
	int counter = 0, show_timer = 0;

	while (lora_status != status) {
		current_time = get_systime_s();
		if (current_time < previous_time) {
			double diff = previous_time + current_time;
			save_time += diff;
		} else {
			global_time= save_time + current_time;
			if (show_timer == 1) {
				if (counter % 10000 == 0) {
					sprintf(debug, "t: %.2fs", global_time);
					println(debug);
				}
				counter++;
			}
		}
		if (timeout != 0) {
			if (sending_time < global_time - timeout) {
				return 0;
			}
		}
		previous_time = current_time;
	}
	if (lora_status == status) {
		return 1;
	} else {
		println("TIMEOUT, EXPECTED STATUS NOT RECEIVED");
		return 0;
	}*/
}

void send_str_lora(char * cmd) {
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 0xFFFF);
}

void clear_lora_pending_message() {
	int leave = 0;
	while (leave == 0) {
		leave = 1;
		println("AT+MSG");
		send_str_lora("AT+MSG\r\n");
		while(lora_status != LW_DONE) {
			if (lora_status == LW_RX) {
				println("- Clear Rx pending");
				lora_status = LW_NONE;
				leave = 0;
			}
		}
		println("Done");
	}
}

void init_at_command() {
	dw_device_t * dev = get_device();

	int cfg = 0;
	char debug[128];
	char ws_addr[8];
	sprintf(ws_addr, "AA00%04X", dev->add);
	for (cfg = 0; cfg < NB_CONFIG; cfg++) {
		memset(lora_config[cfg].at_cmd, '\0', sizeof(char) * MAX_CFG_LENGTH);
		lora_config[cfg].len = 0;
	}
	cfg = 0;
	sprintf((char *) lora_config[cfg++].at_cmd, "AT\r\n");
	//sprintf((char *) lora_config[cfg++].at_cmd, "AT+VER\r\n");
	//sprintf((char *) lora_config[cfg++].at_cmd, "AT+ID\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+DR=EU868\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+DR=SCHEME\r\n");
	//sprintf((char *) lora_config[cfg++].at_cmd, "AT+CH\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+ID=DevAddr,\"%s\"\r\n", ws_addr);
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+KEY=NWKSKEY,\"%s\"\r\n", NWK_SK);
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+KEY=APPSKEY,\"%s\"\r\n", APP_SK);
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+MODE=LWABP\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+CLASS=A\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+UART=TIMEOUT, 1000\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+LW=DC\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+LW=SCR, OFF\r\n");
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+LW=MC,ON,\"ABBA0080\",\"%s\",\"%s\",0\r\n",NWK_SK,APP_SK);
	sprintf((char *) lora_config[cfg++].at_cmd, "AT+RETRY=1\r\n");
	sprintf((char *) lora_config[cfg].at_cmd, "AT+DELAY=RX1, 1000\r\n");

	for (cfg = 0; cfg < NB_CONFIG; cfg++) {
		lora_config[cfg].len = strlen((char *) lora_config[cfg].at_cmd);
	}
}

void enable_loraIRQ() {
	//HAL_NVIC_DisableIRQ(USART1_IRQn);
	//huart1->RxState = HAL_UART_STATE_READY;
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	IRQ_enabled = 1;
	//huart1.Instance->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE;
}

void disable_loraIRQ() {
	IRQ_enabled = 0;
	HAL_NVIC_DisableIRQ(USART1_IRQn);
}

int init_lora() {
	char debug[30];
	println("init lora");
	int delay = 500;
	int nb_try;
	enable_loraIRQ();
	init_at_command();
	while (cfg_id < NB_CONFIG) {
		sprintf(debug, "CONF: [%d/%d]", cfg_id + 1, NB_CONFIG);
		println(debug);
		if ( (cfg_id == 0) || (cfg_id == 7) ) {
			phase = (cfg_id == 0 ? AT : CLASS);
			nb_try = 0;
			answer_ok = 0;
			while ( (!answer_ok) && (nb_try < 10) ) {
				send_cmd_lora(lora_config[cfg_id].at_cmd, lora_config[cfg_id].len);
				HAL_Delay(delay);
				nb_try++;
			}
			if (answer_ok) {
				cfg_id++;
				answer_ok = 0;
			} else {
				println("failed");
				return -1;
			}
		} else {
			send_cmd_lora(lora_config[cfg_id].at_cmd, lora_config[cfg_id].len);
			cfg_id++;
			HAL_Delay(delay);
		}
	}
	//send_str_lora("AT+DELAY\r\n");
	//Sleep(5000);
	clear_uart_buf();

	phase = PGM;
	println("[LORA]---Init finished");
	return 1;
}

