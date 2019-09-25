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

#define DEBUG 1

extern UART_HandleTypeDef huart1;

uart_data lora_data;
at_config lora_config[NB_CONFIG];
int serverOk = 0;
uint8_t lora_status = LW_NONE;
int answer_ok = 0;
static int cfg_id = 0;
int phase = AT;
static int IRQ_enabled;


// Uart callback from lora device
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


void send_str_lora(char * cmd) {
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 0xFFFF);
}

void send_cmd_lora(char * cmd, int size) {
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, size, 0xFFFF);
}

/* @fn isDistFields
 * @brief	Check if the given id must represent a distance (num) or an id (char)
 * 			msg : CCCC02CCCCddddddddddddCCCCddddddddddddCCCCddddddddddddCCCCdddddddddddd
 * 			where (CC) form a byte but (dd) form a number (0x30 to 0x39)
 * 			look at send_cmsghex_lora function for more details
 * @param	int field_id 	- id
 *
 * @return	1 if it represent a distance field
 * 			0 if not
 * */
int isDistFields(int field_id) {
	for (int i = 0; i < 4; i++) {
		if ((field_id >= 3 + 2 + 8 * i) && (field_id < 3 + 8 * (i + 1))) {
			return 1;
		}
	}
	return 0;
}

/* @fn 		getByteFromStrResp
 * @brief 	return a string representing byte as byte (ex: "AB" => 0xab)
 * @param	char* str	- string representing the byte
 *
 * @return	uint8 	    - can not do it now, wait for release
 * */
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

/* @fn 		isLoraConnected
 * @brief 	Check if a lora device is connected with an AT command
 *
 * @return	int 	- 1 if connected, else 0
 * */
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

/* @fn send_cmsghex_lora
 * @brief Send msghex to lora
 * @param	char* hex	- hex data
 * @param	int size	- data size
 *
 * @return	  1  if communication starts
 *			  0  if no free channel or system busy
 *  	 	 -1  unknown error
 * */
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
			if (isDistFields(i)) { // if the byte represent a number, copy the char representing it
				sprintf(msghex + currentsize, "%c", hex[i]);
				currentsize++;
			} else { // else copy the hexadecimal value
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

/* @fn wait_lora_status
 * @brief Wait until status is found or timeout expired
 * @param int status 	- status
 * @param int timeout 	- timeout in milliseconds
 *
 * @return	1  if status found
 * 			0  if timeout expired
 * */
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
}

/* @fn clear_lora_pending_message
 * @brief Send empty message to clear the lorawan server queue in class A
 * */
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

/* @fn init_at_command
 * @brief prepare the lora_config array with all the AT command needed
 *
 * */
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

/* @fn enable_loraIRQ
 * @brief enable lora IRQ
 * */
void enable_loraIRQ() {
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	IRQ_enabled = 1;
}

/* @fn disable_loraIRQ
 * @brief disable lora IRQ
 * */
void disable_loraIRQ() {
	IRQ_enabled = 0;
	HAL_NVIC_DisableIRQ(USART1_IRQn);
}

/* @fn init_lora
 * @brief Send AT command to init the lora device
 *
 * @return 	1 if the initialization is successful
 * 			0 if some command are rejected
 * */
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
	clear_uart_buf();

	phase = PGM;
	println("[LORA]---Init finished");
	return 1;
}

