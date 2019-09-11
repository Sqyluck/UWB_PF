/*
 * LoRadriver.h
 *
 *  Created on: 29 juil. 2019
 *      Author: Noolitic
 */

#ifndef APP_LORADRIVER_LORADRIVER_H_
#define APP_LORADRIVER_LORADRIVER_H_

#define MAX_LENGTH 128
#define MAX_CFG_LENGTH 128
#define RESP_CHECK_LEN 5
#define NB_CONFIG 14
#define LORA_TIMEOUT 20
#define   DEVADDR_ABP  "AA000002"

#define   NWK_SK       "0123456789ABCDEF0123456789ABCDEF"
#define   APP_SK       "ABCDEF0123456789ABCDEF0123456789"
//#define   NWK_SK       "2B7E151628AED2A6ABF7158809CF4F3C"
//#define   APP_SK       "2B7E151628AED2A6ABF7158809CF4F3C"

#define AT		 0x01
#define CLASS	 0x02
#define PGM		 0x03
#define WAIT_ACK 0x04

#define LW_NONE    0x00
#define LW_RX      0x01
#define LW_START   0x02
#define LW_ACK     0x03
#define LW_DONE    0x04
#define LW_BUSY    0x05
#define LW_NO_CHAN 0x06

uint8_t lora_status;


typedef struct {
	char data[MAX_LENGTH];
	int len;
} uart_data;

typedef struct {
	uint8_t exp_resp[RESP_CHECK_LEN];
	int resp_len;
	uint8_t at_cmd[MAX_CFG_LENGTH];
	int len;
}at_config;

uart_data lora_data;
//at_config lora_config[NB_CONFIG];
int serverOk;
int phase;

void clear_uart_buf();
void lora_usart_callback(unsigned char c);
void send_str_lora(char * cmd);
void enable_loraIRQ();
void disable_loraIRQ();
int wait_lora_status(int status, int timeout);
int isLoraConnected();
int send_cmsghex_lora(unsigned char * hex, int size);
int getByteFromStrResp(char * str);
void send_cmd_lora(char * cmd, int size);
void clear_lora_pending_message();

#endif /* APP_LORADRIVER_LORADRIVER_H_ */
