/*
 * main_ds_twr.c

 *
 *  Created on: 16 juil. 2019
 *      Author: Noolitic
 */
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "device.h"
#include "anch_dev.h"
#include "tag_dev.h"
#include "main.h"
#include "LoRadriver/LoRadriver.h"


int main_ds_twr (int init) {

	char debug[50];
	dw_device_t * dev = get_device();
	uint32 partid;
	partid = _dwt_otpread(PARTID_ADDRESS);
	int role = TAG;

	if (init == 1) {
		int lora_active = 0;
		lora_active = isLoraConnected();
		sprintf(debug, "[LORA] %s", lora_active ? "ACTIVATED" : "DISACTIVATED");
		println(debug);
		role = (lora_active ? TAG : ANCHOR);
	} else {
		role = ANCHOR;
	}

	if (init == 1) {
		sprintf(debug, "********* %s *********", (role == TAG ? "TAG" : "ANCHOR"));
		println(debug);
		sprintf(debug, "[LPL] %s", (LPL_MODE == 1 ? "ACTIVATED" : "DISACTIVATED"));
		println(debug);
	}
	init_config(role, DWT_BR_6M8, init);
	if (init) {
		println("init finished");
		/*double ask_time = 0, current_time = 0, previous_time = 0, global_time = 0, save_time = 0;
		int count = 0;
		ask_time = get_systime_s();
		sprintf(debug, "time: %.3fs", ask_time);
		println(debug);
		while (1) {
			current_time = get_systime_s();
			if (current_time < previous_time) {
				double diff = previous_time + current_time;
				sprintf(debug, "diff : %.3f", diff);
				println(debug);
				ask_time -= diff;
			}
			if (ask_time < current_time - 5) {
				println("TIMEOUT");
				ask_time = current_time;// - 17.208;
				sprintf(debug, "at: %.3fs", ask_time);
				println(debug);
			}
			previous_time = current_time;
		}*/
	}
	if (role == TAG) {
		init_lora();
		tag_dev();
	} else {
		anch_dev(init);
	}
	return 1;
}
