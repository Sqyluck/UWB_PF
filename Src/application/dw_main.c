/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"

uint32 inittestapplication(uint8 s1switch);

#define SWS1_SHF_MODE 0x02	//short frame mode (6.81M)
#define SWS1_CH5_MODE 0x04	//channel 5 mode
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_A1A_MODE 0x10  //anchor/tag address A1
#define SWS1_A2A_MODE 0x20  //anchor/tag address A2
#define SWS1_A3A_MODE 0x40  //anchor/tag address A3
#define SWS1_USB2SPI_MODE 0x78  //USB to SPI mode
#define SWS1_TXSPECT_MODE 0x38  //Continuous TX spectrum mode
#define SWS1_RESERVED_MODE1 0x18 //Reserved mode - not used
#define SWS1_RESERVED_MODE2 0x58 //Reserved mode - not used


                             //"1234567812345678"
#define SOFTWARE_VER_STRING    "Ver.2.25mx TREK" //16 bytes!

uint8 s1switch = 0;
int instance_anchaddr = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;
int instance_mode = ANCHOR;

#define LCD_BUFF_LEN (80)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];
uint32_t pauseTWRReports  = 0;
uint32_t printLCDTWRReports  = 0;
uint8_t sendTWRRawReports = 1;

//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
instanceConfig_t chConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
                    {
                        .channelNumber = 2,             // channel
                        .preambleCode = 4,              // preambleCode
                        .pulseRepFreq = DWT_PRF_16M,    // prf
                        .dataRate = DWT_BR_110K,        // datarate
                        .preambleLen = DWT_PLEN_1024,   // preambleLength
                        .pacSize = DWT_PAC32,           // pacSize
                        .nsSFD = 1,                     // non-standard SFD
                        .sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 2 - S1: 2 on, 3 off
                    {
						.channelNumber = 2,            // channel
						.preambleCode = 4,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
						.channelNumber = 5,             // channel
						.preambleCode = 3,              // preambleCode
						.pulseRepFreq = DWT_PRF_16M,    // prf
						.dataRate = DWT_BR_110K,        // datarate
						.preambleLen = DWT_PLEN_1024,   // preambleLength
						.pacSize = DWT_PAC32,           // pacSize
						.nsSFD = 1,                     // non-standard SFD
						.sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						.channelNumber = 5,            // channel
						.preambleCode = 3,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    }
};

//Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
sfConfig_t sfConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
					{
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
					},
#if (DISCOVERY == 1)
                    //mode 2 - S1: 2 on, 3 off
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (100),        //number of slots in the superframe (98 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*100),  //in ms => 1000 ms frame means 1 Hz location rate
						.tagPeriod_ms = (10*100), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
#else
                    //mode 2 - S1: 2 on, 3 off
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
#endif
                    //mode 3 - S1: 2 off, 3 on
                    {
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
                    }
};
// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(uint8 s1switch, uint8 mode)
{
    uint16 instAddress ;

    instance_anchaddr = (((s1switch & SWS1_A1A_MODE) << 2) + (s1switch & SWS1_A2A_MODE) + ((s1switch & SWS1_A3A_MODE) >> 2)) >> 4;

    if(mode == ANCHOR)
    {
    	if(instance_anchaddr > 3)
		{
			instAddress = GATEWAY_ANCHOR_ADDR | 0x4 ; //listener
		}
		else
		{
			instAddress = GATEWAY_ANCHOR_ADDR | instance_anchaddr;
		}
	}
    else
    {
    	instAddress = instance_anchaddr;
    }

    instance_set_16bit_address(instAddress);
}


//returns the use case / operational mode
int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 2;
    }

    return mode;
}

extern SPI_HandleTypeDef hspi1;

uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    int result;
    char debug [50];

    port_set_dw1000_slowrate();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instance_readdeviceid() ;
    sprintf(debug, "devId: %08X", devID);
	println(debug);
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
    	port_wakeup_dw1000();

        devID = instance_readdeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
        	println("DWT_DEVICE_ID Failed");
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    if((s1switch & SWS1_ANC_MODE) == 0)
    {
        instance_mode = TAG;
    }
    else
    {
        instance_mode = ANCHOR;
    }

    result = instance_init(instance_mode) ; // Set this instance mode (tag/anchor)
    if (0 > result) return(-1) ; // Some failure has occurred

    port_set_dw1000_fastrate();
    Sleep(1000);
    devID = dwt_readdevid(); //instance_readdeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
    	println("not supported device id");
    	sprintf(debug, "devId: %08X", devID);
		println(debug);
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }
    addressconfigure(s1switch, instance_mode) ; // set up default 16-bit address

    if((instance_mode == ANCHOR) && (instance_anchaddr > 0x3))
    {
    	//invalid configuration
    	//display "Reserved" on the LCD
    }
    else
    {
		// get mode selection (index) this has 4 values see chConfig struct initialiser for details.
		dr_mode = decarangingmode(s1switch);

		chan = chConfig[dr_mode].channelNumber ;

		instance_config(&chConfig[dr_mode], &sfConfig[dr_mode]) ;                  // Set operating channel etc
    }
    println("success");
    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/


void configure_continuous_txspectrum_mode(uint8 s1switch)
{
	//configure DW1000 into Continuous TX mode
	instance_starttxtest(0x1000);
	//measure the power
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	//user has to reset the board to exit mode
	while(1)
	{
		Sleep(2);
	}

}


/*
 * @fn      main()
 * @brief   main entry point
**/

int dw_main(void)
{
    int rx = 0;
    char debug[50];

    //peripherals_init();
    println("start");
    //spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

    /*s1switch = port_is_boot1_low() << 1
    		| port_is_switch_on(TA_SW1_3) << 2
    		| port_is_switch_on(TA_SW1_4) << 3
    		| port_is_switch_on(TA_SW1_5) << 4
		    | port_is_switch_on(TA_SW1_6) << 5
    		| port_is_switch_on(TA_SW1_7) << 6
    		| port_is_switch_on(TA_SW1_8) << 7;*/

    // logique inverse
    uint8 address[3] = { 0, 0, 0 };
    uint8 mode = ANCHOR; //TAG;
    uint8 channel = 2;
    uint8 dataRate = 0;
    s1switch = dataRate << 1           		   // dataRate: 6M8 / 110k
        		| (channel == 2 ? 0 : 1) << 2  // Channel: 2 / 5
        		| (mode) << 3                    // Mode: Anch / Tag
        		| address[2] << 4              // address 1
    		    | address[1] << 5              // address 2
        		| address[0] << 6              // address 3
        		| 0 << 7; // reserved

    sprintf(debug, "dr: %s, \nmode: %s, \nadd: %d", (dataRate ? "6M8" : "110k"), (mode == TAG ? "TAG" : "ANCHOR"), address[0] << 2 | address[1] << 1 | address[2]);
    println(debug);
    if((s1switch & SWS1_USB2SPI_MODE) == SWS1_USB2SPI_MODE)
    {
        int j = 1000000;

        memset(dataseq, 0, LCD_BUFF_LEN);

        while(j--);

        j = 1000000;

        while(j--);
        return 1;
    }
    else //run DecaRangeRTLS application for TREK
    {
        if(inittestapplication(s1switch) == (uint32)-1) {
        	println("--- INIT APP FAILED---");
        } else {
        	println("--- INIT APP ---");
        }
        // Is continuous spectrum test mode selected?
        if((s1switch & SWS1_TXSPECT_MODE) == SWS1_TXSPECT_MODE)
    	{
        	//this function does not return!
        	configure_continuous_txspectrum_mode(s1switch);
    	}
    }

    // Is reserved mode selected?
    if(((s1switch & SWS1_RESERVED_MODE1) == SWS1_RESERVED_MODE1)
    	||
    	((s1switch & SWS1_RESERVED_MODE2) == SWS1_RESERVED_MODE2)
    	)
	{
    	println("reserved mode selected");
    	//this function does not return!
    	return 1;
	}


    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    sprintf (debug, "MODE : %s", (instance_mode == 1 ? "ANCHOR" : "TAG"));
    println(debug);
    // main loop
    while(1)
    {
    	instance_data_t* inst = instance_get_local_structure_ptr(0);

    	int monitor_local = inst->monitor ;
    	int txdiff = (portGetTickCnt() - inst->timeofTx);

        instance_mode = instance_get_role();

        if(instance_mode == TAG)
    	{
    		tag_run();
    		// println("tag");
    	}
    	else
    	{
    		// println("anch");
    		anch_run();
    	}

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((monitor_local == 1) && ( txdiff > inst->slotDuration_ms))
        {
            inst->wait4ack = 0;

        	if(instance_mode == TAG)
			{
        		tag_process_rx_timeout(inst);
			}
			else //if(instance_mode == ANCHOR)
			{
				dwt_forcetrxoff();	//this will clear all events
				//dwt_rxreset();
				//enable the RX
				inst->testAppState = TA_RXE_WAIT ;
			}
        	inst->monitor = 0;
        }

        rx = instance_newrange();

        //if there is a new ranging report received or a new range has been calculated, then prepare data
        //to output over USB - Virtual COM port, and update the LCD
        if(rx != TOF_REPORT_NUL)
        {
            //led_off(LED_PC9);
            instance_cleardisttableall();
#if (READ_EVENT_COUNTERS == 1)
            {
            	n += sprintf((char*)&usbVCOMout[n], "me %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\r\n",
            			inst->ecounters.ARFE, inst->ecounters.CRCB,
            			inst->ecounters.CRCG, inst->ecounters.HPW,
            			inst->ecounters.RSL, inst->ecounters.PHE,
            			inst->ecounters.RTO, inst->ecounters.SFDTO,
            			inst->ecounters.TXF, inst->ecounters.TXW);
            }
#endif
        } //if new range present
    }


    return 0;
}


