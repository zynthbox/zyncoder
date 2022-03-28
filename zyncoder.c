/*
 * ******************************************************************
 * ZYNTHIAN PROJECT: Zyncoder Library
 * 
 * Library for interfacing Rotary Encoders & Switches connected 
 * to RBPi native GPIOs or expanded with MCP23008. Includes an 
 * emulator mode to ease developping.
 * 
 * Copyright (C) 2015-2018 Fernando Moyano <jofemodo@zynthian.org>
 *
 * ******************************************************************
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * For a full copy of the GNU General Public License see the LICENSE.txt file.
 * 
 * ******************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "zyncoder.h"
#include "zyncontrol.h"

#if defined(HAVE_WIRINGPI_LIB)
	#include <wiringPi.h>
	#include <wiringPiI2C.h>
	#include <mcp23017.h>
	#include <mcp23x0817.h>
	#include <mcp23008.h>

	#if defined(MCP23017_ENCODERS)
		// pins 100-115 are located on the MCP23017
		#define MCP23017_BASE_PIN 100
		// define default I2C Address for MCP23017
		#if !defined(MCP23017_I2C_ADDRESS)
			#define MCP23017_I2C_ADDRESS 0x20
		#endif
		// define default interrupt pins for the MCP23017
		#if !defined(MCP23017_INTA_PIN)
			#define MCP23017_INTA_PIN 27
		#endif
		#if !defined(MCP23017_INTB_PIN)
			#define MCP23017_INTB_PIN 25
		#endif
	#elif defined(MCP23008_ENCODERS)
		// pins 100-107 are located on the MCP23008
		#define MCP23008_BASE_PIN 100
		#define MCP23008_I2C_ADDRESS 0x20
	#endif

#else
	#define MCP23008_BASE_PIN 100
	#define MCP23008_I2C_ADDRESS 0x20
	#include "wiringPiEmu.h"
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

//#define DEBUG

//-----------------------------------------------------------------------------
// Library Initialization
//-----------------------------------------------------------------------------

void zynswitch_rbpi_ISR(uint8_t i);
void (*zynswitch_rbpi_ISRs[]);

int init_zynlib() {
	if (!init_zyncoder()) return 0;
	if (!init_zynmidirouter()) return 0;
	#ifdef ZYNAPTIK_CONFIG
	if (!init_zynaptik()) return 0;
	#endif
	#ifdef ZYNTOF_CONFIG
	if (!init_zyntof()) return 0;
	#endif
	return 1;
}

int end_zynlib() {
	#ifdef ZYNTOF_CONFIG
	if (!end_zyntof()) return 0;
	#endif
	#ifdef ZYNAPTIK_CONFIG
	if (!end_zynaptik()) return 0;
	#endif
	if (!end_zynmidirouter()) return 0;
	if (!end_zyncoder()) return 0;
	return 1;
}

//-----------------------------------------------------------------------------
// Zyncoder Library Initialization
//-----------------------------------------------------------------------------

//Switch Polling interval
int poll_zynswitches_us=10000;

//Switches Polling Thread (should be avoided!)
pthread_t init_poll_zynswitches();

#ifdef MCP23017_ENCODERS
// wiringpi node structure for direct access to the mcp23017
struct wiringPiNodeStruct *zyncoder_mcp23017_node;

// two ISR routines for the two banks
void zyncoder_mcp23017_bankA_ISR() {
	zyncoder_mcp23017_ISR(zyncoder_mcp23017_node, MCP23017_BASE_PIN, 0);
}
void zyncoder_mcp23017_bankB_ISR() {
	zyncoder_mcp23017_ISR(zyncoder_mcp23017_node, MCP23017_BASE_PIN, 1);
}
void (*zyncoder_mcp23017_bank_ISRs[2])={
	zyncoder_mcp23017_bankA_ISR,
	zyncoder_mcp23017_bankB_ISR
};
#endif


void reset_zynswitches() {
	int i;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		zynswitches[i].enabled = 0;
		zynswitches[i].midi_event.type = NONE_EVENT;
	}
}

void reset_zyncoders() {
	int i,j;
	for (i=0;i<MAX_NUM_ZYNCODERS;i++) {
		zyncoders[i].enabled = 0;
		zyncoders[i].inv = 0;
		zyncoders[i].value = 0;
		zyncoders[i].value_flag = 0;
 		zyncoders[i].zpot_i = -1;
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT;j++)
			zyncoders[i].dtus[j] = 0;
	}
}

unsigned int int_to_int(unsigned int k) {
	return (k == 0 || k == 1 ? k : ((k % 2) + 10 * int_to_int(k / 2)));
}

int init_zyncoder() {
    if (!init_zyncontrol()) return 0;
    /*
	int i,j;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		zynswitches[i].enabled=0;
		zynswitches[i].midi_event.type=NONE_EVENT;
	}
	for (i=0;i<MAX_NUM_ZYNCODERS;i++) {
		zyncoders[i].enabled=0;
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT;j++) zyncoders[i].dtus[j]=0;
	}
	wiringPiSetup();

#if defined(MCP23017_ENCODERS)
	zyncoder_mcp23017_node = init_mcp23017(MCP23017_BASE_PIN, MCP23017_I2C_ADDRESS, MCP23017_INTA_PIN, MCP23017_INTB_PIN, zyncoder_mcp23017_bank_ISRs);
#elif defined(MCP23008_ENCODERS)   
	mcp23008Setup(MCP23008_BASE_PIN, MCP23008_I2C_ADDRESS);
	init_poll_zynswitches();
#endif
*/
	return 1;
}

int end_zyncoder() {
	return 1;
}

#ifndef MCP23008_ENCODERS 
struct wiringPiNodeStruct * init_mcp23017(int base_pin, uint8_t i2c_address, uint8_t inta_pin, uint8_t intb_pin, void (*isrs[2])) {
	uint8_t reg;

	mcp23017Setup(base_pin, i2c_address);

	// get the node corresponding to our mcp23017 so we can do direct writes
	struct wiringPiNodeStruct * mcp23017_node = wiringPiFindNode(base_pin);

	// setup all the pins on the banks as inputs and disable pullups on
	// the zyncoder input
	reg = 0xff;
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IODIRA, reg);
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IODIRB, reg);

	// enable pullups on the unused pins (high two bits on each bank)
	reg = 0xff;
	//reg = 0xc0;
	//reg = 0x60;
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_GPPUA, reg);
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_GPPUB, reg);

	// disable polarity inversion
	reg = 0;
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IPOLA, reg);
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IPOLB, reg);

	// disable the comparison to DEFVAL register
	reg = 0;
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_INTCONA, reg);
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_INTCONB, reg);

	// configure the interrupt behavior for bank A
	uint8_t ioconf_value = wiringPiI2CReadReg8(mcp23017_node->fd, MCP23x17_IOCON);
	bitWrite(ioconf_value, 6, 0);	// banks are not mirrored
	bitWrite(ioconf_value, 2, 0);	// interrupt pin is not floating
	bitWrite(ioconf_value, 1, 1);	// interrupt is signaled by high
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IOCON, ioconf_value);

	// configure the interrupt behavior for bank B
	ioconf_value = wiringPiI2CReadReg8(mcp23017_node->fd, MCP23x17_IOCONB);
	bitWrite(ioconf_value, 6, 0);	// banks are not mirrored
	bitWrite(ioconf_value, 2, 0);	// interrupt pin is not floating
	bitWrite(ioconf_value, 1, 1);	// interrupt is signaled by high
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_IOCONB, ioconf_value);

	// finally, enable the interrupt pins for banks a and b
	// enable interrupts on all pins
	reg = 0xff;
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_GPINTENA, reg);
	wiringPiI2CWriteReg8(mcp23017_node->fd, MCP23x17_GPINTENB, reg);

	// pi ISRs for the 23017
	// bank A
	wiringPiISR(inta_pin, INT_EDGE_RISING, isrs[0]);
	// bank B
	wiringPiISR(intb_pin, INT_EDGE_RISING, isrs[1]);

	//Read data for first time ...
	wiringPiI2CReadReg8(mcp23017_node->fd, MCP23x17_GPIOA);
	wiringPiI2CReadReg8(mcp23017_node->fd, MCP23x17_GPIOB);

	#ifdef DEBUG
	printf("ZynCore: MCP23017 at I2C %x initialized in base-pin %d: INTA %d, INTB %d\n", i2c_address, base_pin, inta_pin, intb_pin);
	#endif

	return mcp23017_node;
}
#endif

//-----------------------------------------------------------------------------
// GPIO Switches
//-----------------------------------------------------------------------------

void send_zynswitch_midi(struct zynswitch_st *zynswitch, uint8_t status) {
	if (zynswitch->midi_event.type==CTRL_CHANGE) {
		uint8_t val=0;
		if (status==0) val=127;
		//Send MIDI event to engines and ouput (ZMOPS)
		internal_send_ccontrol_change(zynswitch->midi_event.chan, zynswitch->midi_event.num, val);
		//Update zyncoders
		midi_event_zyncoders(zynswitch->midi_event.chan, zynswitch->midi_event.num, val);
		//Send MIDI event to UI
		write_zynmidi_ccontrol_change(zynswitch->midi_event.chan, zynswitch->midi_event.num, val);
		//printf("Zyncoder: Zynswitch MIDI CC event (chan=%d, num=%d) => %d\n",zynswitch->midi_event.chan, zynswitch->midi_event.num, val);
	}
	else if (zynswitch->midi_event.type==NOTE_ON) {
		if (status==0) {
			//Send MIDI event to engines and ouput (ZMOPS)
			internal_send_note_on(zynswitch->midi_event.chan, zynswitch->midi_event.num, 127);
			//Send MIDI event to UI
			write_zynmidi_note_on(zynswitch->midi_event.chan, zynswitch->midi_event.num, 127);
			//printf("Zyncoder: Zynswitch MIDI Note-On event (chan=%d, num=%d) => %d\n",zynswitch->midi_event.chan, zynswitch->midi_event.num, 127);
		}
		else {
			//Send MIDI event to engines and ouput (ZMOPS)
			internal_send_note_off(zynswitch->midi_event.chan, zynswitch->midi_event.num, 0);
			//Send MIDI event to UI
			write_zynmidi_note_off(zynswitch->midi_event.chan, zynswitch->midi_event.num, 0);
			//printf("Zyncoder: Zynswitch MIDI Note-Off event (chan=%d, num=%d) => %d\n",zynswitch->midi_event.chan, zynswitch->midi_event.num, 0);
		}
	}
	else if (zynswitch->midi_event.type==PROG_CHANGE) {
		if (status==0) {
			//Send MIDI event to engines and ouput (ZMOPS)
			internal_send_program_change(zynswitch->midi_event.chan, zynswitch->midi_event.num);
			//Send MIDI event to UI
			write_zynmidi_program_change(zynswitch->midi_event.chan, zynswitch->midi_event.num);
			//printf("Zyncoder: Zynswitch MIDI Program Change event (chan=%d, num=%d)\n",zynswitch->midi_event.chan, zynswitch->midi_event.num);
		}
	}
}

int get_num_zynswitches() {
	int i;
	int n = 0;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		if (zynswitches[i].enabled!=0) n++;
	}
	return n;
}

int get_last_zynswitch_index() {
	int i;
	int li = 0;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		if (zynswitches[i].enabled!=0) li = i;
	}
	return li;
}

#ifdef MCP23008_ENCODERS
//Update ISR switches (native GPIO)
void update_zynswitch(uint8_t i) {
#else
// Update the mcp23017 based switches from ISR routine
void update_zynswitch(uint8_t i, uint8_t status) {
#endif
	if (i>=MAX_NUM_ZYNSWITCHES) return;
	struct zynswitch_st *zynswitch = zynswitches + i;
	if (zynswitch->enabled==0) return;

#ifdef MCP23008_ENCODERS
	uint8_t status=digitalRead(zynswitch->pin);
#endif
	if (status==zynswitch->status) return;
	zynswitch->status=status;

	send_zynswitch_midi(zynswitch, status);

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	unsigned long int tsus=ts.tv_sec*1000000 + ts.tv_nsec/1000;

	//printf("SWITCH ISR %d => STATUS=%d (%lu)\n",i,zynswitch->status,tsus);
	if (zynswitch->status==1) {
		if (zynswitch->tsus>0) {
			unsigned int dtus=tsus-zynswitch->tsus;
			zynswitch->tsus=0;
			//Ignore spurious ticks
			if (dtus<1000) return;
			//printf("Debounced Switch %d\n",i);
			zynswitch->dtus=dtus;
		}
	} else zynswitch->tsus=tsus;
}

#ifdef MCP23008_ENCODERS
void update_zynswitch_0() { update_zynswitch(0); }
void update_zynswitch_1() { update_zynswitch(1); }
void update_zynswitch_2() { update_zynswitch(2); }
void update_zynswitch_3() { update_zynswitch(3); }
void update_zynswitch_4() { update_zynswitch(4); }
void update_zynswitch_5() { update_zynswitch(5); }
void update_zynswitch_6() { update_zynswitch(6); }
void update_zynswitch_7() { update_zynswitch(7); }
void (*update_zynswitch_funcs[8])={
	update_zynswitch_0,
	update_zynswitch_1,
	update_zynswitch_2,
	update_zynswitch_3,
	update_zynswitch_4,
	update_zynswitch_5,
	update_zynswitch_6,
	update_zynswitch_7
};
#endif

//Update NON-ISR switches (expanded GPIO)
void update_expanded_zynswitches() {
	struct timespec ts;
	unsigned long int tsus;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	tsus=ts.tv_sec*1000000 + ts.tv_nsec/1000;

	int i;
	uint8_t status;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		struct zynswitch_st *zynswitch = zynswitches + i;
		if (!zynswitch->enabled || zynswitch->pin<100) continue;
		status=digitalRead(zynswitch->pin);
		//printf("POLLING SWITCH %d (%d) => %d\n",i,zynswitch->pin,status);
		if (status==zynswitch->status) continue;
		zynswitch->status=status;
		send_zynswitch_midi(zynswitch, status);
		//printf("POLLING SWITCH %d => STATUS=%d (%lu)\n",i,zynswitch->status,tsus);
		if (zynswitch->status==1) {
			if (zynswitch->tsus>0) {
				unsigned int dtus=tsus-zynswitch->tsus;
				zynswitch->tsus=0;
				//Ignore spurious ticks
				if (dtus<1000) return;
				//printf("Debounced Switch %d\n",i);
				zynswitch->dtus=dtus;
			}
		} else zynswitch->tsus=tsus;
	}
}

void * poll_zynswitches(void *arg) {
	while (1) {
		update_expanded_zynswitches();
		usleep(poll_zynswitches_us);
	}
	return NULL;
}

pthread_t init_poll_zynswitches() {
	pthread_t tid;
	int err=pthread_create(&tid, NULL, &poll_zynswitches, NULL);
	if (err != 0) {
		printf("Zyncoder: Can't create zynswitches poll thread :[%s]", strerror(err));
		return 0;
	} else {
		printf("Zyncoder: Zynswitches poll thread created successfully\n");
		return tid;
	}
}

//-----------------------------------------------------------------------------

struct zynswitch_st *setup_zynswitch(uint8_t i, uint8_t pin) {
	if (i >= MAX_NUM_ZYNSWITCHES) {
		printf("Zyncoder: Maximum number of zynswitches exceeded: %d\n", MAX_NUM_ZYNSWITCHES);
		return NULL;
	}
	
	struct zynswitch_st *zynswitch = zynswitches + i;
	zynswitch->enabled = 1;
	zynswitch->pin = pin;
    zynswitch->push=0;
	zynswitch->tsus = 0;
	zynswitch->dtus = 0;
	zynswitch->status = 0;

	if (pin>0) {
		pinMode(pin, INPUT);
		pullUpDnControl(pin, PUD_UP);

		// RBPi GPIO pin
		if (pin<100) {
			wiringPiISR(pin,INT_EDGE_BOTH, zynswitch_rbpi_ISRs[i]);
			zynswitch_rbpi_ISR(i);
		}
		// MCP23017 pin
		else if (pin>=100) {
			zynswitch_mcp23017_update(i);
		}
	}

	return zynswitch;
}

int setup_zynswitch_midi(uint8_t i, uint8_t midi_evt, uint8_t midi_chan, uint8_t midi_num) {
	if (i >= MAX_NUM_ZYNSWITCHES) {
		printf("Zyncoder: Maximum number of zynswitches exceeded: %d\n", MAX_NUM_ZYNSWITCHES);
		return 0;
	}

	struct zynswitch_st *zynswitch = zynswitches + i;
	zynswitch->midi_event.type = midi_evt;
	zynswitch->midi_event.chan = midi_chan;
	zynswitch->midi_event.num = midi_num;
	//printf("Zyncoder: Set Zynswitch %u MIDI %x: %u, %u\n", i, midi_evt, midi_chan, midi_num);

	return 1;
}

unsigned int get_zynswitch_dtus(uint8_t i, unsigned int long_dtus) {
	if (i >= MAX_NUM_ZYNSWITCHES) return -1;

	unsigned int dtus=zynswitches[i].dtus;
	if (dtus>0) {
		zynswitches[i].dtus=0;
		return dtus;
	}
	else if (zynswitches[i].tsus>0) {
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		dtus=ts.tv_sec*1000000 + ts.tv_nsec/1000 - zynswitches[i].tsus;

		/**
		 * Disable long events
		 *
		 * Long events caused the keypress to emit release event after a certain time
		 * which conflicts with zynthbox button usage where we need have the pressed event
		 * as long as the button is pressed and emit release event only when button is released
		 *
		 * Always return 0 instead of the check to determine if the press event exceeded long press
		 * time and hence disable all long events as zynthbox doesnt need any long press event
		 */
		/*if (dtus>long_dtus) {
			zynswitches[i].tsus=0;
			return dtus;
		} else {
			return 0;
		}*/

		return 0;
	}
	return -1;
}

unsigned int get_zynswitch(uint8_t i, unsigned int long_dtus) {
	return get_zynswitch_dtus(i, long_dtus);
}

//-----------------------------------------------------------------------------
// Generic Rotary Encoders
//-----------------------------------------------------------------------------

void midi_event_zyncoders(uint8_t midi_chan, uint8_t midi_ctrl, uint8_t val) {
	//Update zyncoder value => TODO Optimize this fragment!!!
	int j;
	for (j=0;j<MAX_NUM_ZYNCODERS;j++) {
		if (zyncoders[j].enabled && zyncoders[j].midi_chan==midi_chan && zyncoders[j].midi_ctrl==midi_ctrl) {
			zyncoders[j].value=val;
			zyncoders[j].subvalue=val*ZYNCODER_TICKS_PER_RETENT;
			//fprintf (stdout, "ZynMidiRouter: MIDI CC (%x, %x) => UI",midi_chan,midi_ctrl);
		}
	}
}

void send_zyncoder(uint8_t i) {
	if (i>=MAX_NUM_ZYNCODERS) return;
	struct zyncoder_st *zyncoder = zyncoders + i;
	if (zyncoder->enabled==0) return;
	if (zyncoder->midi_ctrl>0) {
		//Send to MIDI output
		internal_send_ccontrol_change(zyncoder->midi_chan,zyncoder->midi_ctrl,zyncoder->value);
		//Send to MIDI controller feedback => TODO: Reverse Mapping!!
		//ctrlfb_send_ccontrol_change(zyncoder->midi_chan,zyncoder->midi_ctrl,zyncoder->value);
		//printf("Zyncoder: SEND MIDI CH#%d, CTRL %d = %d\n",zyncoder->midi_chan,zyncoder->midi_ctrl,zyncoder->value);
	} else if (zyncoder->osc_lo_addr!=NULL && zyncoder->osc_path[0]) {
		if (zyncoder->step >= 8) {
			if (zyncoder->value>=64) {
				lo_send(zyncoder->osc_lo_addr,zyncoder->osc_path, "T");
				//printf("SEND OSC %s => T\n",zyncoder->osc_path);
			} else {
				lo_send(zyncoder->osc_lo_addr,zyncoder->osc_path, "F");
				//printf("SEND OSC %s => F\n",zyncoder->osc_path);
			}
		} else {
			lo_send(zyncoder->osc_lo_addr,zyncoder->osc_path, "i",zyncoder->value);
			//printf("SEND OSC %s => %d\n",zyncoder->osc_path,zyncoder->value);
		}
	}
}

#ifdef MCP23008_ENCODERS
void update_zyncoder(uint8_t i) {
#else
void update_zyncoder(uint8_t i, uint8_t MSB, uint8_t LSB) {
#endif
	if (i>=MAX_NUM_ZYNCODERS) return;
	struct zyncoder_st *zyncoder = zyncoders + i;
	if (zyncoder->enabled==0) return;

#ifdef MCP23008_ENCODERS
	uint8_t MSB = digitalRead(zyncoder->pin_a);
	uint8_t LSB = digitalRead(zyncoder->pin_b);
#endif
	uint8_t encoded = (MSB << 1) | LSB;
	uint8_t sum = (zyncoder->last_encoded << 2) | encoded;
	uint8_t up=(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011);
	uint8_t down=0;
	if (!up) down=(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000);
#ifdef DEBUG
	printf("zyncoder %2d - %08d\t%08d\t%d\t%d\n", i, int_to_int(encoded), int_to_int(sum), up, down);
#endif
	zyncoder->last_encoded=encoded;

	if (zyncoder->step==0) {
		//Get time interval from last tick
		struct timespec ts;
		unsigned long int tsus;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		tsus=ts.tv_sec*1000000 + ts.tv_nsec/1000;
		unsigned int dtus=tsus-zyncoder->tsus;
		//printf("ZYNCODER ISR %d => SUBVALUE=%d (%u)\n",i,zyncoder->subvalue,dtus);
		//Ignore spurious ticks
		if (dtus<1000) return;
		//printf("ZYNCODER DEBOUNCED ISR %d => SUBVALUE=%d (%u)\n",i,zyncoder->subvalue,dtus);
		//Calculate average dtus for the last ZYNCODER_TICKS_PER_RETENT ticks
		int j;
		unsigned int dtus_avg=dtus;
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT;j++) dtus_avg+=zyncoder->dtus[j];
		dtus_avg/=(ZYNCODER_TICKS_PER_RETENT+1);
		//Add last dtus to fifo array
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT-1;j++)
			zyncoder->dtus[j]=zyncoder->dtus[j+1];
		zyncoder->dtus[j]=dtus;
		//Calculate step value
		int dsval=10000*ZYNCODER_TICKS_PER_RETENT/dtus_avg;
		if (dsval<1) dsval=1;
		else if (dsval>2*ZYNCODER_TICKS_PER_RETENT) dsval=2*ZYNCODER_TICKS_PER_RETENT;

		int value=-1;
		if (up) {
			if (zyncoder->max_value-zyncoder->subvalue>=dsval) zyncoder->subvalue=(zyncoder->subvalue+dsval);
			else zyncoder->subvalue=zyncoder->max_value;
			value=zyncoder->subvalue/ZYNCODER_TICKS_PER_RETENT;
		}
		else if (down) {
			if (zyncoder->subvalue>=dsval) zyncoder->subvalue=(zyncoder->subvalue-dsval);
			else zyncoder->subvalue=0;
			value=(zyncoder->subvalue+ZYNCODER_TICKS_PER_RETENT-1)/ZYNCODER_TICKS_PER_RETENT;
		}

		zyncoder->tsus=tsus;
		if (value>=0 && zyncoder->value!=value) {
			//printf("DTUS=%d, %d (%d)\n",dtus_avg,value,dsval);
			zyncoder->value=value;
			send_zyncoder(i);
		}
	} 
	else {
		unsigned int last_value=zyncoder->value;
		if (zyncoder->value>zyncoder->max_value) zyncoder->value=zyncoder->max_value;
		if (zyncoder->max_value-zyncoder->value>=zyncoder->step && up) zyncoder->value+=zyncoder->step;
		else if (zyncoder->value>=zyncoder->step && down) zyncoder->value-=zyncoder->step;
		if (last_value!=zyncoder->value) send_zyncoder(i);
	}

}

#ifdef MCP23008_ENCODERS
void update_zyncoder_0() { update_zyncoder(0); }
void update_zyncoder_1() { update_zyncoder(1); }
void update_zyncoder_2() { update_zyncoder(2); }
void update_zyncoder_3() { update_zyncoder(3); }
void update_zyncoder_4() { update_zyncoder(4); }
void update_zyncoder_5() { update_zyncoder(5); }
void update_zyncoder_6() { update_zyncoder(6); }
void update_zyncoder_7() { update_zyncoder(7); }
void (*update_zyncoder_funcs[8])={
	update_zyncoder_0,
	update_zyncoder_1,
	update_zyncoder_2,
	update_zyncoder_3,
	update_zyncoder_4,
	update_zyncoder_5,
	update_zyncoder_6,
	update_zyncoder_7
};
#endif

//-----------------------------------------------------------------------------

struct zyncoder_st *setup_zyncoder(uint8_t i, uint8_t pin_a, uint8_t pin_b, uint8_t midi_chan, uint8_t midi_ctrl, char *osc_path, unsigned int value, unsigned int max_value, unsigned int step) {
	if (i > MAX_NUM_ZYNCODERS) {
		printf("Zyncoder: Maximum number of zyncoders exceded: %d\n", MAX_NUM_ZYNCODERS);
		return NULL;
	}

	struct zyncoder_st *zyncoder = zyncoders + i;

	//Setup MIDI/OSC bindings
	if (midi_chan>15) midi_chan=0;
	if (midi_ctrl>127) midi_ctrl=1;
	zyncoder->midi_chan = midi_chan;
	zyncoder->midi_ctrl = midi_ctrl;

	//printf("OSC PATH: %s\n",osc_path);
	if (osc_path) {
		char *osc_port_str=strtok(osc_path,":");
		zyncoder->osc_port=atoi(osc_port_str);
		if (zyncoder->osc_port>0) {
			zyncoder->osc_lo_addr=lo_address_new(NULL,osc_port_str);
			strcpy(zyncoder->osc_path,strtok(NULL,":"));
		} else {
			zyncoder->osc_path[0] = 0;
		}
	} else {
		zyncoder->osc_path[0] = 0;
	}

	if (value>max_value) value=max_value;
	zyncoder->step = step;
	if (step>0) {
		zyncoder->value = value;
		zyncoder->subvalue = 0;
		zyncoder->max_value = max_value;
	} else {
		zyncoder->value = value;
		zyncoder->subvalue = ZYNCODER_TICKS_PER_RETENT*value;
		zyncoder->max_value = ZYNCODER_TICKS_PER_RETENT*max_value;
	}

	if (zyncoder->enabled==0 || zyncoder->pin_a!=pin_a || zyncoder->pin_b!=pin_b) {
		zyncoder->enabled = 1;
		zyncoder->pin_a = pin_a;
		zyncoder->pin_b = pin_b;
		zyncoder->last_encoded = 0;
		zyncoder->tsus = 0;

		if (zyncoder->pin_a!=zyncoder->pin_b) {
			pinMode(pin_a, INPUT);
			pinMode(pin_b, INPUT);
			pullUpDnControl(pin_a, PUD_UP);
			pullUpDnControl(pin_b, PUD_UP);

#if defined(MCP23017_ENCODERS) 
			// this is a bit brute force, but update all the banks
			zyncoder_mcp23017_bankA_ISR();
			zyncoder_mcp23017_bankB_ISR();
#elif defined(MCP23008_ENCODERS) 
			wiringPiISR(pin_a,INT_EDGE_BOTH, update_zyncoder_funcs[i]);
			wiringPiISR(pin_b,INT_EDGE_BOTH, update_zyncoder_funcs[i]);
#endif
		}
	}

	return zyncoder;
}

int setup_rangescale_zyncoder(uint8_t i, int32_t min_value, int32_t max_value, int32_t value, int32_t step) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->setup_rangescale_zyncoder(%d, ...): Invalid index!\n", i);
		return 0;
	}
	if (min_value==max_value) {
		printf("ZynCore->setup_rangescale_zyncoder(%d, %d, %d, ...): Invalid range!\n", i, min_value, max_value);
		return 0;
	}

	zyncoder_t *zcdr = zyncoders + i;

	if (min_value>max_value) {
		int32_t swapv = min_value;
		min_value = max_value;
		max_value = swapv;
		zcdr->inv = 1;
	}
	else {
		zcdr->inv = 0;
	}

	if (value>max_value) value = max_value;
	else if (value<min_value) value = min_value;

	zcdr->step = step;
	if (step==0) {
		zcdr->value = value;
		zcdr->subvalue = ZYNCODER_TICKS_PER_RETENT * value;
		zcdr->min_value = ZYNCODER_TICKS_PER_RETENT * min_value;
		zcdr->max_value = ZYNCODER_TICKS_PER_RETENT * (max_value + 1) - 1;
	} else {
		zcdr->value = value;
		zcdr->subvalue = 0;
		zcdr->min_value = min_value;
		zcdr->max_value = max_value;
	}
	zcdr->value_flag = 0;
}

unsigned int get_value_zyncoder(uint8_t i) {
	if (i >= MAX_NUM_ZYNCODERS) return 0;
	return zyncoders[i].value;
}

uint8_t get_value_flag_zyncoder(uint8_t i) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->get_value_flag_zyncoder(%d): Invalid index!\n", i);
		return 0;
	}
	return zyncoders[i].value_flag;
}

void set_value_zyncoder(uint8_t i, unsigned int v, int send) {
	if (i >= MAX_NUM_ZYNCODERS) return;
	struct zyncoder_st *zyncoder = zyncoders + i;
	if (zyncoder->enabled==0) return;

	//unsigned int last_value=zyncoder->value;
	if (zyncoder->step==0) {
		v*=ZYNCODER_TICKS_PER_RETENT;
		if (v>zyncoder->max_value) zyncoder->subvalue=zyncoder->max_value;
		else zyncoder->subvalue=v;
		zyncoder->value=zyncoder->subvalue/ZYNCODER_TICKS_PER_RETENT;
	} else {
		if (v>zyncoder->max_value) zyncoder->value=zyncoder->max_value;
		else zyncoder->value=v;
	}
	if (send) send_zyncoder(i);
}

//-----------------------------------------------------------------------------
// RBPi GPIO ISR
//-----------------------------------------------------------------------------

void zynswitch_rbpi_ISR(uint8_t i) {
	if (i>=MAX_NUM_ZYNSWITCHES) return;
	zynswitch_t *zsw = zynswitches + i;
	if (zsw->enabled==0) return;
	update_zynswitch(i, (uint8_t)digitalRead(zsw->pin));
}

void zynswitch_rbpi_ISR_0() { zynswitch_rbpi_ISR(0); }
void zynswitch_rbpi_ISR_1() { zynswitch_rbpi_ISR(1); }
void zynswitch_rbpi_ISR_2() { zynswitch_rbpi_ISR(2); }
void zynswitch_rbpi_ISR_3() { zynswitch_rbpi_ISR(3); }
void zynswitch_rbpi_ISR_4() { zynswitch_rbpi_ISR(4); }
void zynswitch_rbpi_ISR_5() { zynswitch_rbpi_ISR(5); }
void zynswitch_rbpi_ISR_6() { zynswitch_rbpi_ISR(6); }
void zynswitch_rbpi_ISR_7() { zynswitch_rbpi_ISR(7); }
void (*zynswitch_rbpi_ISRs[8])={
	zynswitch_rbpi_ISR_0,
	zynswitch_rbpi_ISR_1,
	zynswitch_rbpi_ISR_2,
	zynswitch_rbpi_ISR_3,
	zynswitch_rbpi_ISR_4,
	zynswitch_rbpi_ISR_5,
	zynswitch_rbpi_ISR_6,
	zynswitch_rbpi_ISR_7
};


void zyncoder_rbpi_ISR(uint8_t i) {
	if (i>=MAX_NUM_ZYNSWITCHES) return;
	zyncoder_t *zcdr = zyncoders + i;
	if (zcdr->enabled==0) return;
	update_zyncoder(i, (uint8_t)digitalRead(zcdr->pin_a), (uint8_t)digitalRead(zcdr->pin_b));
}

void zyncoder_rbpi_ISR_0() { zyncoder_rbpi_ISR(0); }
void zyncoder_rbpi_ISR_1() { zyncoder_rbpi_ISR(1); }
void zyncoder_rbpi_ISR_2() { zyncoder_rbpi_ISR(2); }
void zyncoder_rbpi_ISR_3() { zyncoder_rbpi_ISR(3); }
void (*zyncoder_rbpi_ISRs[8])={
	zyncoder_rbpi_ISR_0,
	zyncoder_rbpi_ISR_1,
	zyncoder_rbpi_ISR_2,
	zyncoder_rbpi_ISR_3,
};

//-----------------------------------------------------------------------------
// MCP23017 based encoders & switches
//-----------------------------------------------------------------------------

#ifndef MCP23008_ENCODERS

void zynswitch_mcp23017_update(uint8_t i) {
	if (i>=MAX_NUM_ZYNSWITCHES) return;
	zynswitch_t *zsw = zynswitches + i;
	if (zsw->enabled==0) return;

	uint8_t base_pin = (zsw->pin / 100) * 100;
	struct wiringPiNodeStruct * wpns = wiringPiFindNode(base_pin);

	uint8_t bit = zsw->pin % 100;
	uint8_t reg;
	// Bank A
	if (bit<8) {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOA);
	// Bank B
	} else if (bit<16) {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOB);
		bit-=8;
	// Pin out of range!!
	} else {
		printf("ZynCore: zynswitch_mcp23017_update(%d) => pin %d out of range!\n", i, bit);
	}
	update_zynswitch(i, (uint8_t)bitRead(reg, bit));
}

void zyncoder_mcp23017_update(uint8_t i) {
	if (i>=MAX_NUM_ZYNSWITCHES) return;
	zyncoder_t *zcdr = zyncoders + i;
	if (zcdr->enabled==0) return;

	uint8_t base_pin = (zcdr->pin_a / 100) * 100;
	struct wiringPiNodeStruct * wpns = wiringPiFindNode(base_pin);

	uint8_t bit_a = zcdr->pin_a % 100;
	uint8_t bit_b = zcdr->pin_b % 100;
	uint8_t reg;
	// Bank A
	if (bit_a<8 && bit_b<8) {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOA);
	// Bank B
	} else if (bit_a<16 && bit_b<16) {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOB);
		bit_a-=8;
		bit_b-=8;
	// Pin out of range!!
	} else {
		printf("ZynCore: zyncoder_mcp23017_update(%d) => pins (%d, %d) out of range or in different bank!\n", i, bit_a, bit_b);
	}
	uint8_t state_a = bitRead(reg, bit_a);
	uint8_t state_b = bitRead(reg, bit_b);
	update_zyncoder(i, state_a, state_b);
	zcdr->pin_a_last_state = state_a;
	zcdr->pin_b_last_state = state_b;
}

// ISR for handling the mcp23017 interrupts
void zyncoder_mcp23017_ISR(struct wiringPiNodeStruct *wpns, uint16_t base_pin, uint8_t bank) {
	// the interrupt has gone off for a pin change on the mcp23017
	// read the appropriate bank and compare pin states to last
	// on a change, call the update function as appropriate
	int i;
	uint8_t reg;
	uint8_t pin_min, pin_max;

	#ifdef DEBUG
	printf("zyncoder_mcp23017_ISR() => %d, %d\n", base_pin, bank);
	#endif

	if (bank == 0) {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOA);
		//reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_INTCAPA);
		pin_min = base_pin;
	} else {
		reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_GPIOB);
		//reg = wiringPiI2CReadReg8(wpns->fd, MCP23x17_INTCAPB);
		pin_min = base_pin + 8;
	}
	pin_max = pin_min + 7;

	// search all encoders and switches for a pin in the bank's range
	// if the last state != current state then this pin has changed
	// call the update function
	for (i=0; i<MAX_NUM_ZYNCODERS; i++) {
		zyncoder_t *zcdr = zyncoders + i;
		if (zcdr->enabled==0) continue;

		// if either pin is in the range
		if ((zcdr->pin_a >= pin_min && zcdr->pin_a <= pin_max) ||
		    (zcdr->pin_b >= pin_min && zcdr->pin_b <= pin_max)) {
			uint8_t bit_a = zcdr->pin_a - pin_min;
			uint8_t bit_b = zcdr->pin_b - pin_min;
			uint8_t state_a = bitRead(reg, bit_a);
			uint8_t state_b = bitRead(reg, bit_b);
			// if either bit is different
			if ((state_a != zcdr->pin_a_last_state) ||
			    (state_b != zcdr->pin_b_last_state)) {
				update_zyncoder(i, state_a, state_b);
				zcdr->pin_a_last_state = state_a;
				zcdr->pin_b_last_state = state_b;
			}
		}
	}
	for (i=0; i<MAX_NUM_ZYNSWITCHES; i++) {
		zynswitch_t *zsw = zynswitches + i;
		if (zsw->enabled == 0) continue;

		// check the pin range
		if (zsw->pin >= pin_min && zsw->pin <= pin_max) {
			uint8_t bit = zsw->pin - pin_min;
			uint8_t state = bitRead(reg, bit);
			#ifdef DEBUG
			printf("MCP23017 Zynswitch %d => %d\n",i,state);
			#endif
			if (state != zsw->status) {
				update_zynswitch(i, state);
				// note that the update function updates status with state
			}
		}
	}
}
#endif
