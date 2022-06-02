/*
 * ******************************************************************
 * ZYNTHIAN PROJECT: Zyncoder Library
 * 
 * Library for interfacing Rotary Encoders & Switches connected 
 * to RBPi native GPIOs or expanded with MCP23008/MCP23017.
 * Includes an emulator mode for developing on desktop computers.
  * 
 * Copyright (C) 2015-2021 Fernando Moyano <jofemodo@zynthian.org>
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

//#define DEBUG

#ifdef ZYNAPTIK_CONFIG
	#include "zynaptik.h"
#endif

#include "zynpot.h"
#include "zyncoder.h"

//-----------------------------------------------------------------------------
// Function headers
//-----------------------------------------------------------------------------

void send_zynswitch_midi(zynswitch_t *zsw, uint8_t status);

void zynswitch_rbpi_ISR(uint8_t i);
void (*zynswitch_rbpi_ISRs[]);
void zyncoder_rbpi_ISR(uint8_t i);
void (*zyncoder_rbpi_ISRs[]);

int init_zynlib() {
	init_zyncontrol();
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
	return 1;
}

//-----------------------------------------------------------------------------
// Helper functions
//-----------------------------------------------------------------------------

#ifdef DEBUG
unsigned int int_to_int(unsigned int k) {
	return (k == 0 || k == 1 ? k : ((k % 2) + 10 * int_to_int(k / 2)));
}
#endif

//-----------------------------------------------------------------------------
// Zynswitch functions
//-----------------------------------------------------------------------------

void reset_zynswitches() {
	int i;
	for (i=0;i<MAX_NUM_ZYNSWITCHES;i++) {
		zynswitches[i].enabled = 0;
		zynswitches[i].midi_event.type = NONE_EVENT;
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

void update_zynswitch(uint8_t i, uint8_t status) {
	zynswitch_t *zsw = zynswitches + i;

	if (status==zsw->status) return;
	zsw->status=status;

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	unsigned long int tsus=ts.tv_sec*1000000 + ts.tv_nsec/1000;

	//printf("SWITCH ISR %d => STATUS=%d (%lu)\n",i,status,tsus);

	//If pushed ...
	if (zsw->tsus>0) {
		unsigned int dtus;
		dtus=tsus-zsw->tsus;

		//SW debouncing => Ignore spurious clicks
		if (dtus<1000) return;

		//Release
		if (zsw->status==1) {
			zsw->tsus=0;
			//printf("Debounced Switch %d\n",i);
			zsw->dtus=dtus;
		}
	}
	//Push
	else if (zsw->status==0) {
		zsw->push=1;
		zsw->tsus=tsus;		// Save push timestamp
	}
	//Send MIDI
	send_zynswitch_midi(zsw, status);
}

int setup_zynswitch(uint8_t i, uint16_t pin) {
	if (i >= MAX_NUM_ZYNSWITCHES) {
		printf("ZynCore->setup_zynswitch(%d, ...): Invalid index!\n", i);
		return 0;
	}
	
	zynswitch_t *zsw = zynswitches + i;
	zsw->enabled = 0;
	zsw->push=0;
	zsw->tsus = 0;
	zsw->dtus = 0;
	zsw->status = 0;

	if (pin>0) {
		pinMode(pin, INPUT);
		pullUpDnControl(pin, PUD_UP);

		// RBPi GPIO pin
		if (pin<100) {
			zsw->enabled = 1;
			zsw->pin = pin;
			wiringPiISR(pin,INT_EDGE_BOTH, zynswitch_rbpi_ISRs[i]);
			zynswitch_rbpi_ISR(i);
		} 
		// MCP23017 pin
		else if (pin>=100) {
			#if defined(MCP23008_ENCODERS)
				zsw->pin = pin;
				zsw->enabled = 1;
			#elif defined(MCP23017_ENCODERS)
				uint8_t j = pin2index_zynmcp23017(pin);
				if (j>=0) {
					uint8_t bit = pin - zynmcp23017s[j].base_pin;
					if (bit<16) {
						zsw->pin = pin;
						zsw->enabled = 1;
						setup_pin_action_zynmcp23017(pin, ZYNSWITCH_PIN_ACTION, i);
						zynswitch_update_zynmcp23017(i);
					}
					else {
						printf("ZynCore->setup_zynswitch(%d, %d): Pin out of range!\n",i, pin);
						return 0;
					}
				}
				else {
					printf("ZynCore->setup_zynswitch(%d, %d): Pin is not a MPC23017 pin!\n",i, pin);
					return 0;
				}
			#endif
		}
	}

	return 1;
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
	unsigned int dtus=zynswitches[i].dtus;
	if (dtus>0) {
		zynswitches[i].dtus=0;
		return dtus;
	}
	else if (zynswitches[i].tsus>0) {
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		dtus=ts.tv_sec*1000000 + ts.tv_nsec/1000 - zynswitches[i].tsus;
		if (dtus>long_dtus) {
			zynswitches[i].tsus=0;
			return dtus;
		}
	}
	return -1;
}

unsigned int get_zynswitch(uint8_t i, unsigned int long_dtus) {
	if (i >= MAX_NUM_ZYNSWITCHES) {
		printf("ZynCore->get_zynswitch(%d, ...): Invalid index!\n", i);
		return 0;
	}
	if (zynswitches[i].push) {
		zynswitches[i].push = 0;
		return 0;
	} else  {
		return get_zynswitch_dtus(i, long_dtus);
	}
}

int get_next_pending_zynswitch(uint8_t i) {
	while (i<MAX_NUM_ZYNSWITCHES) {
		if (zynswitches[i].dtus>0 || zynswitches[i].tsus>0) return (int)i;
		i++;
	} 
	return -1;
}

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

//-----------------------------------------------------------------------------
// Zyncoder's zynpot API
//-----------------------------------------------------------------------------

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

int get_num_zyncoders() {
	int i;
	int n = 0;
	for (i=0;i<MAX_NUM_ZYNCODERS;i++) {
		if (zyncoders[i].enabled!=0) n++;
	}
	return n;
}

void update_zyncoder(uint8_t i, uint8_t msb, uint8_t lsb) {
	zyncoder_t *zcdr = zyncoders + i;

	//Software Debouncing =>
	//Get time interval from last tick
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	unsigned long int tsus=ts.tv_sec*1000000 + ts.tv_nsec/1000;
	unsigned int dtus=tsus-zcdr->tsus;
	//printf("ZYNCODER ISR %d => %u\n",i,dtus);
	//Ignore spurious ticks
	if (dtus<1000) {
		#ifdef DEBUG
		printf("zyncoder %d => Dropped Step (bouncing: %u)\n",i,dtus);
		#endif
		return;
	}
	//printf("ZYNCODER DEBOUNCED ISR %d => %u\n",i,dtus);

	//Calculate rotation direction => Quadrature Encoder Algorithm
	int spin;
	uint8_t encoded = (msb << 1) | lsb;
	uint8_t sum = (zcdr->last_encoded << 2) | encoded;
	zcdr->last_encoded = encoded;
	switch(sum) {
		case 0b1101:
		case 0b0100:
		case 0b0010:
		case 0b1011:
			spin = 1;
			break;
		case 0b1110:
		case 0b0111:
		case 0b0001:
		case 0b1000:
			spin = -1;
			break;
		default:
			#ifdef DEBUG
			printf("zyncoder %d => Dropped Step (invalid quadrature sequence: %08d)\n",i,int_to_int(sum));
			#endif
			return;
	}
	if (zcdr->inv) spin = -spin;
	#ifdef DEBUG
	printf("zyncoder %d - %08d\t%08d\t%d\n", i, int_to_int(encoded), int_to_int(sum), spin);
	#endif

	int32_t value;
	//Adaptative Step Size
	if (zcdr->step==0) {
		//printf("ZYNCODER DEBOUNCED ISR %d => SUBVALUE=%d (%u)\n",i,zcdr->subvalue,dtus);
		//Calculate average dtus for the last ZYNCODER_TICKS_PER_RETENT ticks
		int j;
		unsigned int dtus_avg=dtus;
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT;j++) dtus_avg+=zcdr->dtus[j];
		dtus_avg/=(ZYNCODER_TICKS_PER_RETENT+1);
		//Add last dtus to fifo array
		for (j=0;j<ZYNCODER_TICKS_PER_RETENT-1;j++)
			zcdr->dtus[j]=zcdr->dtus[j+1];
		zcdr->dtus[j]=dtus;
		//Calculate step value
		int32_t dsval=10000*ZYNCODER_TICKS_PER_RETENT/dtus_avg;
		if (dsval<1) dsval=1;
		else if (dsval>2*ZYNCODER_TICKS_PER_RETENT) dsval=2*ZYNCODER_TICKS_PER_RETENT;

		int32_t sv;
		if (spin>0) {
			sv = zcdr->subvalue + dsval;
			if (sv > zcdr->max_value) sv = zcdr->max_value;
		}
		else if (spin<0) {
			sv = zcdr->subvalue - dsval;
			if (sv < zcdr->min_value) sv = zcdr->min_value;
		}
		zcdr->subvalue = sv;
		value = sv / ZYNCODER_TICKS_PER_RETENT;
		zcdr->tsus=tsus;
		//printf("DTUS=%d, %d (%d)\n",dtus_avg,value,dsval);
	} 
	//Fixed Step Size
	else {
		if (spin>0) {
			value = zcdr->value + zcdr->step;
			if (value>zcdr->max_value) value=zcdr->max_value;
		}
		else if (spin<0) {
			value = zcdr->value - zcdr->step;
			if (value<zcdr->min_value) value=zcdr->min_value;
		}
	}

	if (zcdr->value!=value) {
		zcdr->value=value;
		zcdr->value_flag = 1;
		if (zcdr->zpot_i>=0) {
			send_zynpot(zcdr->zpot_i);
		}
	}
}

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

int setup_zyncoder(uint8_t i, uint16_t pin_a, uint16_t pin_b) {
	if (i>=MAX_NUM_ZYNCODERS) {
		printf("ZynCore->setup_zyncoder(%d, ...): Invalid index!\n", i);
		return 0;
	}
	zyncoder_t *zcdr = zyncoders + i;

	//setup_rangescale_zyncoder(i,0,127,64,0);
	zcdr->enabled = 0;
	zcdr->inv = 0;
	zcdr->step = 1;
	zcdr->value = 0;
	zcdr->subvalue = 0;
	zcdr->min_value = 0;
	zcdr->max_value = 127;
	zcdr->last_encoded = 0;
	zcdr->tsus = 0;

	if (pin_a!=pin_b) {
		// RBPi GPIO pins
		if (pin_a<100 && pin_b<100) {
			pinMode(pin_a, INPUT);
			pinMode(pin_b, INPUT);
			pullUpDnControl(pin_a, PUD_UP);
			pullUpDnControl(pin_b, PUD_UP);
			zcdr->pin_a = pin_a;
			zcdr->pin_b = pin_b;
			zcdr->enabled = 1;
			wiringPiISR(pin_a,INT_EDGE_BOTH, zyncoder_rbpi_ISRs[i]);
			wiringPiISR(pin_b,INT_EDGE_BOTH, zyncoder_rbpi_ISRs[i]);
			zyncoder_rbpi_ISR(i);
			return 1;
		} 
		// MCP23017 pins
		else if (pin_a>=100 && pin_b>=100) {
			#if defined(MCP23017_ENCODERS)
				uint8_t j = pin2index_zynmcp23017(pin_a);
				uint8_t k = pin2index_zynmcp23017(pin_b);
				if (j>=0 && k>=0 && j==k) {
					uint8_t bit_a = pin_a - zynmcp23017s[j].base_pin;
					uint8_t bit_b = pin_b - zynmcp23017s[k].base_pin;
					if (bit_a<16 && bit_b<16) {
						uint8_t bank_a, bank_b;
						if (bit_a<8) bank_a=0;
						else bank_a=1;
						if (bit_b<8) bank_b=0;
						else bank_b=1;
						if (bank_a == bank_b) {
							pinMode(pin_a, INPUT);
							pinMode(pin_b, INPUT);
							pullUpDnControl(pin_a, PUD_UP);
							pullUpDnControl(pin_b, PUD_UP);
							zcdr->pin_a = pin_a;
							zcdr->pin_b = pin_b;
							zcdr->enabled = 1;
							setup_pin_action_zynmcp23017(pin_a, ZYNCODER_PIN_ACTION, i);
							setup_pin_action_zynmcp23017(pin_b, ZYNCODER_PIN_ACTION, i);
							zyncoder_update_zynmcp23017(i);
							return 1;
						}
						else {
							printf("ZynCore->setup_zyncoder(%d, %d, %d): Can't configure zyncoder with pins on different banks!\n", i, pin_a, pin_b);
							return 0;
						}
					}
					else {
						printf("ZynCore->setup_zyncoder(%d, %d, %d): Pin numbers out of range!\n", i, pin_a, pin_b);
						return 0;
					}
				}
				else {
					printf("ZynCore->setup_zyncoder(%d, %d, %d): Can't configure zyncoder with pins on different MCP23017!\n", i, pin_a, pin_b);
					return 0;
				}
			#endif
		}
		else {
			printf("ZynCore->setup_zyncoder(%d, %d, %d): Can't configure zyncoder with mixed pins (RBPi & MCP230XX)!\n", i, pin_a, pin_b);
			return 0;
		}
	}
	else {
		printf("ZynCore->setup_zyncoder(%d, %d, %d): Can't configure zyncoder on a single pin!\n", i, pin_a, pin_b);
		return 0;
	}
	return 0;
}

int setup_rangescale_zyncoder(uint8_t i, int32_t min_value, int32_t max_value, int32_t value, int32_t step) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->setup_rangescale_zyncoder(%d, ...): Invalid index!\n", i);
		return 0;
	}
	if (min_value==max_value) {
		//printf("ZynCore->setup_rangescale_zyncoder(%d, %d, %d, ...): Invalid range!\n", i, min_value, max_value);
		//return 0;
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

int32_t get_value_zyncoder(uint8_t i) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->get_value_zyncoder(%d): Invalid index!\n", i);
		return 0;
	}
	zyncoders[i].value_flag = 0;
	return zyncoders[i].value;
}

uint8_t get_value_flag_zyncoder(uint8_t i) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->get_value_flag_zyncoder(%d): Invalid index!\n", i);
		return 0;
	}
	return zyncoders[i].value_flag;
}

int set_value_zyncoder(uint8_t i, int32_t v) {
	if (i>=MAX_NUM_ZYNCODERS || zyncoders[i].enabled==0) {
		printf("ZynCore->set_value_zyncoder(%d, ...): Invalid index!\n", i);
		return 0;
	}
	zyncoder_t *zcdr = zyncoders + i;

	if (zcdr->step==0) {
		v*=ZYNCODER_TICKS_PER_RETENT;
		if (v>zcdr->max_value) zcdr->subvalue=zcdr->max_value;
		else if (v<zcdr->min_value) zcdr->subvalue=zcdr->min_value;
		else zcdr->subvalue=v;
		zcdr->value=zcdr->subvalue/ZYNCODER_TICKS_PER_RETENT;
	} else {
		if (v>zcdr->max_value) zcdr->value=zcdr->max_value;
		else if (v<zcdr->min_value) zcdr->value=zcdr->max_value;
		else zcdr->value=v;
	}
	//zcdr->value_flag = 1;
	return 1;
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
