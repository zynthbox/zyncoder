/*
 * ******************************************************************
 * ZYNTHIAN PROJECT: Zyncoder Library Tests
 * 
 * Library for interfacing Rotary Encoders & Switches connected 
 * to RBPi native GPIOs or expanded with MCP23008. Includes an 
 * emulator mode to ease developping.
 * 
 * Copyright (C) 2015-2016 Fernando Moyano <jofemodo@zynthian.org>
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
#include <unistd.h>

#include "zyncoder.h"

#ifndef HAVE_WIRINGPI_LIB
#define NUM_SWITCHES 4
unsigned int zyncoder_pin_a[4]={4,5,6,7};
unsigned int zyncoder_pin_b[4]={8,9,10,11};
unsigned int zynswitch_pin[NUM_SWITCHES]={0,1,2,3};
#else

#ifdef MCP23017_ENCODERS
#define NUM_SWITCHES 8
unsigned int zyncoder_pin_a[4] = { 102, 105, 110, 113 };
unsigned int zyncoder_pin_b[4] = { 101, 104, 109, 112 };
unsigned int zynswitch_pin[NUM_SWITCHES]  = { 100, 103, 108, 111, 106, 107, 114, 115 };
#else
#define NUM_SWITCHES 4
//PROTOTYPE-3
//unsigned int zyncoder_pin_a[4]={27,21,3,7};
//unsigned int zyncoder_pin_b[4]={25,26,4,0};
//PROTOTYPE-4
unsigned int zyncoder_pin_a[4]={26,25,0,4};
unsigned int zyncoder_pin_b[4]={21,27,7,3};
unsigned int zynswitch_pin[NUM_SWITCHES]={107,23,106,2};
#endif

#endif

int main() {
	int i;

	printf("Starting ZynCore...\n");
	init_zyncontrol();
	init_zynmidirouter();

	#ifdef DEBUG
	if (zynpots[0].type==ZYNPOT_RV112) {
		fprintf(stdout, "Range 25 = %d\n", RV112_ADS1115_RANGE_25);
		fprintf(stdout, "Range 50 = %d\n", RV112_ADS1115_RANGE_50);
		fprintf(stdout, "Range 75 = %d\n", RV112_ADS1115_RANGE_75);
		fprintf(stdout, "Range 100 = %d\n", RV112_ADS1115_RANGE_100);
	}
	#endif

	int last_zynswitch_index = get_last_zynswitch_index();
	int num_zynswitches = get_num_zynswitches();
	int num_zynpots = get_num_zynpots();

	//Configure zynpots
	for (i=0; i<num_zynpots; i++) {
		setup_rangescale_zynpot(i, 0, 100, 50, 0);
	}
	//setup_rangescale_zynpot(0, 0, 100, 50, 1);

	printf("Testing switches & rotaries...\n");
	while(1) {
		i=0;
		while (i <= last_zynswitch_index) {
			int dtus = get_zynswitch(i, 2000000);
			if (dtus>0) fprintf(stdout, "SW-%d = %d\n", i, dtus);
			i++;
		}
		for (i=0;i<num_zynpots;i++) {
			if (get_value_flag_zynpot(i)) {
				printf("PT-%d = %d\n", i, get_value_zynpot(i));
			}
		}
		usleep(5000);
	}

	return 0;
}
