/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the SW-DP specific functions of the 
 * ARM Debug Interface v5 Architecure Specification, ARM doc IHI0031A.
 */

#include "general.h"
#include "platform.h"
#include "adiv5.h"

#include "swdptap.h"

#include "command.h"

#include <stdlib.h>

#define SWDP_ACK_OK    0x01
#define SWDP_ACK_WAIT  0x02
#define SWDP_ACK_FAULT 0x04

static void adiv5_swdp_write(ADIv5_DP_t *dp, uint8_t addr, uint32_t value);
static uint32_t adiv5_swdp_read(ADIv5_DP_t *dp, uint8_t addr);

static uint32_t adiv5_swdp_error(ADIv5_DP_t *dp);

static uint32_t adiv5_swdp_low_access(ADIv5_DP_t *dp, uint8_t APnDP, uint8_t RnW, 
				      uint8_t addr, uint32_t value);


int adiv5_swdp_scan(void)
{
	uint8_t ack;

	target_list_free();
	ADIv5_DP_t *dp = (void*)calloc(1, sizeof(*dp));

	swdptap_init();
	/* Read the SW-DP IDCODE register to syncronise */
	/* This could be done with adiv_swdp_low_access(), but this doesn't
	 * allow the ack to be checked here. */
	swdptap_seq_out(0xA5, 8);
	ack = swdptap_seq_in(3);
	if((ack != SWDP_ACK_OK) || swdptap_seq_in_parity(&dp->idcode, 32)) {
		DEBUG("invalid id/parity: ack %d, id %08x\n", ack, dp->idcode);
		morse("NO TARGETS.", 1);
		free(dp);
		return -1;
	}
	DEBUG("SWD init success, id %08x\n", dp->idcode);

	dp->dp_write = adiv5_swdp_write;
	dp->dp_read = adiv5_swdp_read;
	dp->error = adiv5_swdp_error;
	dp->low_access = adiv5_swdp_low_access;

	/* Clear errors */
	adiv5_swdp_write(dp, ADIV5_DP_ABORT,
			 ADIV5_DP_ABORT_ORUNERRCLR |
			 ADIV5_DP_ABORT_STKCMPCLR |
			 ADIV5_DP_ABORT_STKERRCLR |
			 ADIV5_DP_ABORT_WDERRCLR);
	adiv5_swdp_write(dp, ADIV5_DP_SELECT, 0); /* select CTRLSTAT */
	adiv5_swdp_write(dp, ADIV5_DP_CTRLSTAT, 0); /* clrear ORUNDETECT */

	adiv5_dp_init(dp);

	if(!target_list) morse("NO TARGETS.", 1);
	else morse(NULL, 0);

	return target_list?1:0;
}

static void adiv5_swdp_write(ADIv5_DP_t *dp, uint8_t addr, uint32_t value)
{
	adiv5_swdp_low_access(dp, ADIV5_LOW_DP, ADIV5_LOW_WRITE, addr, value);
}

static uint32_t adiv5_swdp_read(ADIv5_DP_t *dp, uint8_t addr)
{
	return adiv5_swdp_low_access(dp, ADIV5_LOW_DP, ADIV5_LOW_READ, addr, 0);
}

static uint32_t adiv5_swdp_error(ADIv5_DP_t *dp)
{
	uint32_t err, clr = 0;

	err = adiv5_swdp_read(dp, ADIV5_DP_CTRLSTAT) & 
		(ADIV5_DP_CTRLSTAT_STICKYORUN | ADIV5_DP_CTRLSTAT_STICKYCMP |
		 ADIV5_DP_CTRLSTAT_STICKYERR | ADIV5_DP_CTRLSTAT_WDATAERR);

	if(err & ADIV5_DP_CTRLSTAT_STICKYORUN) 
		clr |= ADIV5_DP_ABORT_ORUNERRCLR;
	if(err & ADIV5_DP_CTRLSTAT_STICKYCMP) 
		clr |= ADIV5_DP_ABORT_STKCMPCLR;
	if(err & ADIV5_DP_CTRLSTAT_STICKYERR) 
		clr |= ADIV5_DP_ABORT_STKERRCLR;
	if(err & ADIV5_DP_CTRLSTAT_WDATAERR)
		clr |= ADIV5_DP_ABORT_WDERRCLR;

	adiv5_swdp_write(dp, ADIV5_DP_ABORT, clr);
	dp->fault = 0;

	return err;
}

static uint32_t adiv5_swdp_low_access(ADIv5_DP_t *dp, uint8_t APnDP, uint8_t RnW, 
				      uint8_t addr, uint32_t value)
{
	uint8_t request = 0x81;
	uint32_t response;
	uint8_t ack;
	int retry = 3;

	if(APnDP && dp->fault) return 0;

	if(APnDP) request ^= 0x22;
	if(RnW)   request ^= 0x24;

	addr &= 0xC;
	request |= (addr << 1) & 0x18;
	if((addr == 4) || (addr == 8))
		request ^= 0x20;

retry:
	for (;;) {
		swdptap_seq_out(request, 8);
		ack = swdptap_seq_in(3);

		const char * const nbl[] =
			{ "0000", "0001", "0010", "0011", "0100", "0101", "0110", "0111",
			  "1000", "1001", "1010", "1011", "1100", "1101", "1110", "1111" };
		DEBUG("req %s%s %s %s %d ack %d",
		      nbl[request >> 4], nbl[request & 0xf],
		      APnDP ? "AP" : "DP", RnW ? "R" : "W", addr >> 2, ack);
		if (ack != SWDP_ACK_WAIT)
			break;
		DEBUG(" - got WAIT, flushing output\n");
		swdptap_seq_out(0, 8); /* flush */
	}

	if(ack == SWDP_ACK_FAULT) {
		DEBUG(" - FAULT\n");
		dp->fault = 1;
		return 0;
	}

	if(ack != SWDP_ACK_OK) {
		if (retry > 0) {
			/* try to recover */
			DEBUG(" - damaged ACK, trying to recover framing\n");
			retry--;
			swdptap_reset();
			swdptap_seq_out(0xa5, 8);
			ack = swdptap_seq_in(3);
			if (ack == SWDP_ACK_OK && swdptap_seq_in_parity(&response, 32) == 0) {
				swdptap_seq_out(0, 8);
				platform_buffer_flush();
				DEBUG("recovery successful\n");
				/* int err = adiv5_swdp_error(dp); */
				/* DEBUG("recovery successful, error flags %08x\n", err); */
				/* if (err) { */
				/* 	dp->fault = 1; */
				/* 	return 0; */
				/* } */
				goto retry;
			}
		}
		/* Fatal error if invalid ACK response */
		PLATFORM_FATAL_ERROR(1);
	}

	if(RnW) {
		if(swdptap_seq_in_parity(&response, 32))  /* Give up on parity error */
			PLATFORM_FATAL_ERROR(1);
		DEBUG(" in:  %08x\n", response);
	} else {
		DEBUG(" out: %08x\n", value);
		swdptap_seq_out_parity(value, 32);
	}

	/* REMOVE THIS */
	swdptap_seq_out(0, 8);
	platform_buffer_flush();
	/* DEBUG("\n"); */

	return response;
}

