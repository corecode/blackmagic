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

/* Quick hack for bit-banging SW-DP interface over FT2232.
 * Intended as proof of concept, not for production.
 */

#include <stdio.h>
#include <assert.h>
#include <ftdi.h>

#include "platform.h"
#include "swdptap.h"

static void swdptap_init_internal(void);
static void swdptap_turnaround(uint8_t dir);

int swdptap_init(void)
{
	assert(ftdic != NULL);

	swdptap_init_internal();

	/* This must be investigated in more detail.
	 * As described in STM32 Reference Manual... */
	/* swdptap_seq_out(0xFFFF, 16);  */
	swdptap_reset();
	swdptap_seq_out(0xE79E, 16); /* 0b0111100111100111 */ 
	/* swdptap_seq_out(0xffffffff, 32); */
	/* swdptap_seq_out(0xfffff, 24); */
	swdptap_reset();
	platform_buffer_flush();

	return 0;
}

void swdptap_reset(void)
{
        swdptap_turnaround(0);
        /* 50 clocks with TMS high */
	swdptap_seq_out(0xffffffff, 32);
	swdptap_seq_out(0x000fffff, 24);
}

#ifdef PROBE_MPSSE
/**
 * This code is for use with a KT-Link adapter, or a Bus Blaster with
 * KT-Link buffer.
 */

static void swdptap_seq_out_internal(uint32_t MS, int ticks);

const uint16_t swdptap_swdoe = 0x1000;
static uint16_t swdptap_outputs = 0xac02;
static uint16_t swdptap_direction = 0xff2b;

static void swdptap_set_bits(void)
{
	uint8_t cmd[] = {
		SET_BITS_LOW, swdptap_outputs & 0xff, swdptap_direction & 0xff,
		SET_BITS_HIGH, (swdptap_outputs >> 8) & 0xff, (swdptap_direction >> 8) & 0xff
	};
	platform_buffer_write(cmd, sizeof(cmd));
}

static void swdptap_init_internal(void)
{
	int err;

	if((err = ftdi_set_bitmode(ftdic, 0, BITMODE_RESET)) ||
	   (err = ftdi_set_bitmode(ftdic, 0, BITMODE_MPSSE))) {
		fprintf(stderr, "ftdi_set_bitmode: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}

	uint8_t setup[] = {
		DIS_DIV_5,
		TCK_DIVISOR, 0, 1,
		LOOPBACK_END
	};
	platform_buffer_write(setup, sizeof(setup));
	swdptap_set_bits();
	platform_buffer_flush();
}

static void swdptap_turnaround(uint8_t in)
{
	const int trn = 1;

	static uint8_t olddir = 0;

	if (in == olddir)
		return;
	olddir = in;

	/* DEBUG("turning %s\n", in ? "in" : "out"); */

	/* swdoe is active low */
	if (in)
		swdptap_outputs |= swdptap_swdoe;
	else
		swdptap_outputs &= ~swdptap_swdoe;

	if (in)
		swdptap_set_bits();

	swdptap_seq_out_internal(0, trn);	/* we're masked.  only for clk. */

	if (!in)
		swdptap_set_bits();
}

uint32_t swdptap_seq_in(int ticks)
{
	uint8_t buf[] = {
		MPSSE_DO_READ | MPSSE_LSB,
		0,		/* len - 1 low */
		0		/* len - 1 high (always 0) */
	};

	assert(ticks <= 32);

	/* DEBUG("seq_in %d\n", ticks); */

	swdptap_turnaround(1);

	if (ticks >= 8) {
		buf[1] = ticks / 8 - 1;
		platform_buffer_write(buf, sizeof(buf));
	}

	if (ticks % 8) {
		buf[0] |= MPSSE_BITMODE;
		buf[1] = ticks % 8 - 1;
		platform_buffer_write(buf, 2);
	}

	uint8_t res[4] = {0};	/* max 32 bits */
	platform_buffer_read(res, (ticks + 7) / 8);

	uint32_t ret = 0;
	for (int msb = ticks / 8 - 1; msb >= 0; --msb)
		ret = (ret << 8) | res[msb];

	int bits_remaining = ticks % 8;
	ret |= (res[ticks / 8] >> /* remaining byte */
		(8 - bits_remaining)) /* bits come in from MSB */
		<< (ticks - bits_remaining); /* move them to the top */

	/* DEBUG("  read: %08x\n", ret); */

	return (ret);
}

static int swdptap_parity(uint32_t data)
{
	data ^= data >> 16;
	data ^= data >> 8;
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return (data & 1);
}

uint8_t swdptap_seq_in_parity(uint32_t *ret, int ticks)
{
	uint32_t result = swdptap_seq_in(ticks);
	int in_parity = swdptap_seq_in(1);

	*ret = result;
	return (in_parity ^ swdptap_parity(result));
}

static void swdptap_seq_out_internal(uint32_t MS, int ticks)
{
	uint8_t buf[] = {
		MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_LSB,
		0,		/* len - 1 low */
		0,		/* len - 1 high (always 0) */
		MS & 0xff,
		(MS >> 8) & 0xff,
		(MS >> 16) & 0xff,
		(MS >> 24) & 0xff,
	};

	assert(ticks <= 32);

	/* DEBUG("seq_out %d, %08x\n", ticks, MS); */

	if (ticks >= 8) {
		buf[1] = ticks / 8 - 1;
		platform_buffer_write(buf, ticks / 8 + 3);
	}

	if (ticks % 8) {
		buf[0] |= MPSSE_BITMODE;
		buf[1] = ticks % 8 - 1;
		buf[2] = (MS >> (ticks / 8 * 8)) & 0xff;
		platform_buffer_write(buf, 3);
	}
}

void swdptap_seq_out(uint32_t MS, int ticks)
{
	swdptap_turnaround(0);
	swdptap_seq_out_internal(MS, ticks);
}

void swdptap_seq_out_parity(uint32_t MS, int ticks)
{
	swdptap_seq_out(MS, ticks);

	uint32_t bitmask = (ticks == 32 ? 0 : (1 << ticks)) - 1;
	uint32_t parity = swdptap_parity(MS & bitmask);
	swdptap_seq_out(parity, 1);
	platform_buffer_flush();
}

#else

static uint8_t swdptap_bit_in(void);
static void swdptap_bit_out(uint8_t val);

static void swdptap_init_internal(void)
{
	if((err = ftdi_set_bitmode(ftdic, 0xAB, BITMODE_BITBANG)) != 0) {
		fprintf(stderr, "ftdi_set_bitmode: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}

	assert(ftdi_write_data(ftdic, "\xAB\xA8", 2) == 2);
}

static void swdptap_turnaround(uint8_t dir)
{
	static uint8_t olddir = 0;

        //DEBUG("%s", dir ? "\n-> ":"\n<- ");
	platform_buffer_flush();

	if(dir == olddir) return;
	olddir = dir;

	if(dir)	/* SWDIO goes to input */
		assert(ftdi_set_bitmode(ftdic, 0xA3, BITMODE_BITBANG) == 0);

	/* One clock cycle */
	ftdi_write_data(ftdic, "\xAB\xA8", 2);

	if(!dir) /* SWDIO goes to output */
		assert(ftdi_set_bitmode(ftdic, 0xAB, BITMODE_BITBANG) == 0);
}

static uint8_t swdptap_bit_in(void) 
{
	uint8_t ret;

	//ftdi_read_data(ftdic, &ret, 1);
	ftdi_read_pins(ftdic, &ret);
	ret &= 0x08;
	ftdi_write_data(ftdic, "\xA1\xA0", 2);

	//DEBUG("%d", ret?1:0);

	return ret;
}

static void swdptap_bit_out(uint8_t val)
{
	uint8_t buf[3] = "\xA0\xA1\xA0";

	//DEBUG("%d", val);

	if(val) {
		for(int i = 0; i < 3; i++)
			buf[i] |= 0x08;
	}
	//ftdi_write_data(ftdic, buf, 3);
	platform_buffer_write(buf, 3);
}

uint32_t swdptap_seq_in(int ticks)
{
	uint32_t index = 1;
	uint32_t ret = 0;

	swdptap_turnaround(1);

	while (ticks--) {
		if (swdptap_bit_in())
			ret |= index;
		index <<= 1;
	}

	return ret;
}

uint8_t swdptap_seq_in_parity(uint32_t *ret, int ticks)
{
	uint32_t index = 1;
	uint8_t parity = 0;
	*ret = 0;

	swdptap_turnaround(1);

	while (ticks--) {
		if (swdptap_bit_in()) {
			*ret |= index;
			parity ^= 1;
		}
		index <<= 1;
	}
	if (swdptap_bit_in())
		parity ^= 1;

	return parity;
}

void swdptap_seq_out(uint32_t MS, int ticks)
{
	swdptap_turnaround(0);

	while (ticks--) {
		swdptap_bit_out(MS & 1);
		MS >>= 1;
	}
}

void swdptap_seq_out_parity(uint32_t MS, int ticks)
{
	uint8_t parity = 0;

	swdptap_turnaround(0);

	while (ticks--) {
		swdptap_bit_out(MS & 1);
		parity ^= MS;
		MS >>= 1;
	}
	swdptap_bit_out(parity & 1);
}
#endif
