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
#include "platform.h"
#include "gdb_if.h"
#include "jtag_scan.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>

struct ftdi_context *ftdic;

#define BUF_SIZE 4096
static uint8_t outbuf[BUF_SIZE];
static uint16_t bufptr = 0;

int platform_init(void) 
{ 
	int err;

	if(ftdic) {
		ftdi_usb_close(ftdic);
		ftdi_free(ftdic);
		ftdic = NULL;
	}
	if((ftdic = ftdi_new()) == NULL) {
		fprintf(stderr, "ftdi_new: %s\n", 
			ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_set_interface(ftdic, INTERFACE_A)) != 0) {
		fprintf(stderr, "ftdi_set_interface: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_usb_open(ftdic, FT2232_VID, FT2232_PID)) != 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}

	if((err = ftdi_usb_reset(ftdic)) != 0) {
		fprintf(stderr, "ftdi_usb_reset: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_set_latency_timer(ftdic, 1)) != 0) {
		fprintf(stderr, "ftdi_set_latency_timer: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_set_baudrate(ftdic, 1000000)) != 0) {
		fprintf(stderr, "ftdi_set_baudrate: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_usb_purge_buffers(ftdic)) != 0) {
		fprintf(stderr, "ftdi_set_baudrate: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	if((err = ftdi_write_data_set_chunksize(ftdic, BUF_SIZE)) != 0) {
		fprintf(stderr, "ftdi_write_data_set_chunksize: %d: %s\n", 
			err, ftdi_get_error_string(ftdic));
		abort();
	}

	assert(gdb_if_init() == 0);

	jtag_scan(NULL);
	
	return 0; 
}

void platform_buffer_flush(void)
{
	if (bufptr == 0)
		return;

	assert(ftdi_write_data(ftdic, outbuf, bufptr) == bufptr);
//	printf("FT2232 platform_buffer flush: %d bytes\n", bufptr);
	bufptr = 0;
}

int platform_buffer_write(const uint8_t *data, int size)
{
	if((bufptr + size) / BUF_SIZE > 0) platform_buffer_flush();
	memcpy(outbuf + bufptr, data, size);
	bufptr += size;
	return size;
}

int platform_buffer_read(uint8_t *data, int size)
{
	int index = 0;
	platform_buffer_flush();
	while (index < size) {
		int res = ftdi_read_data(ftdic, data + index, size-index);
		assert(res >= 0);
		index += res;
		if (res == 0)
			usleep(1000); /* XXX hack */
	}
	return size;
}

#ifdef WIN32
#warning "This vasprintf() is dubious!"
int vasprintf(char **strp, const char *fmt, va_list ap)
{
	int size = 128, ret = 0;

	*strp = malloc(size);
	while(*strp && ((ret = vsnprintf(*strp, size, fmt, ap)) == size)) 
		*strp = realloc(*strp, size <<= 1);
	
	return ret;
}
#endif

const char *platform_target_voltage(void)
{
	return "not supported";
}

void platform_delay(uint32_t delay)
{
	usleep(delay * 100000);
}

