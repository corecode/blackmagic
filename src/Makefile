PROBE_HOST ?= native
PLATFORM_DIR = platforms/$(PROBE_HOST)
VPATH += $(PLATFORM_DIR)

BUILDDATE := `date +"%Y%m%d"`

CFLAGS += -Wall -Wextra -Wno-pointer-sign -Wno-char-subscripts\
	-Wno-sign-compare \
	-O2 -std=gnu99 -g3 -DBUILDDATE=\"$(BUILDDATE)\"\
	-I. -Iinclude -I$(PLATFORM_DIR) \
	-DVERSION_SUFFIX=\"`../scripts/setlocalversion`\" -MD

SRC = 	gdb_if.c	\
	gdb_packet.c	\
	gdb_main.c	\
	hex_utils.c	\
	jtagtap.c	\
	swdptap.c	\
	adiv5.c		\
	adiv5_swdp.c	\
	cortexm.c	\
	stm32f1.c	\
	nxp_tgt.c	\
	main.c		\
	platform.c	\
	command.c	\
	jtag_scan.c	\
	adiv5_jtagdp.c	\
	lmi.c		\
	arm7tdmi.c	\
	stm32f4.c	\
	crc32.c		\
	sam3x.c		\
	target.c	\

include $(PLATFORM_DIR)/Makefile.inc

OBJ = $(SRC:.c=.o)

blackmagic: $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

.PHONY:	clean host_clean

clean:	host_clean
	$(RM) *.o *.d *~ blackmagic $(HOSTFILES)

-include *.d

