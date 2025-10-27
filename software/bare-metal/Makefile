#
# Copyright Altera 2013,2014
# All Rights Reserved.
#
#ORIGINAL:QSYS_HDR := hps_0.h
#ORIGINAL:QSYS_HDR_DIR := qsys_headers

# check that user created qsys_headers output
#ORIGINAL:ifneq ($(QSYS_HDR_DIR)/$(QSYS_HDR), $(wildcard $(QSYS_HDR_DIR)/$(QSYS_HDR)))
#ORIGINAL:$(error Contents of '$(QSYS_HDR_DIR)' does not appear to exist. Did you run 'sopc-create-header-files' a#ORIGINAL:s instructed for this lab?)
#ORIGINAL:endif

SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
LOCAL_FILES = ./alt_ether

# look for SoC EDS 15.0 directory structure
ifneq (,$(wildcard $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib/include/soc_cv_av/socal))
SOCAL_ROOT := $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib/include/soc_cv_av
endif

# look for SoC EDS 14.0 directory structure
ifneq (,$(wildcard $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib/include/socal))
SOCAL_ROOT := $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib/include
endif

ifndef SOCAL_ROOT
$(error SOCAL_ROOT is undefined)
endif

#ORIGINAL:EXAMPLE_SRC := fft.c hello.c io.c 
#ORIGINAL:EXAMPLE_SRC += msgdma/sgdma_dispatcher.c
EXAMPLE_SRC := main.c io.c $(wildcard $(LOCAL_FILES)/*.c)

C_SRC       := $(EXAMPLE_SRC)

LINKER_SCRIPT := cycloneV-dk-ram-modified.ld

MULTILIBFLAGS := -mcpu=cortex-a9 -mfloat-abi=softfp -mfpu=neon 
CFLAGS  := -g -O0 -Wall -Werror -std=c99 $(MULTILIBFLAGS) -I$(HWLIBS_ROOT)/include -I. -Imsgdma -Iqsys_headers -I$(SOCAL_ROOT) -Dsoc_cv_av -I$(LOCAL_FILES)
CFLAGS += -DPRINTF_HOST
LDFLAGS := -T$(LINKER_SCRIPT) $(MULTILIBFLAGS)

CROSS_COMPILE := arm-altera-eabi-
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)g++
NM := $(CROSS_COMPILE)nm
OC := $(CROSS_COMPILE)objcopy
OD := $(CROSS_COMPILE)objdump
RM := rm -rf
CP := cp -f

ELF ?= $(basename $(firstword $(C_SRC))).axf
OBJ := $(patsubst %.c,%.o,$(C_SRC))
BIN = $(basename $(firstword $(C_SRC))).bin
IMG = $(basename $(BIN)).img

.PHONY: all
all: $(BIN)

.PHONY: clean
clean:
	$(RM) $(ELF) $(OBJ) $(BIN)
	$(RM) *.map
	$(RM) *.objdump
	$(RM) *.img
	$(RM) $(LOCAL_FILES)/*.o

%.c: $(HWLIBS_ROOT)/src/hwmgr/%.c $(LOCAL_FILES)/%.c $(HWLIBS_ROOT)/src/utils/%.c
	$(CP) $< $@

$(OBJ): %.o: %.c Makefile $(QSYS_HDR_DIR)/$(QSYS_HDR) $(LOCAL_FILES)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) -o $@
	$(NM) $@ > $@.map
	$(OD) -d $@ > $@.objdump

$(BIN): $(ELF)
	$(OC) -O binary $(ELF) $(BIN)

.PHONY: preloader
preloader: $(BIN)
	mkimage -A arm -O u-boot -T firmware -C none -a 0x00100040 -e 0x00100040 -n 'bare metal app' -d $(BIN) $(IMG)

.PHONY: flash
flash:
	sudo dd if=../spl_bsp/preloader-mkpimage.bin of=/dev/sdb3
	sudo dd if=main.img of=/dev/sdb3 bs=262144 seek=1
	sync
	sudo eject /dev/sdb
	


