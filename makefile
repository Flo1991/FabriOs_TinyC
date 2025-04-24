# ----------
#  ______    _          _    ____
# |  ____|  | |        (_)  / __ \
# | |__ __ _| |__  _ __ _  | |  | |___
# |  __/ _` | '_ \| '__| | | |  | / __|
# | | | (_| | |_) | |  | | | |__| \__ \
# |_|  \__,_|_.__/|_|  |_|  \____/|___/
# 
# Tiny C implementation for Attiny2313 
# Copyright (c) 2025, Flo1991
# BSD 3-Clause License - see LICENSE file for details
#
# Author : Florian Wank
# Date: 15.10.2024
# Brief : very basic makefile for attiny2313 FabriOs variant in very tiny C language implementation
# ----------

PROJECT_NAME := FabriOs_TinyC

LINKER_FILE := Attiny2313.ld

OUTDIR := FabriOs_TinyC

CC := avr-gcc

# MCU name
MCU := attiny2313

COMPILER_FLAGS := -mmcu=$(MCU) -O1 -g -std=gnu99 -funsigned-char -Wall -nodevicelib

DEPS := common.h

SOURCEFILES := source/appl/appl_main.c \
			   source/mcal/gpt/gpt.c \
			   source/mcal/startup/startup.c \
			   source/servl/schm/schm.c

OBJS = startup.o \
		appl_main.o \
		gpt.o \
		schm.o

OBJS_PATHS:= $(foreach item,$(OBJS),$(OUTDIR)/$(item))

FabriOs_TinyC : $(OBJS) \
				 Attiny2313.ld
	@$(CC) $(COMPILER_FLAGS) -T $(LINKER_FILE) -nostartfiles -Wl,--gc-sections,-Map,$(OUTDIR)/$(PROJECT_NAME).map -o $(OUTDIR)/$(PROJECT_NAME).elf $(OBJS_PATHS)
	$(info $0: create hex file...)
	avr-objcopy -O ihex $(OUTDIR)/$(PROJECT_NAME).elf $(OUTDIR)/$(PROJECT_NAME).hex
	$(info $0: create listing...)
	avr-objdump --source --all-headers --demangle --line-numbers --wide $(OUTDIR)/$(PROJECT_NAME).elf > $(OUTDIR)/$(PROJECT_NAME).lst
	$(info $0: size...)
	avr-size --format=berkeley $(OUTDIR)/$(PROJECT_NAME).elf
	
startup.o : source/mcal/startup/startup.c \
			source/common/common.h \
			source/common/types.h \
			source/mcal/core/core.h 
			@$(CC) -c $(COMPILER_FLAGS) source/mcal/startup/startup.c -I source  -o $(OUTDIR)/startup.o

appl_main.o : source/appl/appl_main.c \
			source/common/common.h \
			source/common/types.h \
			source/mcal/core/core.h \
			source/servl/schm/schm.h \
			source/mcal/gpt/gpt.h \
			source/appl/appl_main.h 
			@$(CC) -c $(COMPILER_FLAGS) source/appl/appl_main.c -I source  -o $(OUTDIR)/appl_main.o

gpt.o : source/mcal/gpt/gpt.c \
			source/common/common.h \
			source/common/types.h \
			source/mcal/core/core.h \
			source/mcal/gpt/gpt.h 
			@$(CC) -c $(COMPILER_FLAGS) source/mcal/gpt/gpt.c -I source  -o $(OUTDIR)/gpt.o

schm.o : source/servl/schm/schm.c \
			source/common/common.h \
			source/common/types.h \
			source/mcal/core/core.h \
			source/mcal/gpt/gpt.h \
			source/servl/schm/schm.h
			@$(CC) -c $(COMPILER_FLAGS) source/servl/schm/schm.c -I source  -o $(OUTDIR)/schm.o

FlashProg : FabriOs_TinyC
			avrdude -c jtag3isp -p t2313 -U flash:w:$(OUTDIR)/$(PROJECT_NAME).elf