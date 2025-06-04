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

# set compiler to use
CC := avr-gcc

# MCU name
MCU := attiny2313

# Setup my compiler flags; must be O1 for this project to work correctly
COMPILER_FLAGS := -mmcu=$(MCU) -O1 -g -std=gnu99 -funsigned-char -Wall -nodevicelib

# Setup the virtual paths; needed for c file compilation to object files
# list all paths to sources here
VPATH := source/appl \
		source/mcal/gpt \
		source/mcal/startup \
		source/servl/schm \
		source/common \

# automatically find all header files and create a list with full dir
# Example: ./source/common/common.h 
DEPS := $(shell find $(SOURCEDIR) -name '*.h')
# Comment the following line in to print the generated list
#$(info DEPS is $(DEPS))

# Create object file names automatically from found c files; so search all c files in 
# directory (and subdirs) named "source", and create matching object output files in
# style e.g. FabriOs_TinyC/startup.o , where FabriOs_TinyC must match OUTDIR variable in this
# makefile!
# Example list that is generated
# OBJS :=  $(OUTDIR)/startup.o \
#		$(OUTDIR)/appl_main.o \
#		$(OUTDIR)/gpt.o \
#		$(OUTDIR)/schm.o
OBJS := $(shell ls source/ -R | grep .c$$ | sed -e 's/\.c/\.o/g' | sed -e 's/^/FabriOs_TinyC\//')
# Comment the following line in to print the generated list
#$(info OBJS is $(OBJS))

FabriOs_TinyC : $(OBJS) \
				 Attiny2313.ld
	@$(CC) $(COMPILER_FLAGS) -T $(LINKER_FILE) -nostartfiles -Wl,--gc-sections,-Map,$(OUTDIR)/$(PROJECT_NAME).map -o $(OUTDIR)/$(PROJECT_NAME).elf $(OBJS)
	$(info $0: create hex file...)
	avr-objcopy -O ihex $(OUTDIR)/$(PROJECT_NAME).elf $(OUTDIR)/$(PROJECT_NAME).hex
	$(info $0: create listing...)
	avr-objdump --source --all-headers --demangle --line-numbers --wide $(OUTDIR)/$(PROJECT_NAME).elf > $(OUTDIR)/$(PROJECT_NAME).lst
	$(info $0: size...)
	avr-size --format=berkeley $(OUTDIR)/$(PROJECT_NAME).elf

# Compile all sources to object files; the object files will be located as flat list to $(OUTDIR)
$(OUTDIR)/%.o: %.c $(DEPS)
		$(CC) $(COMPILER_FLAGS) -c $< -I source -o $@ 

# flash the program to the target
FlashProg : FabriOs_TinyC
			avrdude -c jtag3isp -p t2313 -U flash:w:$(OUTDIR)/$(PROJECT_NAME).elf