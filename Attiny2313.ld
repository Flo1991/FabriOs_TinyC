/*********************************************************************************************************************/
/**
 *  ______    _          _    ____
 * |  ____|  | |        (_)  / __ \
 * | |__ __ _| |__  _ __ _  | |  | |___
 * |  __/ _` | '_ \| '__| | | |  | / __|
 * | | | (_| | |_) | |  | | | |__| \__ \
 * |_|  \__,_|_.__/|_|  |_|  \____/|___/
 *
 * Tiny C implementation for Attiny2313
 *
 * Copyright (c) 2025, Flo1991
 * BSD 3-Clause License - see LICENSE file for details
 *
 * @file Attiny2313.ld
 * @date 08.03.2025
 * @author Florian Wank
 * @brief Linker script file for Attiny2313 Mcu
 *
 * This is the linker script file that is responsible to link the compiled source code together.
 * It defines all available memory in the target an the different sections which are necessary for
 * the application. 
 */
/*********************************************************************************************************************/

/* define the entry point for mcu startup */
ENTRY(__vector_Application_ResetHandler)

/* define memory map here; note that the address bus is 16 bit, so the SRAM, EEPROM, FUSE areas are virtual memory
   addresses that are used for debugging; the physical start of sram is 0x60 and 0 of flash, but different core
   instructions are used for accessing these locations! */
MEMORY
{
   FLASH(rx)       : ORIGIN = 0x0000000, LENGTH = 2K
   SRAM(rw!x)      : ORIGIN = 0x0800060, LENGTH = 128
   EEPROM (rw!x)   : ORIGIN = 0x0810000, LENGTH = 128
   FUSE(rw!x)      : ORIGIN = 0x0820000, LENGTH = 1K
}

SECTIONS
{
   /* the text section is read only, so the load memory address (from flash, short LMA) is the same as the 
      virtual memory address (VMA), which is used at runtime to access data (also read from flash) */
   .text : 
   {   
      /* get location counter of start of flash memory */  
      _start_of_flash = .;  
      _stext = .;
      /* get current location counter "." and store to _isr_start; */	  
      _isr_start = .;
      /* * = wild-card */
      /* locate the isr vector table directly at memory start; never remove this data! */
      KEEP(*(.isr_vector))
      
      /* Align the location counter, so that the next memory area is also aligned to 2 bytes = 16 bit */
      . = ALIGN(2);

      /* create a list of the used interrupt vector here to realize C code startup implementation */
      _isr_reset_handler_start = .;
      KEEP(*(.isr_vector_reset_handler))
      /* calculate the command that is put in the vector table entry in C : the avr core architecture 
         requires a rjmp instruction, which is 0xCXXX where X is the address to jump to; need to 
         divide by two, because addressing is word wise; subtract 1 for final location 
         (PC <- PC + k + 1, correct + 1 offset from core instruction) */
      _vtable_entry_reset_handler = _isr_reset_handler_start / 2 - 1 + 0xC000;

      /* Align the location counter, so that the next memory area is also aligned to 2 bytes = 16 bit */
      . = ALIGN(2);
      
      /* Text section */
      /* this expression matches all .text sections in all input files */
      *(.text)
      /* this expression matches all .text* sections in all input files, so also match .text.mysection (need for -ffunction-sections) */
      *(.text*)
      
      /* Read only data section */
      *(.rodata)
      
      /* Align the location counter, so that the next memory area is also aligned to 2 bytes = 16 bit */
      . = ALIGN(2);  
       
      /* Location counter has here the end of the text region; create symbol here */
      _etext = .;
   }> FLASH


   
   /* the data section has a load memory address (LMA) in flash (here the const data is stored), but a
      virtual memory address (VMA) in SRAM, from which the data is accessed at runtime; therefore we 
      use ">SRAM AT>FLASH"; the _sdata and _edata are refered to VMA */
   .data :
   {
      /* Start of Data section on SRAM; create symbol here -> VMA */
      _sdata = .;
      *(.data)
      *(.data*)
      *(.ramtext)
      /* Align to next 16bit multiple at the End of this section */
      . = ALIGN(2);
      /* locate ram functions aligned to 16 bit */
      *(.code_ram)
      /* Align to next 16bit multiple at the End of this section */
      . = ALIGN(2);
      /* End of Data section on SRAM; create symbol here -> VMA */
      _edata = .;
   } > SRAM AT>FLASH
   
   /* get the size of the .data section */
   _data_size = _edata - _sdata;
   /* get the load address (LMA) of the .data section */
   _data_loadaddr = LOADADDR(.data);
   
	
   .bss :
   {
      /* Start of Uninitialized Data section on SRAM; create symbol here -> VMA */
      _sbss = .;
      *(.bss)
      *(.bss*)
      *(COMMON)
      /* Align to next 16bit multiple at the End of this section */
      . = ALIGN(4);
      /* End of Uninitialized Data section on SRAM; create symbol here -> VMA */
      _ebss = .;
   } > SRAM AT>FLASH   
   
   /* get the size of the .bss section */
   _bss_size = _ebss - _sbss;
   
   /* use all unused SRAM as stack */
   .stack :
   {
     /* one should always align the stackpointer to 2 bytes */
     . = ALIGN(2);
	 /* end of stack; stack is predecrementing */
     _estack = .;
     *(.stack)     
   } > SRAM AT>FLASH
   
   /* End of SRAM is start of stack, because stackpointer of Mcu is implemented by postdecrement */
   _sstack = 0x8000E0 - 1;
   /* get the size of the .stack section */
   _stack_size = _sstack - _estack;

   .eeprom  :
  {
    /* See .data above...  */
    KEEP(*(.eeprom*))
     __eeprom_end = . ;
  }  > EEPROM

  .fuse  :
  {
    KEEP(*(.fuse))
    KEEP(*(.lfuse))
    KEEP(*(.hfuse))
    KEEP(*(.efuse))
  }  > FUSE
   
}