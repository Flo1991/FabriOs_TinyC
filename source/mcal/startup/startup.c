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
 * @file : startup.c
 * @author : Florian Wank
 * @date : 23.04.2025
 *
 *
 * @brief  Startup file for FabriOs Tiny C port to attiny2313
 *
 * @details
 * This file contains the startup code for the mcu to get it run.
 * The target mcu attiny2313 has an avr25 core with a 8 bit stack pointer.
 * This implementation is without usage of any avr libs - so bare metal startup. With some linker script
 * magic it is possible to write the startup file also in C. The linker script must handle the correct interrupt
 * vector table generation : from the given entry address we must calculate the needed core instruction which is
 * than put in a vector table array. It is estimated that the clock is set to 8 MHz using a matching fuse configuration.
 * In order to setup the memory correctly one must know the architecture of the avr :
 * - the flash memory is located at start address 0, but must be accessed with special core instructions
 *   (pgm read -> lpm instruction)
 * - the ram memory starts from address 0x60 and can be directly accessed (so handled by compiler)
 * The given virtual memory addresses in the linker script must not be changed!
 * During startup the init implementation in the reset vector does:
 * (1) Setup the stack pointer (8bit for attiny2313)
 * (2) Init the registers all to 0 (the hardware does not reset the core registers at startup)
 * (3) Init the sram (set bss to 0 and data to corresponding data from flash)
 * (4) -> call main function
 *
 * @note Dev commands
 * Start simulator: simavr -g -m attiny2313 Project_Blinky.elf
 * Attach Debugging interface: ddd Project_Blinky.elf --debugger avr-gdb --eval-command="target remote localhost:1234"
 *
 */
/*********************************************************************************************************************/

/*********************************************************************************************************************
 * Module Includes
 **********************************************************************************************************************/
#include "common/common.h"
#include "common/types.h"
#include "mcal/core/core.h"

/* appl_main is the application entry point at which all memories have been setup by the Startup code;
 * make this function available so that it can be called after init from this file */
extern void appl_main(void);

/* extern symbols from linker script are always pointer like, so they must
 * always be accessed using "&" in front of the variable - the reason is that the symbols defined in the
 * linker script will create an entry in the symbol table with the name; the symbol table contains addresses
 * that are pointing to the values which are related to that name; so if one codes "int a = 5", the symbol
 * table will contain an entry of a with an address that points to a destination that contains value 5 - for
 * linker script symbols no memory containing a value is reserved, so only a symbol in the symbol table is
 * generated, which actually is an address; so in order to access the symbol table entry, always use the "&" */

/** linker script defined variable at which the data section starts; contains the virtual memory address, so
 * where the data has to start in SRAM; to be loaded from symbol table only, so must use "&" ;
 * the mcu uses 16 bit addressing - the vma may be larger, due to debugging purposes */
extern u16 _sdata;

/** linker script defined variable which represents the size of the data section; is same for virtual
 * memory area and for load memory area; to be loaded from symbol table only, so must use "&" ;
 * the mcu uses 16 bit addressing - the vma may be larger, due to debugging purposes */
extern u16 _data_size;

/** linker script defined variable which represents the load address for the data section;
 *  to be loaded from symbol table only, so must use "&" ;
 *  the mcu uses 16 bit addressing - the vma may be larger, due to debugging purposes */
extern u16 _data_loadaddr;

/** linker script defined variable at which the bss section starts; contains the virtual memory address, so
 * where the bss has to start in SRAM; to be loaded from symbol table only, so must use "&" ;
 * the mcu uses 16 bit addressing - the vma may be larger, due to debugging purposes */
extern u16 _sbss;

/** linker script defined variable which represents the size of the bss section;
 *  to be loaded from symbol table only, so must use "&";
 *  the mcu uses 16 bit addressing - the vma may be larger, due to debugging purposes */
extern u16 _bss_size;

/** symbol created by the linker script which represents the entry for the vector table for the reset handler */
extern u16 _vtable_entry_reset_handler;

/**********************************************************************************************************************
 * Module Definitions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Constants
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Enumerations
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Variables
 **********************************************************************************************************************/

#if defined(SW_TEST) && (SW_TEST != 0)
/* for software testing the vector table cannot be created, because at
 * compile time it is not know where the functions to be executed will be
 * located, because never access physical addresses on normal computer */
#else
/* the symbol(s) used here are from linker script; the core requires a rjmp instruction with the
  destination address to the interrupt service function; we get the address by creating a section per
  interrupt vector (--> note that this requires no additional memory, this are just linker instructions).
  In the linker script modify the target address so that we get a valid rjmp instruction:
  The rjmp instruction is 0xCXXX where X is the target address, but the X value is half of the actual address.
  Means: 0xC001 will relative jump by 0x02 (--> because core instructions have two byte size!);
  currently implemented this behavior only for the reset vector, the rest of the interrupt vector area
  (have 19 vectors, each 16 bit size) are used for normal code. This is possible if no other interrupt is used!
  --> saves 36 bytes flash
  --> be aware not to enable an interrupt for which no vector is defined! There must be no empty entries in the
      vector list inbetween two vectors (means means if want to use reset vector and INT1 vector, INT0 must
      also be implemented!) */
COMMON_ATTRIBUTE_VECTORTABLE u16 g_VectorTable_ui32[1] = {
    (u16)&_vtable_entry_reset_handler,
    /* if need interrupts, one can add handlers with help of linker script here */
    /* (u16)&_vtable_entry_handler_INT0
     * (u16)&_vtable_entry_handler_INT1
     * (u16)&_vtable_entry_handler_TIMER1_CAPT
     * (u16)&_vtable_entry_handler_TIMER1_COMPA
     * (u16)&_vtable_entry_handler_TIMER1_OVF
     * (u16)&_vtable_entry_handler_TIMER0_OVF
     * (u16)&_vtable_entry_handler_USART0_RX
     * (u16)&_vtable_entry_handler_USART0_UDRE
     * (u16)&_vtable_entry_handler_USART0_TX
     * (u16)&_vtable_entry_handler_ANALOG_COMP
     * (u16)&_vtable_entry_handler_TIMER1_COMPB
     * (u16)&_vtable_entry_handler_TIMER0_COMPA
     * (u16)&_vtable_entry_handler_TIMER0_COMPB
     * (u16)&_vtable_entry_handler_USI_START
     * (u16)&_vtable_entry_handler_USI_OVERFLOW
     * (u16)&_vtable_entry_handler_EE_READY
     * (u16)&_vtable_entry_handler_WDT_OVERFLOW
     */
};
#endif

/**********************************************************************************************************************
 * Module Prototypes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

/**
 * @brief function to copy the given number of bytes from source to destination (bytewise)
 *
 * @param[out] f_pDestStart_u8 destination start address to which data should be written
 * @param[in] f_pSourceStart_u8 source start address from that data should be read
 * @param[in] f_bytesToCopy_u32 number of bytes to copy from source to destination (starting from start address)
 */
STATIC INLINE void Startup_copyMemorySection(u8 *f_pDestStart_u8, u16 f_pSourceStart_u16, u8 f_bytesToCopy_u32)
{
  for (u8 l_idx_u32 = 0; l_idx_u32 < f_bytesToCopy_u32; l_idx_u32++)
  {
    f_pDestStart_u8[l_idx_u32] = __read_byte_from_flash(f_pSourceStart_u16 + l_idx_u32);
  }
}

/*********************************************************************************************************************/
/**
 * @brief function to initialize a memory section to a defined value (bytewise init)
 *
 * @param[out] f_pDestStart_u8 start address of the memory area which should be initialized
 * @param[in] f_valueToSet_u8 value byte which should be written to each byte in the memory region to init
 * @param[in] f_bytesToSet_u32 number of bytes to initialize (starting from start address)
 */
STATIC INLINE void Startup_initMemorySection(u8 *f_pDestStart_u8, u8 f_valueToSet_u8, u8 f_bytesToSet_u32)
{
  for (u8 l_idx_u32 = 0; l_idx_u32 < f_bytesToSet_u32; l_idx_u32++)
  {
    f_pDestStart_u8[l_idx_u32] = f_valueToSet_u8;
  }
}

/*********************************************************************************************************************/
__attribute__((section(".isr_vector_reset_handler"))) __attribute__((naked)) void
__vector_Application_ResetHandler(void)
{
  /* should not use any local variable here, the stack pointer is not yet initialized! */

  /* set stack pointer here; stack pointer on avr uses post decrement, so start at last existing memory location in ram;
have only 128 byte ram, so have no SPH */
  asm volatile("ldi r16, 0xDF\n\t"
               "out __SP_L__,r16\n\t" ::);

  /* default init of core registers to 0; is not done byte hardware */
  asm volatile("clr r0  \n\t"
               "clr r1  \n\t"
               "clr r2  \n\t"
               "clr r3  \n\t"
               "clr r4  \n\t"
               "clr r5  \n\t"
               "clr r6  \n\t"
               "clr r7  \n\t"
               "clr r8  \n\t"
               "clr r9  \n\t"
               "clr r10  \n\t"
               "clr r11  \n\t"
               "clr r12  \n\t"
               "clr r13  \n\t"
               "clr r14  \n\t"
               "clr r15  \n\t"
               "clr r16  \n\t"
               "clr r17  \n\t"
               "clr r18  \n\t"
               "clr r19  \n\t"
               "clr r20  \n\t"
               "clr r21  \n\t"
               "clr r22  \n\t"
               "clr r23  \n\t"
               "clr r24  \n\t"
               "clr r25  \n\t"
               "clr r26  \n\t"
               "clr r27  \n\t"
               "clr r28  \n\t"
               "clr r29  \n\t"
               "clr r30  \n\t"
               "clr r31  \n\t" ::);

  /* initialize the data section; must use the & in front of the "variables", because they are
   * symbols that come from the linker script and must always be handled that way; this must
   * also be done for _data_size ! */
  Startup_copyMemorySection((u8 *)&_sdata, (u16)&_data_loadaddr, (u8)(u16)&_data_size);

  /* initialize the bss section; must use the & in front of the "variables", because they are
   * symbols that come from the linker script and must always be handled that way; this must
   * also be done for _data_size ! */
  Startup_initMemorySection((u8 *)&_sbss, 0u, (u8)(u16)&_bss_size);

  /* call the main application; use rjmp to save stack data, because function call results in rcall
    which stores the pc + 1 on as 16 bit on stack */
  asm volatile("rjmp appl_main" ::);
}
