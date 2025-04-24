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
 * @file : core.h
 * @author : Florian Wank
 * @date : 23.04.2025
 *
 *
 * @brief  header file defining the core register, peripherals, core functions
 *
 *
 */
/*********************************************************************************************************************/

#ifndef CORE_H
#define CORE_H

/*********************************************************************************************************************
 * Module Includes
 **********************************************************************************************************************/

#include "common/types.h"

/**********************************************************************************************************************
 * Module Definitions
 **********************************************************************************************************************/

 /** register offset definition; may not be necessary for other avr devices */
#define REG_OFFSET 0x20

#define REG_SREG ((volatile u8 *)(0x3F + REG_OFFSET))
#define REG_SPL ((volatile u8 *)(0x3D + REG_OFFSET))
#define REG_OCR0B ((volatile u8 *)(0x3C + REG_OFFSET))
#define REG_GIMSK ((volatile u8 *)(0x3B + REG_OFFSET))
#define REG_EIFR ((volatile u8 *)(0x3A + REG_OFFSET))
#define REG_TIMSK ((volatile u8 *)(0x39 + REG_OFFSET))
#define REG_TIFR ((volatile u8 *)(0x38 + REG_OFFSET))
#define REG_SPMCSR ((volatile u8 *)(0x37 + REG_OFFSET))
#define REG_OCR0A ((volatile u8 *)(0x36 + REG_OFFSET))
#define REG_MCUCR ((volatile u8 *)(0x35 + REG_OFFSET))
#define REG_MCUSR ((volatile u8 *)(0x34 + REG_OFFSET))
#define REG_TCCR0B ((volatile u8 *)(0x33 + REG_OFFSET))
#define REG_TCNT0 ((volatile u8 *)(0x32 + REG_OFFSET))
#define REG_OSCCAL ((volatile u8 *)(0x31 + REG_OFFSET))
#define REG_TCCR0A ((volatile u8 *)(0x30 + REG_OFFSET))
#define REG_TCCR1A ((volatile u8 *)(0x2F + REG_OFFSET))
#define REG_TCCR1B ((volatile u8 *)(0x2E + REG_OFFSET))
#define REG_TCNT1H ((volatile u8 *)(0x2D + REG_OFFSET))
#define REG_TCNT1L ((volatile u8 *)(0x2C + REG_OFFSET))
#define REG_OCR1AH ((volatile u8 *)(0x2B + REG_OFFSET))
#define REG_OCR1AL ((volatile u8 *)(0x2A + REG_OFFSET))
#define REG_OCR1BH ((volatile u8 *)(0x29 + REG_OFFSET))
#define REG_OCR1BL ((volatile u8 *)(0x28 + REG_OFFSET))
#define REG_CLKPR ((volatile u8 *)(0x26 + REG_OFFSET))
#define REG_ICR1H ((volatile u8 *)(0x25 + REG_OFFSET))
#define REG_ICR1L ((volatile u8 *)(0x24 + REG_OFFSET))
#define REG_GTCCR ((volatile u8 *)(0x23 + REG_OFFSET))
#define REG_TCCR1C ((volatile u8 *)(0x22 + REG_OFFSET))
#define REG_WDTCSR ((volatile u8 *)(0x21 + REG_OFFSET))
#define REG_PCMSK ((volatile u8 *)(0x20 + REG_OFFSET))
#define REG_EEAR ((volatile u8 *)(0x1E + REG_OFFSET))
#define REG_EEDR ((volatile u8 *)(0x1D + REG_OFFSET))
#define REG_EECR ((volatile u8 *)(0x1C + REG_OFFSET))
#define REG_PORTA ((volatile u8 *)(0x1B + REG_OFFSET))
#define REG_DDRA ((volatile u8 *)(0x1A + REG_OFFSET))
#define REG_PINA ((volatile u8 *)(0x19 + REG_OFFSET))
#define REG_PORTB ((volatile u8 *)(0x18 + REG_OFFSET))
#define REG_DDRB ((volatile u8 *)(0x17 + REG_OFFSET))
#define REG_PINB ((volatile u8 *)(0x16 + REG_OFFSET))
#define REG_GPIOR2 ((volatile u8 *)(0x15 + REG_OFFSET))
#define REG_GPIOR1 ((volatile u8 *)(0x14 + REG_OFFSET))
#define REG_GPIOR0 ((volatile u8 *)(0x13 + REG_OFFSET))
#define REG_PORTD ((volatile u8 *)(0x12 + REG_OFFSET))
#define REG_DDRD ((volatile u8 *)(0x11 + REG_OFFSET))
#define REG_PIND ((volatile u8 *)(0x10 + REG_OFFSET))
#define REG_USIDR ((volatile u8 *)(0x0F + REG_OFFSET))
#define REG_USISR ((volatile u8 *)(0x0E + REG_OFFSET))
#define REG_USICR ((volatile u8 *)(0x0D + REG_OFFSET))
#define REG_UDR ((volatile u8 *)(0x0C + REG_OFFSET))
#define REG_UCSRA ((volatile u8 *)(0x0B + REG_OFFSET))
#define REG_UCSRB ((volatile u8 *)(0x0A + REG_OFFSET))
#define REG_UBRRL ((volatile u8 *)(0x09 + REG_OFFSET))
#define REG_ACSR ((volatile u8 *)(0x08 + REG_OFFSET))
#define REG_UCSRC ((volatile u8 *)(0x03 + REG_OFFSET))
#define REG_UBRRH ((volatile u8 *)(0x02 + REG_OFFSET))
#define REG_DIDR ((volatile u8 *)(0x01 + REG_OFFSET))

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

/**********************************************************************************************************************
 * Module Prototypes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

static inline u8 __read_byte_from_flash(u16 address)
{
  u8 byte;
  asm volatile("lpm \n\t"
               "mov %0, r0 \n\t"
               : "=r"(byte)
               : "z"(address)
               : "r0");
  return byte;
}

#endif /* CORE_H */
