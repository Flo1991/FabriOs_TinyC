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
 * @file : common.h
 * @author : Florian Wank
 * @date : 23.04.2025
 *
 *
 * @brief  header file defining common definitions for the project
 *
 * @details
 * Since this project does not use any avr lib, we need to define various
 * definitions to be used in the project.
 *
 */
/*********************************************************************************************************************/

#ifndef COMMON_H
#define COMMON_H

/*********************************************************************************************************************
 * Module Includes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Definitions
 **********************************************************************************************************************/

/** NULL is a nullpointer definition */
#define NULL ((void *)0)

/** nullpointer definition; can be used as alternative to NULL */
#define NULLPTR ((void *)0)

/** mcu clock is 8 MHz */
#define F_CPU 8000000UL

#if defined(SW_TEST) && (SW_TEST != 0)
/** for software testing make all functions available, so STATIC results in
 * empty define */
#define STATIC
#else
/** define STATIC macro as static keyword; to be used for all static functions;
 * avoid using static functions in header files due to mocking effort */
#define STATIC static
#endif

#if defined(SW_TEST) && (SW_TEST != 0)
/** for software testing make all functions available, so INLINE results in
 * empty define */
#define INLINE
#else
/** define INLINE macro as inline keyword; to be used for all inline functions;
 * avoid using inline functions in header files due to mocking effort */
#define INLINE inline
#endif

#if defined(SW_TEST) && (SW_TEST != 0)
/** for software testing with simulator with have no vector table, so use empty
 * define */
#define COMMON_ATTRIBUTE_VECTORTABLE
#else
/** compiler attribute definition to relocate data to the isr_vector section;
 * must only be used once to setup the vector table */
#define COMMON_ATTRIBUTE_VECTORTABLE __attribute__((section(".isr_vector")))
#endif

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

#endif /* COMMON_H */