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
 * @file : gpt.h
 * @author : Florian Wank
 * @date : 23.04.2025
 *
 *
 * @brief  header file for timer implementation
 *
 *
 */
/*********************************************************************************************************************/

#ifndef GPT_H
#define GPT_H

/*********************************************************************************************************************
 * Module Includes
 **********************************************************************************************************************/
#include "common/common.h"
#include "common/types.h"
#include "mcal/core/core.h"
#include "mcal/gpt/gpt.h"

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

/**********************************************************************************************************************
 * Module Prototypes
 **********************************************************************************************************************/

/**
 * @brief initialize the 16 bit timer to run in ctc mode
 *
 * @brief The 16 bit timer of the mcu is used in this application by the scheduler to create the correct timing. The
 *        timer is normally running and the scheduler should reset the count value. The timer runs in ctc mode and
 *        will set a status flag if the limit is reached. The limit can be used to see if a task took much too long.
 *        Configure the limit to your requirements.
 */
void gpt_init(void);

/**
 * @brief get the current count value of the 16 bit timer
 */
u16 gpt_getCntValue(void);

/**
 * @brief reset the cnt value of the 16 bit timer to 0
 */
void gpt_resetCntValue(void);

/**
 * @brief check if the 16 bit timer is elapsed
 *
 * @brief The 16 bit timer is running in ctc mode and used by the scheduler for correct timing. It the timer
 *        is elapsed, the intended scheduling time for cooperative scheduling was exceeded.
 */
bool gpt_isTimerElapsed(void);

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

#endif /* GPT_H */