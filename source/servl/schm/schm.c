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
 * @file : schm.c
 * @author : Florian Wank
 * @date : 23.04.2025
 *
 *
 * @brief  source file for the scheduler implementation
 *
 * @details This source file contains a simple and very small scheduler. On the top level a task handling is 
 *          implemented with a total slot time of 250us in the following way: 
 * [uTask - 25us][Task0 - 225us][uTask - 25us][Task1 - 225us][uTask - 25us][Task2 - 225us][uTask - 25us][Task3 - 225us]
 *          The shown sequence is repeated endless. This is already a cooperative scheduling scheme which allows 
 *          each task to be executed every 1ms with a maximum task time of 225us. For longer tasks a cooperative 
 *          processing mechanism is implemented on top. In Task 1 a process is called. Due to memory restrictions
 *          in this implementations are only two processes : the main process, which contains the normal task execution,
 *          and an additional process with ID0 that is called every Task 1. This process itself yields, so gives
 *          control back to main process. So the full processing scheme is like following:
 *          [uTask ][Task0][uTask][Task1 -> PID0 until yield][uTask][Task2][uTask][Task3]
 *          [uTask ][Task0][uTask][Task1 -> PID0 until yield][uTask][Task2][uTask][Task3]
 *          ....
 *          So in PID0 process one can create a long sequence which normally exceeds the task time, but give control
 *          back frequently, so that during scheduling the slot time is never exceeded.
 * 
 *          The implementation is kept as simple as possible and it is taken care of the very limited ressources. 
 *          For that reason one must note, that only a few bytes are left free. So if implementing a real 
 *          application on top one should 
 *          (1) use a very long call depth of functions, so inline whereever possible
 *          (2) analyse the memory usage with a simulator (->the timer value check must be commented out to be always true)
 * 
 *          In general this scheduler implementation is just for prooving that we can implement a scheduler in C for
 *          such a tiny device :-)
 *
 */
/*********************************************************************************************************************/

/*********************************************************************************************************************
 * Module Includes
 **********************************************************************************************************************/
#include "servl/schm/schm.h"
#include "common/common.h"
#include "common/types.h"
#include "mcal/core/core.h"
#include "mcal/gpt/gpt.h"

/**********************************************************************************************************************
 * Module Definitions
 **********************************************************************************************************************/

/** type definition that is part of scheduler; defines the task function */
typedef void (*taskFunction)(void);

/** type definition that is part of scheduler; defines the process function */
typedef void (*processFunc)(void);

/** define the available process IDs (PIDs); the chosen mcu attiny2313 is very small, so can have
 * only one process in addition to the main process
 */
typedef enum {
  PID_00 = 0,      /**< process 0 ID */
  PID_MAIN = 0xC3, /**< main process ID */
} pid;

/** @brief struct that defines a task; actually this struct should contain more information about the task,
 * e.g. max runtime or min runtime, but the chosen mcu has not enough memory to support this
 */
typedef struct {
  taskFunction task_t; /**< task function to be executed */
} Task;

/** @brief scheduler definition; must create only one instance of that type */
typedef struct {
  taskFunction activeTaskFunction_t; /**< the currently active task function */
  u32 schedulerTimestamp_u32; /**< the scheduler timestamp; is limited to u32, so will overflow after ~12,43days at the
                                 chosen 250us timeslots */
  u16 maxTaskRuntime_u16; /**< maximum task runtime; normally there should be a runtime value per task, due to memory
                             limitations can only have one value here */
  taskFunction uTask0_t;  /**< urgent task 0, executed every 250us, max duration should be 15us (25us slot reserved in
                             each 250us slot) */
  u8 taskIdx_u8;          /**< active task index */
  Task *taskList_t[4];    /**< task list */
} scheduler;

/** @brief process definition */
typedef struct {
  pid pid_t;          /**< process identifier */
  u16 stackPtr_u16;   /**< value of the stack pointer that is used by the task */
  processFunc func_t; /**< function to be processed */
} process;

/**********************************************************************************************************************
 * Module Constants
 **********************************************************************************************************************/

/** define the context size on stack;
 *  have 32 mcu registers; have 1 status register to save; have 2 bytes for correct return to save
 * (-> "link address on stack is 16 bit") */
#define CONTEXT_SIZE (32 + 1 + 2)

/** define the stack size of for process with PID00; must be at least CONTEXT_SIZE, should not exceed 40 for attiny2313
 */
#define STACK_SIZE_PID00 40

/** schedule timeslot time; must never be changed! in us */
#define TASK_SCHEDULE_TIMESLOT_TIME_US 250

/** urgent task time in us; urgent task is called every TASK_SCHEDULE_TIMESLOT_TIME_US */
#define TASK_SCHEDULE_URGENT_TASK_TIME_US 25

/** count correction value for scheduler; is used to get the correct timing for 250us task slot aligning, because of
 * accuracy issues with the internal clock source
 */
#define F_CPU_TASK_CNT_CORRECTION 65

/** count correction value for scheduler; is used to get the correct timing for 25us urgent slot aligning, because of
 * accuracy issues with the internal clock source
 */
#define F_CPU_URGENT_TASK_CNT_CORRECTION 6

/** reference count value to which the scheduler is synced; this value ensures the timeslot matching; use a correction
 * factor due to oscillator accuracy limitations */
#define TASK_SCHED_CNT_START_REF_VAL                                                                                   \
  ((F_CPU / (1000000UL) * TASK_SCHEDULE_TIMESLOT_TIME_US) + F_CPU_TASK_CNT_CORRECTION)

/** time amount that is used for urgent task; if the urgent task is faster, wait this value to be deterministic! use a
 * correction factor due to oscillator accuracy limitations */
#define TASK_SCHED_CNT_URGENT_REF_VAL                                                                                  \
  ((F_CPU / (1000000UL) * TASK_SCHEDULE_URGENT_TASK_TIME_US) + F_CPU_URGENT_TASK_CNT_CORRECTION)

/**********************************************************************************************************************
 * Module Enumerations
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Module Variables
 **********************************************************************************************************************/

/* need function declaration here in order to setup the module variables */

/** @brief urgent task; to be executed every 250 us -> before every normal task */
void uTask_0(void);

/** @brief task 0 function definition */
void Task_0(void);

/** @brief task 1 function definition */
void Task_1(void);

/** @brief task 2 function definition */
void Task_2(void);

/** @brief task 3 function definition */
void Task_3(void);

/** @brief process function for ID0 definition */
void process_Pid00(void);

/** task 0 */
Task Task_0_t = {.task_t = Task_0};

/** task 1 */
Task Task_1_t = {.task_t = Task_1};

/** task 2 */
Task Task_2_t = {.task_t = Task_2};

/** task 3 */
Task Task_3_t = {.task_t = Task_3};

/** scheduler data; should have only one scheduler instance in project;
 * setup the default startup data, ensure correct setup by review!
 */
scheduler Scheduler_t = {
    .activeTaskFunction_t = NULL,
    .schedulerTimestamp_u32 = 0,
    .uTask0_t = uTask_0,
    .taskIdx_u8 = 0,
    .maxTaskRuntime_u16 = 0,
    .taskList_t =
        {
            [0] = &Task_0_t,
            [1] = &Task_1_t,
            [2] = &Task_2_t,
            [3] = &Task_3_t,
        },
};

/** main process; the main process uses the normal application stack, so
 * have no additional stack pointer reserved and do not have to setup here; during first
 * call of different process the stack pointer will get its first value; in addition the processing
 * function is currently only for information and debugging purpose, here NULL
 */
process main_process = {
    .pid_t = PID_MAIN,
    .stackPtr_u16 = 0,
    .func_t = NULL,
};

/** stack for the process with ID 0 */
u8 stackPid00[STACK_SIZE_PID00] = {0};

/** process definition of process 0; setup the stack pointer so that it points to the location at
 * which a popping can correctly start; need to call the init function for additional stack setup,
 * before the scheduler can be launched! The processing function is currently for information and debugging
 * purpse only!
 */
process pid0_process = {
    .pid_t = PID_00,
    .stackPtr_u16 = (u16)&stackPid00[STACK_SIZE_PID00 - CONTEXT_SIZE - 1],
    .func_t = &process_Pid00,
};

/** active process id variable */
pid activeProcess = PID_MAIN;

/**********************************************************************************************************************
 * Module Prototypes
 **********************************************************************************************************************/

/** @brief function init the stack; must be called before scheduling starts!
 *
 * @details The stack organization is return prog counter low byte, prog counter high byte,
 *          r0, r1 ... r31, status register */
STATIC INLINE void schm_initStack(u8 *stack_u8, u8 stacksize_u8, pid pid_t);

/**
 * @brief function to evaluate the maximum scheduling time; returns the max. value of the two input params
 */
STATIC INLINE u16 schm_getMaxTime(u16 taskMaxTime_u32, u16 measuredTaskTime_u32);

/** @brief get the current stack pointer value */
STATIC INLINE u16 schm_getSP(void);

/** @brief  set the stack pointer to the given value */
STATIC INLINE void schm_setSP(u16 value);

/**
 * @brief function to change the processing context!
 * 
 * @details This function changes the active context to the next context. The contexts must never be equal.
 *          During context switch first the active context is stored to the currently active stack, than
 *          the stack pointer is switched. After switching the stack pointer the next process context is
 *          restored from stack. The avr core stores the jump back address also on stack, so if this function
 *          is left, the execution address is restored from the just setup stack.
 *  
 */
extern __attribute__((naked)) void changeContext(process *active, process *next);

/** @brief function to run the process with the given process ID */
void __attribute__((noinline)) run_process(pid pid_t);

/** @brief function to yield the processing; must not be called in main process, because will resume to main process */
void __attribute__((noinline)) yield(void);

/**********************************************************************************************************************
 * Module Function Definitions
 **********************************************************************************************************************/

/* stack organization is return prog counter low byte, prog counter high byte,
   r0, r1 ... r31, status register */
STATIC INLINE void schm_initStack(u8 *stack_u8, u8 stacksize_u8, pid pid_t)
{
  /* setup deadbeef pattern in bytes to see if stack reaches the limit */
  stack_u8[0] = 0xde;
  stack_u8[1] = 0xad;
  stack_u8[2] = 0xbe;
  stack_u8[3] = 0xef;

  /* init the stack with default 0*/
  for (u8 i = 4; i < stacksize_u8; i++)
  {
    stack_u8[i] = 0;
  }

  /* setup default ret address; during ret two bytes are popped from stack which
  represent the address to which to jump back; for init of process need to
  setup the function entry points and locate them on the stack */
  u16 funcLink_u16 = (u16)&process_Pid00;
  stack_u8[stacksize_u8 - 1] = funcLink_u16 & 0xFF;
  stack_u8[stacksize_u8 - 2] = funcLink_u16 >> 8;
}

/*********************************************************************************************************************/
STATIC INLINE u16 schm_getMaxTime(u16 taskMaxTime_u32, u16 measuredTaskTime_u32)
{
  return ((taskMaxTime_u32 > measuredTaskTime_u32) ? taskMaxTime_u32 : measuredTaskTime_u32);
}

/*********************************************************************************************************************/
STATIC INLINE u16 schm_getSP(void)
{
  /* have only spl on this attiny; if have sph, read sph to r31 */
  u16 value = 0;
  asm volatile("in r30, __SP_L__ \n\t" : "=z"(value) : :);
  return value;
}

/*********************************************************************************************************************/
STATIC INLINE void schm_setSP(u16 value)
{
  /* have only spl on this attiny; if have sph, write r31 to sph */
  asm volatile("out __SP_L__,r30\n\t" : : "z"(value) :);
}

/*********************************************************************************************************************/
extern __attribute__((naked)) void changeContext(process *active, process *next)
{
  asm volatile("push r0 \r\n"
               "push r1 \r\n"
               "push r2 \r\n"
               "push r3 \r\n"
               "push r4 \r\n"
               "push r5 \r\n"
               "push r6 \r\n"
               "push r7 \r\n"
               "push r8 \r\n"
               "push r9 \r\n"
               "push r10 \r\n"
               "push r11 \r\n"
               "push r12 \r\n"
               "push r13 \r\n"
               "push r14 \r\n"
               "push r15 \r\n"
               "push r16 \r\n"
               "push r17 \r\n"
               "push r18 \r\n"
               "push r19 \r\n"
               "push r20 \r\n"
               "push r21 \r\n"
               "push r22 \r\n"
               "push r23 \r\n"
               "push r24 \r\n"
               "push r25 \r\n"
               "push r26 \r\n"
               "push r27 \r\n"
               "push r28 \r\n"
               "push r29 \r\n"
               "push r30 \r\n"
               /* could not push r31 with emulator, so use workaround to push r31 */
               "mov r30, r31 \r\n"
               "push r30 \r\n"
               "in r0, __SREG__ \r\n"
               "push r0 \r\n"
               :
               :
               :);

  activeProcess = next->pid_t;
  active->stackPtr_u16 = (u16)schm_getSP();
  schm_setSP((u16)(next->stackPtr_u16));

  asm volatile(

      "pop r0 \r\n"
      "out __SREG__, r0 \r\n"
      "pop r31 \r\n"
      "pop r30 \r\n"
      "pop r29 \r\n"
      "pop r28 \r\n"
      "pop r27 \r\n"
      "pop r26 \r\n"
      "pop r25 \r\n"
      "pop r24 \r\n"
      "pop r23 \r\n"
      "pop r22 \r\n"
      "pop r21 \r\n"
      "pop r20 \r\n"
      "pop r19 \r\n" /** schedule timeslot time; must never be changed! */
      "pop r18 \r\n"
      "pop r17 \r\n"
      "pop r16 \r\n"
      "pop r15 \r\n"
      "pop r14 \r\n"
      "pop r13 \r\n"
      "pop r12 \r\n"
      "pop r11 \r\n"
      "pop r10 \r\n"
      "pop r9 \r\n"
      "pop r8 \r\n"
      "pop r7 \r\n"
      "pop r6 \r\n"
      "pop r5 \r\n"
      "pop r4 \r\n"
      "pop r3 \r\n"
      "pop r2 \r\n"
      "pop r1 \r\n"
      "pop r0 \r\n"
      "ret \r\n"
      :
      :
      :);
}

/*********************************************************************************************************************/
void __attribute__((noinline)) yield(void)
{
  /* to yield switch back to main process; in this implementation have only process with pid0 due to
     memory limitations; if have more processes, one has to organize them in an array and use the
     active pid to get the process that yields; */
  changeContext(&pid0_process, &main_process);
}

/*********************************************************************************************************************/
void __attribute__((noinline)) run_process(pid pid_t)
{
  /* if have more processes, one has to organize them in an array and use the
     active pid  as index to get the process which should be executed */
  if (pid_t == PID_00)
  {
    changeContext(&main_process, &pid0_process);
  }
}

/*********************************************************************************************************************/
__attribute__((noreturn)) void Sched_run(void)
{
  while (1)
  {
    if (gpt_getCntValue() >= (TASK_SCHED_CNT_START_REF_VAL))
    {
      gpt_resetCntValue();

      Scheduler_t.activeTaskFunction_t = Scheduler_t.uTask0_t;
      Scheduler_t.uTask0_t();

      /* wait that 25 us are over for urgent slot*/
      while (gpt_getCntValue() < (TASK_SCHED_CNT_URGENT_REF_VAL))
        ;

      Scheduler_t.activeTaskFunction_t = Scheduler_t.taskList_t[Scheduler_t.taskIdx_u8]->task_t;
      Scheduler_t.activeTaskFunction_t();
      u16 taskRuntime_u32 = gpt_getCntValue() - 200;
      Scheduler_t.maxTaskRuntime_u16 = schm_getMaxTime(Scheduler_t.maxTaskRuntime_u16, taskRuntime_u32);
      Scheduler_t.taskIdx_u8++;
      Scheduler_t.taskIdx_u8 = (Scheduler_t.taskIdx_u8 >= 4) ? 0 : Scheduler_t.taskIdx_u8;

      if (gpt_isTimerElapsed())
      {
        /* should not get here; if get here, the cooperation slot time during scheduling was exceeded */
        asm volatile("nop \r\n");
      }
    }
  }
}

/*********************************************************************************************************************/
void uTask_0(void)
{
  /* the urgent task 0 is the time provider for the system; this makes the system very deterministic;
    caution: due to memory limitations and the configured scheduling time, the timestamp will overflow
    after ~12 days */
  Scheduler_t.schedulerTimestamp_u32 += TASK_SCHEDULE_TIMESLOT_TIME_US;
}

/*********************************************************************************************************************/
void Task_0(void)
{
  /* currently nothing to do in task 0 */
  asm volatile("nop \r\n");
}

/*********************************************************************************************************************/
void Task_1(void)
{
  asm volatile("nop \r\n");
  run_process(PID_00);
}

/*********************************************************************************************************************/
void Task_2(void)
{
  /* currently nothing to do in task 2 */
  asm volatile("nop \r\n");
}

/*********************************************************************************************************************/
void Task_3(void)
{
  /* currently nothing to do in task 3 */
  asm volatile("nop \r\n");
}

/*********************************************************************************************************************/
void process_Pid00(void)
{
  u8 a = 0;
  u8 cnt = 0;
  u8 toggle = 0;
  while (1)
  {
    cnt++;
    /* process that is called by task is executed every 1ms; count to 100 and have two yields, so high time / low time
     * is 200ms each */
    if (cnt == 100)
    {
      cnt = 0;
      if (toggle == 0)
      {
        toggle = 1;
        *REG_PORTD &= ~(1 << 0);
      } else
      {
        toggle = 0;
        *REG_PORTD |= (1 << 0);
      }
    }

    asm volatile("nop \r\n");
    yield();
    a++;
    asm volatile("nop \r\n");
    yield();
    a++;
    asm volatile("nop \r\n");
    if (a > 10)
    {
      a = 0;
    }
  }
}

/*********************************************************************************************************************/
void schm_init(void)
{
  /* init the stack(s) for the implemented processes */
  schm_initStack((u8 *)&stackPid00, STACK_SIZE_PID00, PID_00);
}
