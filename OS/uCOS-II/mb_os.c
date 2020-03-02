/*
*********************************************************************************************************
*                                              uC/Modbus
*                                       The Embedded Modbus Stack
*
*                    Copyright 2003-2020 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                   MODBUS uC/OS-II LAYER INTERFACE
*
* Filename : mb_os.c
* Version  : V2.14.00
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#define   MB_OS_MODULE
#include <mb.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  OS_EVENT  *MB_OS_RxSemTbl[MODBUS_CFG_MAX_CH];
#endif

#if (MODBUS_CFG_SLAVE_EN  == DEF_ENABLED)
static  OS_STK     MB_OS_RxTaskStk[MB_OS_CFG_RX_TASK_STK_SIZE];

static  OS_EVENT  *MB_OS_RxQ;
static  void      *MB_OS_RxQTbl[MODBUS_CFG_MAX_CH];
#endif


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  void  MB_OS_InitMaster(void);
static  void  MB_OS_ExitMaster(void);
#endif

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  void  MB_OS_InitSlave (void);
static  void  MB_OS_ExitSlave (void);
static  void  MB_OS_RxTask    (void  *p_arg);
#endif


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              MB_OS_Init()
*
* Description : This function initializes the RTOS interface.  This function creates the following:
*
*               (1) A message queue to signal the reception of a packet.
*
*               (2) A task that waits for packets to be received.
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : MB_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_OS_Init (void)
{
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
    MB_OS_InitMaster();
#endif

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
    MB_OS_InitSlave();
#endif
}


/*
*********************************************************************************************************
*                                          MB_OS_InitMaster()
*
* Description : This function initializes and creates the kernel objectes needed for Modbus Master
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  void  MB_OS_InitMaster (void)
{
    OS_EVENT    *pevent;
    CPU_INT08U   i;
    CPU_INT08U   err;


    for (i = 0; i < MODBUS_CFG_MAX_CH; i++) {                             /* Create a semaphore for each channel   */
        pevent = OSSemCreate(0);
        if (pevent != (OS_EVENT *)0) {                                    /* Assign a name to the semaphore        */
            MB_OS_RxSemTbl[i] = pevent;

#if (OS_VERSION         < 287)
#if (OS_EVENT_NAME_SIZE >  16)

            OSEventNameSet(pevent,
                           (INT8U    *)"uC/Modbus Rx Sem",
                           &err);
            (void)&err;
#endif
#else
#if (OS_EVENT_NAME_EN > 0)

            OSEventNameSet(pevent,
                           (INT8U    *)"uC/Modbus Rx Sem",
                           &err);
            (void)&err;
#endif
#endif
        }
    }
}
#endif


/*
*********************************************************************************************************
*                                          MB_OS_InitSlave()
*
* Description : This function initializes and creates the kernel objectes needed for Modbus Salve
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  void  MB_OS_InitSlave (void)
{
    INT8U  err;


    MB_OS_RxQ = OSQCreate(&MB_OS_RxQTbl[0],                              /* Create Rx Task message queue           */
                          MODBUS_CFG_MAX_CH);

#if (OS_VERSION         < 287)
#if (OS_EVENT_NAME_SIZE >  14)
    if (MB_OS_RxQ != (OS_EVENT *)0) {                                    /* Assign a name to the message queue     */
        OSEventNameSet(MB_OS_RxQ,
                       (INT8U    *)"uC/Modbus Rx Q",
                       &err);
       (void)&err;
    }
#endif
#else
#if (OS_EVENT_NAME_EN   > 0)
    if (MB_OS_RxQ != (OS_EVENT *)0) {
        OSEventNameSet(MB_OS_RxQ,
                       (INT8U    *)"uC/Modbus Rx Q",
                       &err);
       (void)&err;
    }
#endif
#endif

#if (OS_TASK_CREATE_EXT_EN > 0)
    #if (OS_STK_GROWTH == 1)
    (void)OSTaskCreateExt((void (*)(void *)) MB_OS_RxTask,
                          (void           *) 0,
                                            &MB_OS_RxTaskStk[MB_OS_CFG_RX_TASK_STK_SIZE - 1],
                                             MB_OS_CFG_RX_TASK_PRIO,
                                             MB_OS_CFG_RX_TASK_ID,
                                            &MB_OS_RxTaskStk[0],
                                             MB_OS_CFG_RX_TASK_STK_SIZE,
                          (void           *) 0,
                          (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    #else
    (void)OSTaskCreateExt((void (*)(void *)) MB_OS_RxTask,
                          (void           *) 0,
                                            &MB_OS_RxTaskStk[0],
                                             MB_OS_CFG_RX_TASK_PRIO,
                                             MB_OS_CFG_RX_TASK_ID,
                                            &MB_OS_RxTaskStk[MB_OS_CFG_RX_TASK_STK_SIZE - 1],
                                             MB_OS_CFG_RX_TASK_STK_SIZE,
                          (void           *) 0,
                          (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    #endif
#else
    #if OS_STK_GROWTH == 1
    (void)OSTaskCreate((void (*)(void *)) MB_OS_RxTask,
                       (void           *) 0,
                                         &MB_OS_RxTaskStk[MB_OS_CFG_RX_TASK_STK_SIZE - 1],
                                          MB_OS_CFG_RX_TASK_PRIO);
    #else
    (void)OSTaskCreate((void (*)(void *)) MB_OS_RxTask,
                       (void           *) 0,
                                         &MB_OS_RxTaskStk[0],
                                          MB_OS_CFG_RX_TASK_PRIO);
    #endif
#endif

#if (OS_VERSION         < 287)
#if (OS_EVENT_NAME_SIZE >  12)

    OSTaskNameSet(         MB_OS_CFG_RX_TASK_PRIO,                       /* Assign a name to the event flag group  */
                  (INT8U *)"uC/Modbus Rx",
                           &err);
#endif
#else
#if (OS_EVENT_NAME_EN   > 0)
    OSTaskNameSet(         MB_OS_CFG_RX_TASK_PRIO,
                  (INT8U *)"uC/Modbus Rx",
                           &err);

#endif
#endif


}
#endif


/*
*********************************************************************************************************
*                                             MB_OS_Exit()
*
* Description : This function is called to terminate the RTOS interface for Modbus channels.  We delete
*               the following uC/OS-II objects:
*
*               (1) An event flag group to signal the reception of a packet.
*               (2) A task that waits for packets to be received.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_Exit().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_OS_Exit (void)
{
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
    MB_OS_ExitMaster();
#endif

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
    MB_OS_ExitSlave();
#endif
}


/*
*********************************************************************************************************
*                                          MB_OS_ExitMaster()
*
* Description : This function is called to terminate the RTOS interface for Modbus Master channels.  The
*               following objects are deleted.
*
*               (1) An event flag group to signal the reception of a packet.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_Exit().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  void  MB_OS_ExitMaster (void)
{
    CPU_INT08U  i;
    INT8U       err;


    for (i = 0; i < MODBUS_CFG_MAX_CH; i++) {                 /* Delete semaphore for each channel     */
        OSSemDel(MB_OS_RxSemTbl[i],
                 OS_DEL_ALWAYS,
                 &err);
    }
}
#endif


/*
*********************************************************************************************************
*                                          MB_OS_ExitSlave()
*
* Description : This function is called to terminate the RTOS interface for Modbus Salve channels.
*               The following objects are deleted.
*
*               (1) A task that waits for packets to be received.
*               (2) A message queue to signal the reception of a packet.
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_Exit().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
void  MB_OS_ExitSlave (void)
{
    INT8U  err;


    OSTaskDel(MB_OS_CFG_RX_TASK_PRIO);                        /* Delete Modbus Rx Task                 */
    OSQDel(MB_OS_RxQ,                                         /* Delete the Queue                      */
           OS_DEL_ALWAYS,
           &err);
}
#endif


/*
*********************************************************************************************************
*                                              MB_OS_RxSignal()
*
* Description : This function signals the reception of a packet either from the Rx ISR(s) or the RTU timeout
*               timer(s) to indicate that a received packet needs to be processed.
*
* Argument(s) : pch     specifies the Modbus channel data structure in which a packet was received.
*
* Return(s)   : none.
*
* Caller(s)   : MB_ASCII_RxByte(),
*               MB_RTU_TmrUpdate().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_OS_RxSignal (MODBUS_CH *pch)
{
    if (pch != (MODBUS_CH *)0) {
        switch (pch->MasterSlave) {
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
            case MODBUS_MASTER:
                 (void)OSSemPost(MB_OS_RxSemTbl[pch->Ch]);
                 break;
#endif

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
            case MODBUS_SLAVE:
            default:
                 (void)OSQPost(MB_OS_RxQ,
                               (void *)pch);
                 break;
#endif
        }
    }
}


/*
*********************************************************************************************************
*                                              MB_OS_RxWait()
*
* Description : This function waits for a response from a slave.
*
* Argument(s) : pch     specifies the Modbus channel data structure to wait on.
*
*               perr    is a pointer to a variable that will receive an error code.  Possible errors are:
*
*                       MODBUS_ERR_NONE        the call was successful and a packet was received
*                       MODBUS_ERR_TIMED_OUT   a packet was not received within the specified timeout
*                       MODBUS_ERR_NOT_MASTER  the channel is not a Master
*                       MODBUS_ERR_INVALID     an invalid error was detected
*
* Return(s)   : none.
*
* Caller(s)   : MBM_FCxx()  Modbus Master Functions
*
* Return(s)   : none.
*********************************************************************************************************
*/

void  MB_OS_RxWait (MODBUS_CH   *pch,
                    CPU_INT16U  *perr)
{
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
    INT8U  err;


    if (pch != (MODBUS_CH *)0) {
        if (pch->MasterSlave == MODBUS_MASTER) {
            OSSemPend(MB_OS_RxSemTbl[pch->Ch],
                      pch->RxTimeout,
                      &err);
            switch (err) {
                case OS_ERR_EVENT_TYPE:
                case OS_ERR_PEND_ISR:
                case OS_ERR_PEND_LOCKED:
                     *perr = MODBUS_ERR_INVALID;
                     break;

                case OS_ERR_TIMEOUT:
                     *perr = MODBUS_ERR_TIMED_OUT;
                     break;

                case OS_ERR_NONE:
                     *perr = MODBUS_ERR_NONE;
                     break;
            }
        } else {
            *perr = MODBUS_ERR_NOT_MASTER;
        }
    } else {
        *perr = MODBUS_ERR_NULLPTR;
    }
#else
    *perr = MODBUS_ERR_INVALID;
#endif
}


/*
*********************************************************************************************************
*                                            MB_OS_RxTask()
*
* Description : This task is created by MB_OS_Init() and waits for signals from either the Rx ISR(s) or
*               the RTU timeout timer(s) to indicate that a packet needs to be processed.
*
* Argument(s) : p_arg       is a pointer to an optional argument that is passed by uC/OS-II to the task.
*                           This argument is not used.
*
* Return(s)   : none.
*
* Caller(s)   : This is a Task.
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  void  MB_OS_RxTask (void *p_arg)
{
    INT8U        err;
    MODBUS_CH   *pch;


    (void)p_arg;

    while (DEF_TRUE) {
        pch = (MODBUS_CH *)OSQPend(MB_OS_RxQ,    /* Wait for a packet to be received                   */
                                   0,
                                   &err);
        MB_RxTask(pch);                          /* Process the packet received                        */
    }
}
#endif

