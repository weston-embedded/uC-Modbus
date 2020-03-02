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
*                                    MODBUS NO-OS LAYER INTERFACE
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

#define    MB_OS_MODULE
#include  <mb.h>


/*
*********************************************************************************************************
*                                           LOCAL DEFINES
*********************************************************************************************************
*/

#define  MB_OS_ERR_NONE                                 0x00u
#define  MB_OS_ERR_Q_FULL                               0x01u
#define  MB_OS_ERR_Q_EMPTY                              0x03u
#define  MB_OS_ERR_TIMEOUT                              0x02u


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

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
typedef struct mb_os_q {
    MODBUS_CH   **OS_Q_StoPtr;                               /* Queue storage pointer                                   */
    CPU_INT08U    OS_Q_Entries;                              /* Queue current number of entries                         */
    CPU_INT08U    OS_Q_MaxEntries;                           /* Queue max number of entries                             */
    CPU_INT08U    OS_Q_FirstEntry;                           /* Queue first entry index                                 */
    CPU_INT08U    OS_Q_LastEntry;                            /* Queue last  entry index                                 */
} MB_OS_Q;
#endif


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

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  MB_OS_Q     MB_OS_RxQ;
static  MODBUS_CH  *MB_OS_RxQ_Tbl[MODBUS_CFG_MAX_CH];
#endif


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
                                                                 /* Queue functions                                    */
static  void        MB_OS_Q_Create (MB_OS_Q      *p_q,
                                    MODBUS_CH   **p_sto,
                                    CPU_INT08U    max_entries);

static  MODBUS_CH  *MB_OS_Q_Pend   (MB_OS_Q      *p_q);

static  CPU_INT08U  MB_OS_Q_Post   (MB_OS_Q      *p_q,
                                    MODBUS_CH    *p_msg);

static  void        MB_OS_InitSlave(void);
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
* Description : This function initializes the NON OS interface.  This function creates the following:
*
*               (1) A queue structure that contains the pointer to the channel(s) wher a message needs to be processed. (Modbus Slave)
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_Init()
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_OS_Init (void)
{
#if (MODBUS_CFG_SLAVE_EN  == DEF_ENABLED)
    MB_OS_InitSlave();
#endif
}


/*
*********************************************************************************************************
*                                          MB_OS_InitSlave()
*
* Description : This function create a queue structure for income Rx messages. (Modbus Slave)
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_Init()
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  void  MB_OS_InitSlave (void)
{
    MB_OS_Q_Create(&MB_OS_RxQ, &MB_OS_RxQ_Tbl[0], MODBUS_CFG_MAX_CH);
}
#endif


/*
*********************************************************************************************************
*                                             MB_OS_Exit()
*
* Description : This function is called to terminate the RTOS interface for Modbus channels.
*
*               No action is performed for the no-OS layer.
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

}


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
#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
    if (pch != (MODBUS_CH *)0) {
        MB_OS_Q_Post(&MB_OS_RxQ,
                     pch);
    }
#endif
}

/*
*********************************************************************************************************
*                                            MB_OS_RxTask()
*
* Description : This functions needs to be called frequently by the application to poll if a Rx has been
*               received.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) MB_OS_RxTask() checks whether there is an Rx message in the queue; if there is not,
                    this function will return immediately.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
void  MB_OS_RxTask (void)
{
    MODBUS_CH  *p_ch;


    p_ch = (MODBUS_CH *)MB_OS_Q_Pend(&MB_OS_RxQ);                /* Wait for a Rx packet                               */

    if (p_ch == (MODBUS_CH *)0) {
        return;
    }

    MB_RxTask(p_ch);                                            /* Process the packet received                        */

}
#endif

/*
*********************************************************************************************************
*                                            MB_OS_SemCreate()
*
* Description : This function creates a semaphore.
*
* Argument(s) : p_sem       Pointer to the semaphore.
*
*               cnt         Initial value of the semaphore
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_InitMaster()
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  void  MB_OS_SemCreate (MB_OS_SEM  *p_sem,
                               CPU_INT32U  cnt)
{
    p_sem->OS_SemCnt = cnt;
}
#endif


/*
*********************************************************************************************************
*                                            MB_OS_SemPost()
*
* Description : This function signals a semaphore
*
* Argument(s) : p_sem       Pointer to the semaphore.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_InitMaster()
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
static  void  MB_OS_SemPost (MB_OS_SEM  *p_sem)
{
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();                                       /* Increment the semaphore counts value               */
    p_sem->OS_SemCnt++;
    CPU_CRITICAL_EXIT();
}
#endif


/*
*********************************************************************************************************
*                                            MB_OS_Q_Create()
*
* Description : This function creates a message queue.
*
* Argument(s) : p_sem           Pointer to the message queue.
*
*               p_sto           Pointer to the base address of the message queue storage area.
*
*               max_entries     Number of elements in the storage area.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_InitMaster()
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  void  MB_OS_Q_Create (MB_OS_Q     *p_q,
                              MODBUS_CH  **p_sto,
                              CPU_INT08U   max_entries)
{
    p_q->OS_Q_StoPtr     = p_sto;
    p_q->OS_Q_Entries    = 0;
    p_q->OS_Q_MaxEntries = max_entries;
    p_q->OS_Q_FirstEntry = 0;
    p_q->OS_Q_LastEntry  = 0;
}
#endif


/*
*********************************************************************************************************
*                                           MB_OS_Q_Post()
*
* Description : This function posts to a message queue.
*
* Argument(s) : p_sem       Pointer to the message queue.
*
*               p_msg       Message to post.
*
* Return(s)   : MB_OS_ERR_NONE,   is message put in queue.
*               MB_OS_ERR_Q_FULL, if queue full.
*
* Caller(s)   : MB_OS_InitMaster()
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  CPU_INT08U  MB_OS_Q_Post (MB_OS_Q    *p_q,
                                  MODBUS_CH  *p_msg)
{
    MODBUS_CH   **p_sto;
    CPU_INT08U    err;
    CPU_SR_ALLOC();


    p_sto = p_q->OS_Q_StoPtr;
    err   = MB_OS_ERR_NONE;

    CPU_CRITICAL_ENTER();
    if ((p_q->OS_Q_MaxEntries) > (p_q->OS_Q_Entries)) {
        p_q->OS_Q_Entries++;

        if (p_q->OS_Q_LastEntry == (p_q->OS_Q_MaxEntries - 1)) {
            p_q->OS_Q_LastEntry = 0;
        } else {
            p_q->OS_Q_LastEntry++;
        }

        p_sto[p_q->OS_Q_LastEntry] = p_msg;

    } else {
        err = MB_OS_ERR_Q_FULL;
    }
    CPU_CRITICAL_EXIT();

    return (err);
}
#endif


/*
*********************************************************************************************************
*                                            MB_OS_Q_Pend()
*
* Description : This function wait until at least on message receive in the queue or a timeout ocurrs.
*
* Argument(s) : p_q         Pointer to the message queue.
*
* Return(s)   : A pointer to the message received.
*
* Caller(s)   : MB_OS_RxTask().
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
static  MODBUS_CH  *MB_OS_Q_Pend  (MB_OS_Q  *p_q)
{
    MODBUS_CH  **p_sto;
    MODBUS_CH   *p_msg;
    CPU_SR_ALLOC();


    p_sto =  p_q->OS_Q_StoPtr;
    p_msg = (MODBUS_CH *)0;

    CPU_CRITICAL_ENTER();
    if (p_q->OS_Q_Entries > 0) {
         p_msg = (p_sto[p_q->OS_Q_FirstEntry]);

         if (p_q->OS_Q_FirstEntry == (p_q->OS_Q_MaxEntries - 1)) {
             p_q->OS_Q_FirstEntry = 0;
         } else {
             p_q->OS_Q_FirstEntry++;
         }

         p_q->OS_Q_Entries--;
    }
    CPU_CRITICAL_EXIT();

    return (p_msg);
}
#endif

