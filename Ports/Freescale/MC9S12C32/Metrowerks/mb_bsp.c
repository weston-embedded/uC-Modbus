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
*                                              uC/Modbus
*
*                                     MODBUS BOARD SUPPORT PACKAGE
*                                         Freescale MC9S12C32
*
*
* Filename : mb_bsp.c
* Version  : V2.14.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>

/*
*********************************************************************************************************
*                                               GLOBALS
*********************************************************************************************************
*/     


 
/*
*********************************************************************************************************
*                                             MB_CommExit()
*
* Description : This function is called to terminate Modbus communications.  All Modbus channels are close.
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : MB_Exit()
*
* Note(s)     : none.
*********************************************************************************************************
*/


CPU_VOID  MB_CommExit (CPU_VOID)
{
    CPU_INT08U   ch;
    MODBUS_CH   *pch;   


    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {    
        MBS_CommTxIntDis(pch);
        MBS_CommRxIntDis(pch);
        pch++;
    }
}

 
/*
*********************************************************************************************************
*                                           MB_CommPortCfg()
*
* Description : This function initializes the serial port to the desired baud rate and the UART will be
*               configured for N, 8, 1 (No parity, 8 bits, 1 stop).
*
* Argument(s) : pch        is a pointer to the Modbus channel
*               port_nbr   is the desired serial port number.  This argument allows you to assign a
*                          specific serial port to a sepcific Modbus channel.
*               baud       is the desired baud rate for the serial port.
*               parity     is the desired parity and can be either:
*
*                          MODBUS_PARITY_NONE
*                          MODBUS_PARITY_ODD
*                          MODBUS_PARITY_EVEN
*
*               bits       specifies the number of bit and can be either 7 or 8.
*               stops      specifies the number of stop bits and can either be 1 or 2
*
* Return(s)   : none.
*
* Caller(s)   : MB_CfgCh()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_VOID  MB_CommPortCfg (CPU_INT08U  ch, 
                          CPU_INT08U  port_nbr, 
                          CPU_INT32U  baud, 
                          CPU_INT08U  bits, 
                          CPU_INT08U  parity, 
                          CPU_INT08U  stops)
{
    MODBUS_CH   *pch;
    CPU_INT08U   reg_val;


    if (ch < MODBUS_CFG_MAX_CH) {
        pch           = &MB_ChTbl[ch];
        pch->PortNbr  = port_nbr;
        pch->BaudRate = baud;
        pch->Parity   = parity;
        pch->Bits     = bits;
        pch->Stops    = stops;

        SCI_RxDis();                                                  /* Disable the SCI Receiver to reprogram it                */
        SCI_TxDis();                                                  /* Disable the SCI Transmitter to reprogram it             */

        reg_val = SCISR1;                                             /* Clear pending interrupts                                */
        reg_val = SCIDRL;                                             /* Clear pending interrupts                                */        
        (void)reg_val;                                                /* Eliminate compiler warning: reg_val not used            */
        
        switch (baud) {                                               /* Setup Baud Rate                                         */
            case   9600: 
                  SCI_SetBaud(9600);                                  
                  break;

            case  19200: 
                  SCI_SetBaud(19200);                                  
                  break;

            case  38400: 
                  SCI_SetBaud(38400);                                  
                  break;

            case  57600: 
                  SCI_SetBaud(57600);                                  
                  break;

            case 115200: 
                  SCI_SetBaud(115200);                                  
                  break;
                  
            default:                                                  /* Assume 38,400 by default                                */
                  SCI_SetBaud(38400);                                  
                  break;
        }

        switch (bits) {
            case 8:
                 SCI_SetDataBits(8);                                 
                 break;

            case 9:
                 SCI_SetDataBits(9);                                 
            
            default:
                 SCI_SetDataBits(8);                                 
                 break;
        }

        switch (parity) {
            case MODBUS_PARITY_ODD:
                 SCI_SetParity(SCI_PARITY_ODD);
                 break;

            case MODBUS_PARITY_EVEN:
                 SCI_SetParity(SCI_PARITY_EVEN);
                 break;

            case MODBUS_PARITY_NONE:
                 SCI_SetParity(SCI_PARITY_NONE);
                 break;
                 
            default:
                 SCI_SetParity(SCI_PARITY_NONE);
                 break;
        }
                 
        SCI_RxIntEn();                                                /* Enable Rx interrupts ONLY                               */

        SCI_RxEn();                                                   /* Enable the SCI Receiver                                 */
        SCI_TxEn();                                                   /* Enable the SCI Transmitter                              */
    }
}


 
/*
*********************************************************************************************************
*                                         MB_CommRxIntDis()
*
* Description : This function disables Rx interrupts.
*
* Argument(s) : pch        is a pointer to the Modbus channel
*
* Return(s)   : none.
*
* Caller(s)   : MB_CommExit()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_VOID  MBS_CommRxIntDis (MODBUS_CH  *pch)
{
#if OS_CRITICAL_METHOD == 3
    OS_CPU_SR  cpu_sr;


    cpu_sr = 0;
#endif
    OS_ENTER_CRITICAL();
    switch (pch->PortNbr) {
        case 0:
             SCI_RxIntDis();
             break;
        
        default:
             break;
    }
    OS_EXIT_CRITICAL();
}

 
/*
*********************************************************************************************************
*                                          MB_CommRxIntEn()
*
* Description : This function enables Rx interrupts.
*
* Argument(s) : pch        is a pointer to the Modbus channel
*
* Return(s)   : none.
*
* Caller(s)   : MB_TxByte()
*
* Note(s)     : none.
*********************************************************************************************************
*/
CPU_VOID  MBS_CommRxIntEn (MODBUS_CH  *pch)
{
#if OS_CRITICAL_METHOD == 3
    OS_CPU_SR  cpu_sr;


    cpu_sr = 0;
#endif
    OS_ENTER_CRITICAL();
    switch (pch->PortNbr) {
        case 0:
             SCI_RxIntEn();
             break;
             
        default:
             break;
    }
    OS_EXIT_CRITICAL();
}

 
/*
*********************************************************************************************************
*                                       MB_CommRxTxISR_Handler()
*
* Description : This function is the ISR for either a received or transmitted character.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is a ISR
*
* Note(s)     : (1) The pseudo-code for this function should be:  
*
*               if (Rx Byte has been received) {
*                  c = get byte from serial port;
*                  Clear receive interrupt;
*                  pch->RxCtr++;                      Increment the number of bytes received
*                  MB_RxByte(pch, c);                 Pass character to Modbus to process
*              }
*
*              if (Byte has been transmitted) {
*                  pch->TxCtr++;                      Increment the number of bytes transmitted
*                  MB_TxByte(pch);                    Send next byte in response
*                  Clear transmit interrupt           Clear Transmit Interrupt flag
*              }
*********************************************************************************************************
*/

CPU_VOID  MBS_CommRxTxISR_Handler (CPU_INT08U  port_nbr)
{						 
    CPU_INT08U   c;
    CPU_INT08U   ch;
    MODBUS_CH   *pch;
    CPU_INT32U   mis;
    CPU_INT08U   SCI_Status;

    
    (CPU_VOID)port_nbr;
    (CPU_VOID)ch;
    (CPU_VOID)mis;
        
    SCI_Status     = SCISR1;

    pch = &MB_ChTbl[0];

    if (SCI_Status & SCI_RX_COMPLETE) {
        c          = (CPU_INT08U)(SCIDRL);             /* Read the character from the UART             */
        pch->RxCtr++;
        MBS_RxByte(pch, c);                            /* Pass character to Modbus to process          */
    }
    
    if (SCI_Status & SCI_TX_COMPLETE) {	        
        pch->TxCtr++;
        MBS_TxByte(pch);                               /* Send next byte in response                   */        
    }    
}

 
/*
*********************************************************************************************************
*                                UART #0 Rx/Tx Communication handler for Modbus 
*                                        (THIS IS THE START OF THE ISR!)
*********************************************************************************************************
*/

interrupt CPU_VOID  MBS_CommRxTxISR_0_Handler (CPU_VOID)
{
    MBS_CommRxTxISR_Handler(0);
}


 
/*
*********************************************************************************************************
*                                             MB_CommTx1()
*
* Description : This function is called to obtain the next byte to send from the transmit buffer.  When
*               all bytes in the reply have been sent, transmit interrupts are disabled and the receiver
*               is enabled to accept the next Modbus request.
*
* Argument(s) : c     is the byte to send to the serial port
*
* Return(s)   : none.
*
* Caller(s)   : MB_TxByte()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_VOID  MBS_CommTx1 (MODBUS_CH  *pch, 
                   CPU_INT08U  c)
{
    switch (pch->PortNbr) {
        case 0:
             SCI_Tx1(c);
             break;
             
        default:
             break;
    }
}


 
/*
*********************************************************************************************************
*                                         MB_CommTxIntDis()
*
* Description : This function disables Tx interrupts.
*
* Argument(s) : pch        is a pointer to the Modbus channel
*
* Return(s)   : none.
*
* Caller(s)   : MB_CommExit()
*               MB_TxByte()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_VOID  MBS_CommTxIntDis (MODBUS_CH  *pch)
{									  
#if OS_CRITICAL_METHOD == 3
    OS_CPU_SR  cpu_sr;


    cpu_sr = 0;
#endif

    (CPU_VOID)pch;
    
    OS_ENTER_CRITICAL();
    SCI_TxIntDis();                               /* Enable SCI Receive Interrupts                     */
    OS_EXIT_CRITICAL();
}

 
/*
*********************************************************************************************************
*                                         MB_CommTxIntEn()
*
* Description : This function enables Tx interrupts.
*
* Argument(s) : pch        is a pointer to the Modbus channel
*
* Return(s)   : none.
*
* Caller(s)   : MB_Tx()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_VOID  MBS_CommTxIntEn (MODBUS_CH  *pch)
{
#if OS_CRITICAL_METHOD == 3
    OS_CPU_SR  cpu_sr;


    cpu_sr = 0;
#endif

    (CPU_VOID)pch;
    
    OS_ENTER_CRITICAL();
    SCI_TxIntEn();                                /* Enable SCI Rx Interrupts                          */
    OS_EXIT_CRITICAL();
}

 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrInit()
*
* Description : This function is called to initialize the RTU timeout timer.
*
* Argument(s) : freq          Is the frequency of the modbus RTU timer interrupt.
*
* Return(s)   : none.
*
* Caller(s)   : MB_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_VOID  MB_RTU_TmrInit (void)
{						
    ECT_ChInt_Init(MB_ECT_CH, MB_RTU_Freq);           /* Use ECT Channel selected in app.cfg at freq hz     */
                                                      /* for the RTU timer                                  */

    MB_RTU_TmrResetAll();                             /* Reset all the RTU timers, we changed freq.         */
}
#endif

 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrExit()
*
* Description : This function is called to disable the RTU timeout timer.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_Exit()
*
* Note(s)     : none.
*********************************************************************************************************
*/s

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_VOID  MB_RTU_TmrExit (CPU_VOID)
{    
    ECT_IntDis(MB_ECT_CH);                            /* Disable the MB ECT Timer Channel Interrupts        */                              
}
#endif


 
/*
*********************************************************************************************************
*                                       MB_RTU_TmrISR_Handler()
*
* Description : This function handles the case when the RTU timeout timer expires.
*
* Arguments   : none.
*
* Returns     : none.
*
* Caller(s)   : This is a ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
interrupt CPU_VOID  MB_RTU_Tmr_Timeout (CPU_VOID)
{
    TFLG1    |= (1 << MB_ECT_CH);                     /* Clear the ECT interrupt flag for the channel used  */
    MB_RTU_TmrCtr++;                                  /* Indicate that we had activities on this interrupt. */
    MB_RTU_TmrUpdate();                               /* Check for RTU timers that have expired             */
}
#endif

