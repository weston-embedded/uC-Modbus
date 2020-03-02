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
*                                            uC/Modbus
*
*                                    MODBUS BOARD SUPPORT PACKAGE
*                                       Philips LPC2000 (ARM7)
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
*                                            LOCAL DEFINES 
*********************************************************************************************************
*/

#define  BIT0   0x01
#define  BIT1   0x02
#define  BIT2   0x04
#define  BIT3   0x08
#define  BIT4   0x10
#define  BIT5   0x20
#define  BIT6   0x40
#define  BIT7   0x80

/*
*********************************************************************************************************
*                                             LOCAL VARIABLES
*********************************************************************************************************
*/

static  CPU_INT32U  MB_Tmr_ReloadCnts;


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

void  MB_CommExit (void)
{
    CPU_INT08U   ch;
    MODBUS_CH   *pch;


    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {
        MB_CommTxIntDis(pch);
        MB_CommRxIntDis(pch);
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

void  MB_CommPortCfg (MODBUS_CH  *pch,
                      CPU_INT08U  port_nbr,
                      CPU_INT32U  baud,
                      CPU_INT08U  bits,
                      CPU_INT08U  parity,
                      CPU_INT08U  stops)
{
    CPU_INT32U   pinsel;
    CPU_INT32U   peripheral_clk_freq;
    CPU_INT16U   div;                                                 /* Baud rate divisor                          */
    CPU_INT08U   divlo;
    CPU_INT08U   divhi;
    CPU_INT08U   lcr;                                                 /* Line Control Register                      */
    CPU_SR       cpu_sr   = 0;


    if (pch != (MODBUS_CH *)0) {
        pch->PortNbr        = port_nbr;                               /* Store configuration in channel             */
        pch->BaudRate       = baud;
        pch->Parity         = parity;
        pch->Bits           = bits;
        pch->Stops          = stops;
                                                                      /* Compute divisor for desired baud rate      */
        peripheral_clk_freq = BSP_CPU_ClkFreqPeripheral();
        div                 = (CPU_INT16U)(((2 * peripheral_clk_freq / 16 / baud) + 1) / 2);
        divlo               =  div & 0x00FF;                          /* Split divisor into LOW and HIGH bytes      */
        divhi               = (div >> 8) & 0x00FF;
        lcr                 = 0;                                      /* Setup #bits, parity and stop bits          */
        switch (bits) {
            case 7:
                 lcr |= 2 << 0;
                 break;

            case 8:
            default:
                 lcr |= 3 << 0;
                 break;
        }

        switch (parity) {
            case MODBUS_PARITY_ODD:
                 lcr |= BIT3;
                 break;

            case MODBUS_PARITY_EVEN:
                 lcr |= BIT4 | BIT3;
                 break;

            case MODBUS_PARITY_NONE:
            default:
                 break;
        }

        switch (stops) {
            case 1:
                 break;

            case 2:
            default:
                 lcr |= BIT2;
                 break;
        }

        switch (port_nbr) {                                           /* Determine which serial port is assigned    */
            case 0:
                 CPU_CRITICAL_ENTER();
				 pinsel              = PINSEL0;						  /* Enable UART0 I/Os                          */
				 pinsel             &= 0xFFFFFFF0;
				 pinsel             |= 0x00000005;
                 PINSEL0             = pinsel;
                 U0LCR               = BIT7;                          /* Set divisor access bit                     */
                 U0DLL               = divlo;                         /* Load divisor                               */
                 U0DLM               = divhi;
                 U0LCR               = lcr;                           /* Set line control register (Bit 8 is 0)     */
                 U0IER               = BIT0;                          /* Enable Rx interrupts only                  */
                 U0FCR               = 0x07;                          /* Enable FIFO, flush Rx & Tx                 */
                 CPU_CRITICAL_EXIT();
                                                                      /* VIC UART #0 Initialization                 */
                 VICIntSelect       &= ~(1 << VIC_UART0);             /* Enable interrupts                          */
                 VICVectAddr14       = (CPU_INT32U)MB_CommRxTxISR_0_Handler; /* Set the vector address              */
                 VICVectCntl14       = 0x20 | VIC_UART0;              /* Enable vectored interrupts                 */
                 VICIntEnable        =  (1 << VIC_UART0);             /* Enable Interrupts                          */
                 break;

            case 1:
                 CPU_CRITICAL_ENTER();
				 pinsel              = PINSEL0;						  /* Enable UART1 I/Os                          */
				 pinsel             &= 0xFFF0FFFF;
				 pinsel             |= 0x00050000;
                 PINSEL0             = pinsel;
                 U1LCR               = BIT7;                          /* Set divisor access bit                     */
                 U1DLL               = divlo;                         /* Load divisor                               */
                 U1DLM               = divhi;
                 U1LCR               = lcr;                           /* Set line control register (Bit 8 is 0)     */
                 U1IER               = BIT0;                          /* Enable Rx interrupts only                  */
                 U1FCR               = 0x07;                          /* Enable FIFO, flush Rx & Tx                 */
                 CPU_CRITICAL_EXIT();
                                                                      /* VIC UART #1 Initialization                 */
                 VICIntSelect       &= ~(1 << VIC_UART1);             /* Enable interrupts                          */
                 VICVectAddr14       = (CPU_INT32U)MB_CommRxTxISR_1_Handler; /* Set the vector address              */
                 VICVectCntl14       = 0x20 | VIC_UART1;              /* Enable vectored interrupts                 */
                 VICIntEnable        =  (1 << VIC_UART1);             /* Enable Interrupts                          */
        }
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

void  MB_CommRxIntDis (MODBUS_CH  *pch)
{
    CPU_SR  cpu_sr = 0;


    CPU_CRITICAL_ENTER();
    switch (pch->PortNbr) {
        case 0:
             U0IER = 0;
             break;

        case 1:
             U1IER = 0;
             break;
    }
    CPU_CRITICAL_EXIT();
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

void  MB_CommRxIntEn (MODBUS_CH  *pch)
{
    CPU_SR  cpu_sr = 0;


    CPU_CRITICAL_ENTER();
    switch (pch->PortNbr) {
        case 0:
             U0IER = BIT0;
             break;

        case 1:
             U1IER = BIT0;
             break;
    }
    CPU_CRITICAL_EXIT();
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

void  MB_CommRxTxISR_Handler (CPU_INT08U  port_nbr)
{
              CPU_INT08U   ch;
              MODBUS_CH   *pch;
    volatile  CPU_INT08U   rx_data;
    volatile  CPU_INT08U   lsr;
    volatile  CPU_INT08U   iir;


    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {       /* Find the channel assigned to this port       */
        if (pch->PortNbr == port_nbr) {
            break;
        } else {
            pch++;
        }
    }

    switch (port_nbr) {                                /* Point to UART port data structure            */
        case 0:
             iir = U0IIR & 0x0F;
             while (iir != 1) {
                 switch (iir) {
                     case  0:                          /* Modem interrupt?                             */
                          break;

                     case  2:                          /* Transmitted character?                       */
                          pch->TxCtr++;
                          MB_TxByte(pch);              /* Send next byte                               */
                          break;

                     case  4:                          /* Received a character?                        */
                          lsr     = U0LSR;
                          rx_data = U0RBR;
                          pch->RxCtr++;
                          MB_RxByte(pch, rx_data);     /* Pass character to Modbus to process          */
                          break;

                     case  6:                          /* Receive Line Status interrupt?               */
                          break;

                     case 12:                          /* CTI interrupt?                               */
                          break;
                 }
                 iir = U0IIR & 0x0F;
             }
             break;

        case 1:
             iir = U1IIR & 0x0F;
             while (iir != 1) {
                 switch (iir) {
                     case  0:                          /* Modem interrupt?                             */
                          break;

                     case  2:                          /* Transmitted character?                       */
                          pch->TxCtr++;
                          MB_TxByte(pch);              /* Send next byte                               */
                          break;

                     case  4:                          /* Received a character?                        */
                          lsr     = U1LSR;
                          rx_data = U1RBR;
                          pch->RxCtr++;
                          MB_RxByte(pch, rx_data);     /* Pass character to Modbus to process          */
                          break;

                     case  6:                          /* Receive Line Status interrupt?               */
                          break;

                     case 12:                          /* CTI interrupt?                               */
                          break;
                 }
                 iir = U1IIR & 0x0F;
             }
             break;
    }

    VICVectAddr = 0x00000000L;                         /* Clear the vector address register            */
}


/*
*********************************************************************************************************
*                                UART #0 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_0_Handler (void)
{
    MB_CommRxTxISR_Handler(0);
}


/*
*********************************************************************************************************
*                                UART #1 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_1_Handler (void)
{
    MB_CommRxTxISR_Handler(1);
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

void  MB_CommTx1 (MODBUS_CH  *pch,
                  CPU_INT08U  c)
{
    switch (pch->PortNbr) {
        case 0:
             U0THR = c;
             break;

        case 1:
             U1THR = c;
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

void  MB_CommTxIntDis (MODBUS_CH  *pch)
{
    CPU_SR  cpu_sr = 0;


    CPU_CRITICAL_ENTER();
    switch (pch->PortNbr) {                       /* Just enable the receiver interrupt                */
        case 0:
             U0IER = BIT0;
             break;

        case 1:
             U1IER = BIT0;
             break;
    }
    CPU_CRITICAL_EXIT();
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

void  MB_CommTxIntEn (MODBUS_CH  *pch)
{
    CPU_SR  cpu_sr = 0;


    CPU_CRITICAL_ENTER();
    switch (pch->PortNbr) {                       /* Just enable the receiver interrupt                */
        case 0:
             U0IER = BIT1 | BIT0;
             break;

        case 1:
             U1IER = BIT1 | BIT0;
             break;
    }
    CPU_CRITICAL_EXIT();
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
void  MB_RTU_TmrInit (void)
{
    CPU_INT32U  peripheral_clk_freq;


    peripheral_clk_freq  = BSP_CPU_ClkFreqPeripheral();
    MB_Tmr_ReloadCnts    = peripheral_clk_freq / MB_RTU_Freq;
    T1TCR                = 0;                                 /* Disable timer 1.                           */
    T1PC                 = 0;                                 /* Prescaler is set to no division.           */
    T1MR0                = T1TC + MB_Tmr_ReloadCnts;
    T1MCR                = 1;                                 /* Interrupt on MR0 (match register 0).       */
    T1CCR                = 0;                                 /* Capture is disabled.                       */
    T1EMR                = 0;                                 /* No external match output.                  */
    T1TCR                = 1;                                 /* Enable timer 1                             */

    VICIntSelect        &= ~(1 << VIC_TIMER1);                /* Enable interrupts                          */
    VICVectAddr13        = (CPU_INT32U)MB_RTU_TmrISR_Handler; /* Set the vector address                     */
    VICVectCntl13        = 0x20 | VIC_TIMER1;                 /* Enable vectored interrupts                 */
    VICIntEnable         =  (1 << VIC_TIMER1);                /* Enable Interrupts                          */

    MB_RTU_TmrResetAll();                                     /* Reset all the RTU timers, we changed freq. */
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
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_TmrExit (void)
{
    T1TCR = 0;                                                /* Disable timer 1                            */
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
void  MB_RTU_TmrISR_Handler (void)
{
    T1IR         = 0xFF;                         /* Clear timer #1 interrupt                           */
                                                 /* Reload 'relative' to current interrupt time        */
    T1MR0        = T1TC + MB_Tmr_ReloadCnts;
    VICVectAddr  = 0;

    MB_RTU_TmrCtr++;                             /* Indicate that we had activities on this interrupt. */
    MB_RTU_TmrUpdate();                          /* Check for RTU timers that have expired             */
}
#endif
