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
*                                       Sharp LH79520 (ARM7)
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

#include    <includes.h>


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
    UARTREGS    *pUART;
    CPU_INT32U   lcr;


    if (pch != (MODBUS_CH *)0) {
        pch->PortNbr  = port_nbr;
        pch->BaudRate = baud;
        pch->Parity   = parity;
        pch->Bits     = bits;
        pch->Stops    = stops;
        switch (port_nbr) {                                                           /* Determine which serial port is assigned */
            case 0:
                 pUART                           = UART0;                             /* Point to UART0                          */
                 RCPC->periphclkctrl            &= ~(1 << 0);                         /* Enable the UART clock                   */
                 RCPC->periphclksel             &= ~(1 << 0);                         /* Select the internal crystal oscillator  */
                 IOCON->uartmux                 |= UARTMUX_U0RXD | UARTMUX_U0TXD;     /* Enable UART pins                        */
                 VIC->vectcntl[VIC_VECT_PORT_0]  = VIC_VECTCNTL_ENABLE | VIC_UARTINT0;
                 VIC->vectaddr[VIC_VECT_PORT_0]  = (CPU_INT32U)MB_CommRxTxISR_0_Handler;
                 VIC->intenable                  = _BIT(VIC_UARTINT0);
                 break;

            case 1:
                 pUART                           = UART1;                             /* Point to UART1                          */
                 RCPC->periphclkctrl            &= ~(1 << 1);                         /* Enable the UART clock                   */
                 RCPC->periphclksel             &= ~(1 << 1);                         /* Select the internal crystal oscillator  */
                 IOCON->uartmux                 |= UARTMUX_U1RXD | UARTMUX_U1TXD;     /* Enable UART pins                        */
                 VIC->vectcntl[VIC_VECT_PORT_1]  = VIC_VECTCNTL_ENABLE | VIC_UARTINT1;
                 VIC->vectaddr[VIC_VECT_PORT_1]  = (CPU_INT32U)MB_CommRxTxISR_1_Handler;
                 VIC->intenable                  = _BIT(VIC_UARTINT1);
                 break;

            case 2:
                 pUART                           = UART2;                             /* Point to UART2                          */
                 RCPC->periphclkctrl            &= ~(1 << 2);                         /* Enable the UART clock                   */
                 RCPC->periphclksel             &= ~(1 << 2);                         /* Select the internal crystal oscillator  */
                 IOCON->ssimux                  |= SSIMUX_UT2RXD | SSIMUX_UT2TXD;     /* Enable UART pins                        */
                 VIC->vectcntl[VIC_VECT_PORT_2]  = VIC_VECTCNTL_ENABLE | VIC_UARTINT2;
                 VIC->vectaddr[VIC_VECT_PORT_2]  = (CPU_INT32U)MB_CommRxTxISR_2_Handler;
                 VIC->intenable                  = _BIT(VIC_UARTINT2);
                 break;
        }
        pUART->cr   = 0x0000;                                         /* DISABLE the UART to reprogram it!                       */

        pUART->icr  = UARTINT_RX                                      /* Clear pending interrupts                                */
                    | UARTINT_TX
                    | UARTINT_RT
                    | UARTINT_FE
                    | UARTINT_PE
                    | UARTINT_BE
                    | UARTINT_OE;

        pUART->ifls = UARTIFLS_RX_1_8                                 /* Set FIFO interrupt levels                               */
                    | UARTIFLS_TX_1_8;

        switch (baud) {                                               /* Setup Baud Rate                                         */
            case   9600: 
                  pUART->ibrd = UARTBRINT_9600;
                  pUART->fbrd = UARTBRFRAC_9600;
                  break;

            case  19200: 
                  pUART->ibrd = UARTBRINT_19200;
                  pUART->fbrd = UARTBRFRAC_19200;
                  break;

            case  38400: 
                  pUART->ibrd = UARTBRINT_38400;
                  pUART->fbrd = UARTBRFRAC_38400;
                  break;

            case  57600: 
                  pUART->ibrd = UARTBRINT_57600;
                  pUART->fbrd = UARTBRFRAC_57600;
                  break;

            case 115200: 
                  pUART->ibrd = UARTBRINT_115200;
                  pUART->fbrd = UARTBRFRAC_115200;
                  break;

            default:                                                  /* Assume 38,400 by default                                */
                  pUART->ibrd = UARTBRINT_38400;
                  pUART->fbrd = UARTBRFRAC_38400;
                  break;
        }

        lcr = 0;                                                      /* Setup #bits, parity and stop bits                       */
        switch (bits) {
            case 7:
                 lcr |= UARTLCR_WLEN7;
                 break;

            case 8:
            default:
                 lcr |= UARTLCR_WLEN8;
                 break;
        }

        switch (parity) {
            case MODBUS_PARITY_ODD:
                 lcr |= UARTLCR_PARITY_ODD;
                 break;

            case MODBUS_PARITY_EVEN:
                 lcr |= UARTLCR_PARITY_EVEN;
                 break;

            case MODBUS_PARITY_NONE:
            default:
                 lcr |= UARTLCR_PARITY_NONE;
        }
                 
        switch (stops) {
            case 1:
                 lcr |= UARTLCR_STP1;
                 break;

            case 2:
            default:
                 lcr |= UARTLCR_STP2;
                 break;
        }

        pUART->lcr_h = lcr;

        pUART->imsc  = UARTINT_RX;                                    /* Enable Rx interrupts ONLY                               */

        pUART->cr    = UARTCR_RXE                                     /* Enable Rx, Tx and UART                                  */
                     | UARTCR_TXE                  
                     | UARTCR_ENABLE;              
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
             UART0->imsc &= ~UARTINT_RX;
             break;

        case 1:
             UART1->imsc &= ~UARTINT_RX;
             break;

        case 2:
             UART2->imsc &= ~UARTINT_RX;
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
             UART0->imsc |= UARTINT_RX;
             break;

        case 1:
             UART1->imsc |= UARTINT_RX;
             break;

        case 2:
             UART2->imsc |= UARTINT_RX;
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
    CPU_INT08U   c;
    CPU_INT08U   ch;
    MODBUS_CH   *pch;
    CPU_INT32U   mis;
    UARTREGS    *pUART;


    switch (port_nbr) {                                        /* Point to UART port data structure            */
        case 0:
             pUART = UART0;                                             
             break;

        case 1:
             pUART = UART1;
             break;

        case 2:
             pUART = UART2;
             break;
    }
    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {                   /* Find the channel assigned to this port       */
        if (pch->PortNbr == port_nbr) {
            mis   = pUART->mis;
            if (mis & UARTINT_RX) {
                c          = (CPU_INT08U)(pUART->dr & 0x00FF); /* Read the character from the UART             */
                pUART->icr = UARTINT_RX;                       /* Clear receive interrupt                      */
                pch->RxCtr++;
                MB_RxByte(pch, c);                             /* Pass character to Modbus to process          */
            }
            if (mis & UARTINT_TX) {
                pch->TxCtr++;
                MB_TxByte(pch);                                /* Send next byte                               */
                pUART->icr = UARTINT_TX;                       /* Clear transmit interrupt                     */
            }
            break;
        } else {
            pch++;
        }
    }
    pUART->icr = UARTINT_RT                                    /* Clear other 'spurious' interrupts            */
               | UARTINT_FE
               | UARTINT_PE
               | UARTINT_BE
               | UARTINT_OE;
    VIC->vectoraddr = 0x00000000L;                             /* Clear the vector address register            */
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
*                                UART #1 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_2_Handler (void)
{
    MB_CommRxTxISR_Handler(2);
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
             UART0->dr = (CPU_INT32U)c;
             break;

        case 1:
             UART1->dr = (CPU_INT32U)c;
             break;

        case 2:
             UART2->dr = (CPU_INT32U)c;
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
             UART0->imsc = UARTINT_RX;
             break;

        case 1:
             UART1->imsc = UARTINT_RX;
             break;

        case 2:
             UART2->imsc = UARTINT_RX;
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
             UART0->imsc = UARTINT_RX | UARTINT_TX;
             break;

        case 1:
             UART1->imsc = UARTINT_RX | UARTINT_TX;
             break;

        case 2:
             UART2->imsc = UARTINT_RX | UARTINT_TX;
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
    TIMER3->clear                          = 0;       /* Enable Timer #3 for Modbus RTU Timer               */
    TIMER3->load                           = BSP_LH79520_CLK / 16 / MB_RTU_Freq;
    TIMER3->control                        = TMRCTRL_ENABLE  | TMRCTRL_MODE_PERIODIC | TMRCTRL_PRESCALE16;

                                                      /* Setup the interrupt vector for the tick ISR        */
                                                      /*    Timer interrupt is a medium priority            */
    VIC->vectcntl[VIC_VECT_MODBUS_RTU_TMR] = VIC_VECTCNTL_ENABLE | VIC_TIMER3;
    VIC->vectaddr[VIC_VECT_MODBUS_RTU_TMR] = (CPU_INT32U)MB_RTU_TmrISR_Handler;
    VIC->intenable                         = _BIT(VIC_TIMER3);

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
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_TmrExit (void)
{
    TIMER3->control = TMRCTRL_DISABLE | TMRCTRL_MODE_PERIODIC | TMRCTRL_PRESCALE16;
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
    TIMER3->clear   = 0x00000000L;                    /* Clear the RTU interrupt source                     */
    VIC->vectoraddr = 0x00000000L;                    /* Clear the vector address register                  */

    MB_RTU_TmrCtr++;                                  /* Indicate that we had activities on this interrupt. */
    MB_RTU_TmrUpdate();                               /* Check for RTU timers that have expired             */
}
#endif
