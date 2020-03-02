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
*                                      Board Support Package
*                                     Atmel AT91SAM9XE (ARM9)
*
* Filename : mb_bsp.h
* Version  : V2.14.00
*********************************************************************************************************
*/

#ifndef  __MB_BSP_H__
#define  __MB_BSP_H__


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
                                                                /* ------------------ UART PORT NUMBERS DEFINES ------------ */
#define  MB_BSP_UART_00                        1
#define  MB_BSP_UART_01                        2
#define  MB_BSP_UART_02                        3
#define  MB_BSP_UART_03                        4
#define  MB_BSP_UART_04                        5
#define  MB_BSP_UART_DBG                       6
                                                                /* ------------------- UART COMMUNICATION MODE ------------- */
#define  MB_BSP_UART_RS232_MODE                1                
#define  MB_BSP_UART_RS485_MODE                2
                                                                /* ----------------- TIMER CONFIGURATION. RTU  MODE -------- */
#define  MB_BSP_TMR_0                          1
#define  MB_BSP_TMR_1                          2
#define  MB_BSP_TMR_2                          3
#define  MB_BSP_TMR_3                          4
#define  MB_BSP_TMR_4                          5
#define  MB_BSP_TMR_5                          6


void  MB_CommRxTxISR_3_Handler (void);


#endif
