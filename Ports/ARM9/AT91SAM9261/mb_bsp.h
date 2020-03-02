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
*                                     Atmel AT91SAM9261 (ARM9)
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
#define  MB_BSP_UART_DBG                       4
                                                                /* ------------------- UART COMMUNICATION MODE ------------- */
#define  MB_BSP_UART_RS232_MODE                1                
#define  MB_BSP_UART_RS485_MODE                2
                                                                /* ----------------- TIMER CONFIGURATION. RTU  MODE -------- */
#define  MB_BSP_TMR_0                          1
#define  MB_BSP_TMR_1                          2
#define  MB_BSP_TMR_2                          3



/*
*********************************************************************************************************
*                                        CONFIGURATION ERRORS
*********************************************************************************************************
*/

void  MB_CommRxTxISR_3_Handler (void);

/*
*********************************************************************************************************
*                                        CONFIGURATION ERRORS
*********************************************************************************************************
*/
                                                                /*-------------- UART MODE CONFIGURATION ERRORS ------------ */
#ifndef MB_BSP_CFG_UART_00_MODE
    #error  "MB_BSP_CFG_UART_00_MODE           not #define'd in 'app_cfg.h'          "
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "

#elif   (MB_BSP_CFG_UART_00_MODE != MB_BSP_UART_RS232_MODE  ) && \
        (MB_BSP_CFG_UART_00_MODE != MB_BSP_UART_RS485_MODE  ) 
    #error  "MB_BSP_CFG_UART_00_MODE           illegally #define'd in 'app_cfg.h'    "
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "
#endif

#ifndef MB_BSP_CFG_UART_01_MODE
    #error  "MB_BSP_CFG_UART_01_MODE           not #define'd in 'app_cfg.h'          "
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "

#elif   (MB_BSP_CFG_UART_01_MODE != MB_BSP_UART_RS232_MODE  ) && \
        (MB_BSP_CFG_UART_01_MODE != MB_BSP_UART_RS485_MODE  ) 
    #error  "MB_BSP_CFG_UART_01_MODE           illegally #define'd in 'app_cfg.h'    
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "
#endif

#ifndef MB_BSP_CFG_UART_01_MODE
    #error  "MB_BSP_CFG_UART_01_MODE           not #define'd in 'app_cfg.h'          "
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "

#elif   (MB_BSP_CFG_UART_02_MODE != MB_BSP_UART_RS232_MODE  ) && \
        (MB_BSP_CFG_UART_02_MODE != MB_BSP_UART_RS485_MODE  ) 
    #error  "MB_BSP_CFG_UART_01_MODE           illegally #define'd in 'app_cfg.h'    "
    #error  "                                  [MUST be  MB_BSP_UART_RS232_MODE   ]  "
    #error  "                                  [     ||  MB_BSP_UART_RS485_MODE   ]  "
#endif

                                                                /*-------------- TIMER SEL CONFIGURATION  ------------ */
#ifndef MS_BSP_CFG_TMR
    #error  "MS_BSP_CFG_TMR           not #define'd in 'app_cfg.h'                  "
    #error  "                                  [MUST be  MB_BSP_TMR_0   ]           "
    #error  "                                  [     ||  MB_BSP_TMR_1   ]           "
    #error  "                                  [     ||  MB_BSP_TMR_2   ]           "

#elif   (MS_BSP_CFG_TMR != MB_BSP_TMR_0 ) && \
        (MS_BSP_CFG_TMR != MB_BSP_TMR_1 ) && \
        (MS_BSP_CFG_TMR != MB_BSP_TMR_2 )            
    #error  "MS_BSP_CFG_TMR           illegally #define'd in 'app_cfg.h'          "
    #error  "                                  [MUST be  MB_BSP_TMR_0   ]         "
    #error  "                                  [     ||  MB_BSP_TMR_1   ]         "
    #error  "                                  [     ||  MB_BSP_TMR_2   ]         "

#endif



#endif
