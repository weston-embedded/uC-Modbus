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
*                                        Atmel AVR32UC3A
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
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define  AVR32UC3A_USART0_BASE_ADDR    ((CPU_INT32U)0xFFFF1400)
#define  AVR32UC3A_USART1_BASE_ADDR    ((CPU_INT32U)0xFFFF1800)
#define  AVR32UC3A_USART2_BASE_ADDR    ((CPU_INT32U)0xFFFF1C00)
#define  AVR32UC3A_USART3_BASE_ADDR    ((CPU_INT32U)0xFFFF2000)

#define  AVR32UC3A_TC_BASE_ADDR        ((CPU_INT32U)0xFFFF3800)


#if (__GNUC__   && __AVR32_UC3A0512ES__) || \
    (__ICCAVR32__ && __AT32UC3A0512ES__)

#define     BSP_USART0_RXD_PIN          AVR32_USART0_RXD_0_PIN
#define     BSP_USART0_RXD_FUNCTION     AVR32_USART0_RXD_0_FUNCTION
#define     BSP_USART0_TXD_PIN          AVR32_USART0_TXD_0_PIN
#define     BSP_USART0_TXD_FUNCTION     AVR32_USART0_TXD_0_FUNCTION
#define     BSP_USART0_RTS_PIN          AVR32_USART0_RTS_0_0_PIN
#define     BSP_USART0_RTS_FUNCTION     AVR32_USART0_RTS_0_0_FUNCTION
#define     BSP_USART1_RXD_PIN          AVR32_USART1_RXD_0_PIN
#define     BSP_USART1_RXD_FUNCTION     AVR32_USART1_RXD_0_FUNCTION
#define     BSP_USART1_TXD_PIN          AVR32_USART1_TXD_0_PIN
#define     BSP_USART1_TXD_FUNCTION     AVR32_USART1_TXD_0_FUNCTION
#define     BSP_USART1_RTS_PIN          AVR32_USART1_RTS_0_0_PIN
#define     BSP_USART1_RTS_FUNCTION     AVR32_USART1_RTS_0_0_FUNCTION
#define     BSP_USART2_RXD_PIN          AVR32_USART2_RXD_0_PIN
#define     BSP_USART2_RXD_FUNCTION     AVR32_USART2_RXD_0_FUNCTION
#define     BSP_USART2_TXD_PIN          AVR32_USART2_TXD_0_PIN
#define     BSP_USART2_TXD_FUNCTION     AVR32_USART2_TXD_0_FUNCTION
#define     BSP_USART2_RTS_PIN          AVR32_USART2_RTS_0_0_PIN
#define     BSP_USART2_RTS_FUNCTION     AVR32_USART2_RTS_0_0_FUNCTION
#define     BSP_USART3_RXD_PIN          AVR32_USART3_RXD_0_PIN
#define     BSP_USART3_RXD_FUNCTION     AVR32_USART3_RXD_0_FUNCTION
#define     BSP_USART3_TXD_PIN          AVR32_USART3_TXD_0_PIN
#define     BSP_USART3_TXD_FUNCTION     AVR32_USART3_TXD_0_FUNCTION
#define     BSP_USART3_RTS_PIN          AVR32_USART3_RTS_0_0_PIN
#define     BSP_USART3_RTS_FUNCTION     AVR32_USART3_RTS_0_0_FUNCTION
#define     BSP_TC_CLK0_PIN             AVR32_TC_CLK0_0_0_PIN
#define     BSP_TC_CLK0_FUNCTION        AVR32_TC_CLK0_0_0_FUNCTION
#define     BSP_TC_A0_PIN               AVR32_TC_A0_0_0_PIN
#define     BSP_TC_A0_FUNCTION          AVR32_TC_A0_0_0_FUNCTION
#define     BSP_TC_B0_PIN               AVR32_TC_B0_0_0_PIN
#define     BSP_TC_B0_FUNCTION          AVR32_TC_B0_0_0_FUNCTION

#elif (__GNUC__   && __AVR32_UC3A0512__) || \
      (__ICCAVR32__ && __AT32UC3A0512__)

#define     BSP_USART0_RXD_PIN          AVR32_USART0_RXD_0_0_PIN
#define     BSP_USART0_RXD_FUNCTION     AVR32_USART0_RXD_0_0_FUNCTION
#define     BSP_USART0_TXD_PIN          AVR32_USART0_TXD_0_0_PIN
#define     BSP_USART0_TXD_FUNCTION     AVR32_USART0_TXD_0_0_FUNCTION
#define     BSP_USART0_RTS_PIN          AVR32_USART0_RTS_0_0_PIN
#define     BSP_USART0_RTS_FUNCTION     AVR32_USART0_RTS_0_0_FUNCTION
#define     BSP_USART1_RXD_PIN          AVR32_USART1_RXD_0_0_PIN
#define     BSP_USART1_RXD_FUNCTION     AVR32_USART1_RXD_0_0_FUNCTION
#define     BSP_USART1_TXD_PIN          AVR32_USART1_TXD_0_0_PIN
#define     BSP_USART1_TXD_FUNCTION     AVR32_USART1_TXD_0_0_FUNCTION
#define     BSP_USART1_RTS_PIN          AVR32_USART1_RTS_0_0_PIN
#define     BSP_USART1_RTS_FUNCTION     AVR32_USART1_RTS_0_0_FUNCTION
#define     BSP_USART2_RXD_PIN          AVR32_USART2_RXD_0_0_PIN
#define     BSP_USART2_RXD_FUNCTION     AVR32_USART2_RXD_0_0_FUNCTION
#define     BSP_USART2_TXD_PIN          AVR32_USART2_TXD_0_0_PIN
#define     BSP_USART2_TXD_FUNCTION     AVR32_USART2_TXD_0_0_FUNCTION
#define     BSP_USART2_RTS_PIN          AVR32_USART2_RTS_0_0_PIN
#define     BSP_USART2_RTS_FUNCTION     AVR32_USART2_RTS_0_0_FUNCTION
#define     BSP_USART3_RXD_PIN          AVR32_USART3_RXD_0_0_PIN
#define     BSP_USART3_RXD_FUNCTION     AVR32_USART3_RXD_0_0_FUNCTION
#define     BSP_USART3_TXD_PIN          AVR32_USART3_TXD_0_0_PIN
#define     BSP_USART3_TXD_FUNCTION     AVR32_USART3_TXD_0_0_FUNCTION
#define     BSP_USART3_RTS_PIN          AVR32_USART3_RTS_0_0_PIN
#define     BSP_USART3_RTS_FUNCTION     AVR32_USART3_RTS_0_0_FUNCTION
#define     BSP_TC_CLK0_PIN             AVR32_TC_CLK0_0_0_PIN
#define     BSP_TC_CLK0_FUNCTION        AVR32_TC_CLK0_0_0_FUNCTION
#define     BSP_TC_A0_PIN               AVR32_TC_A0_0_0_PIN
#define     BSP_TC_A0_FUNCTION          AVR32_TC_A0_0_0_FUNCTION
#define     BSP_TC_B0_PIN               AVR32_TC_B0_0_0_PIN
#define     BSP_TC_B0_FUNCTION          AVR32_TC_B0_0_0_FUNCTION

#else
#error bsp.c: Unknown compiler or unknown AVR32 part defined
#endif


/*
*********************************************************************************************************
*                                            AVR32UC3A REGISTER BIT DEFINES
*********************************************************************************************************
*/

                                                                /* ---------- USART MODE (MR) REGISTER BIT DEFINES --------- */
#define  AVR32UC3A_USART_MR_MODE_NORMAL           (0x00 <<  0)      
#define  AVR32UC3A_USART_MR_MODE_RS485            (0x01 <<  0)
#define  AVR32UC3A_USART_MR_MODE_IRDA             (0x08 <<  0)

#define  AVR32UC3A_USART_MR_USCLKS_CLKUSART       (0x00 <<  4)
#define  AVR32UC3A_USART_MR_USCLKS_CLKUSART_DIV   (0x01 <<  4)
#define  AVR32UC3A_USART_MR_USCLKS_CLK            (0x03 <<  4)

#define  AVR32UC3A_USART_MR_CHRL_5                (0x00 <<  6)
#define  AVR32UC3A_USART_MR_CHRL_6                (0x01 <<  6)
#define  AVR32UC3A_USART_MR_CHRL_7                (0x02 <<  6)
#define  AVR32UC3A_USART_MR_CHRL_8                (0x03 <<  6)

#define  AVR32UC3A_USART_MR_PAR_EVEN              (0x00 <<  9)
#define  AVR32UC3A_USART_MR_PAR_ODD               (0x01 <<  9)
#define  AVR32UC3A_USART_MR_PAR_FORCED0           (0x02 <<  9)
#define  AVR32UC3A_USART_MR_PAR_FORCED1           (0x03 <<  9)
#define  AVR32UC3A_USART_MR_PAR_NONE              (0x04 <<  9)
#define  AVR32UC3A_USART_MR_PAR_MULTIDROP         (0x06 <<  9)
 
#define  AVR32UC3A_USART_MR_NBSTOP_1              (0x00 << 12)
#define  AVR32UC3A_USART_MR_NBSTOP_1_5            (0x01 << 12)
#define  AVR32UC3A_USART_MR_NBSTOP_2              (0x02 << 12)

#define  AVR32UC3A_USART_MR_CHMODE_NORMAL         (0x00 << 14)
#define  AVR32UC3A_USART_MR_CHMODE_AUTOECHO       (0x01 << 14)
#define  AVR32UC3A_USART_MR_CHMODE_LOOPBACK       (0x02 << 14)
#define  AVR32UC3A_USART_MR_CHMODE_REMOTE         (0x03 << 14)
                                                                /* -------- USART CONTROL (CR) REGISTER BIT DEFINES -------- */
#define  AVR32UC3A_USART_CR_RSTRX                 DEF_BIT_02
#define  AVR32UC3A_USART_CR_RSTTX                 DEF_BIT_03
#define  AVR32UC3A_USART_CR_RXEN                  DEF_BIT_04
#define  AVR32UC3A_USART_CR_RXDIS                 DEF_BIT_05
#define  AVR32UC3A_USART_CR_TXEN                  DEF_BIT_06
#define  AVR32UC3A_USART_CR_TXDIS                 DEF_BIT_07
#define  AVR32UC3A_USART_CR_RSTSTA                DEF_BIT_08
                                                                /* -- USART CHANNEL STATUS (CSR) REGISTER BIT DEFINES --- */
#define  AVR32UC3A_USART_CSR_RXRDY                DEF_BIT_00
#define  AVR32UC3A_USART_CSR_TXRDY                DEF_BIT_01
#define  AVR32UC3A_USART_CSR_ENDRX                DEF_BIT_03
#define  AVR32UC3A_USART_CSR_ENDTX                DEF_BIT_04
#define  AVR32UC3A_USART_CSR_OVRE                 DEF_BIT_05
#define  AVR32UC3A_USART_CSR_FRAME                DEF_BIT_06
#define  AVR32UC3A_USART_CSR_PARE                 DEF_BIT_07
#define  AVR32UC3A_USART_CSR_TIMEOUT              DEF_BIT_08
#define  AVR32UC3A_USART_CSR_TXEMPTY              DEF_BIT_09
#define  AVR32UC3A_USART_CSR_ITERATION            DEF_BIT_09
#define  AVR32UC3A_USART_CSR_TXBUFE               DEF_BIT_11
#define  AVR32UC3A_USART_CSR_RXBUFF               DEF_BIT_12

#define  AVR32UC3A_USART_IDR_DIS_ALL              0xFFFFFFFF
                                                                /* --------------- TC_CMR Register Bit Defines ------------- */
#define  AVR32UC3A_TC_CMR_CPCTRG_EN               DEF_BIT_14
                                                                /* --------------- TC_IER Register Bit Defines ------------- */
#define  AVR32UC3A_TC_IER_CPCS                    DEF_BIT_04
                                                                /* ---------------- TC_SR Register Bit Defines ------------- */
#define  AVR32UC3A_TC_SR_CPCS                     DEF_BIT_04
                                                                /* ---------------- TC_CR Register Bit Defines ------------- */
#define  AVR32UC3A_TC_CCR_CLKEN                   DEF_BIT_00
#define  AVR32UC3A_TC_CCR_CLKDIS                  DEF_BIT_01
#define  AVR32UC3A_TC_CCR_SWTRG                   DEF_BIT_02
                                                                /* ---------------- US_CR Register Bit Defines ------------- */
#define  AVR32UC3A_US_RSTRX                       DEF_BIT_02
#define  AVR32UC3A_US_RSTTX                       DEF_BIT_03
#define  AVR32UC3A_US_RXEN                        DEF_BIT_04
#define  AVR32UC3A_US_RXDIS                       DEF_BIT_05
#define  AVR32UC3A_US_TXEN                        DEF_BIT_06
#define  AVR32UC3A_US_TXDIS                       DEF_BIT_07
#define  AVR32UC3A_US_RSTSTA                      DEF_BIT_08

#define AVR32_TC_SWTRG_MASK                       0x00000004
#define AVR32_TC_CLKEN_MASK                       0x00000001
/*
*********************************************************************************************************
*                                        REGISTER DEFINITIONS
*
* Note(s) : (1) The device register definition structure MUST take into account appropriate
*               register offsets and apply reserved space as required.  The registers listed
*               within the register definition structure MUST reflect the exact ordering and
*               data sizes illustrated in the device user guide.
*********************************************************************************************************
*/
                                                                /* +------------|---------------------|--------------------+ */
                                                                /* |Offset      | Register            | Name               | */
                                                                /* |------------|---------------------|--------------------| */
                                                                /* |============= UASART UNIT REGISTERS =================  | */
typedef  struct  avr32uc3a_usart {                              /* +------------|---------------------|--------------------+ */
    CPU_INT32U  CR;                                             /* |0x0000      | Control             |CR                  | */
    CPU_INT32U  MR;                                             /* |0x0004      | Mode                |MR                  | */
    CPU_INT32U  IER;                                            /* |0x0008      | Interrupt Enable    |IER                 | */
    CPU_INT32U  IDR;                                            /* |0x000C      | Interrupt Disable   |IDR                 | */                                         
    CPU_INT32U  IMR;                                            /* |0x0010      | Interrupt Mask      |IMR                 | */
    CPU_INT32U  CSR;                                            /* |0x0014      | Channel Status      |CSR                 | */
    CPU_INT32U  RHR;                                            /* |0x0018      | Receiver Holding     |RHR                 | */
    CPU_INT32U  THR;                                            /* |0x001C      | Transmitter Holding |THR                 | */
    CPU_INT32U  BRGR;                                           /* |0x0020      | Baud Rate Generator |BRGR                | */
} AVR32UC3A_USART;                                              /* +------------|---------------------|--------------------+ */
                                                                /* |============= TC CHANNEL REGISTER  ==================  | */
typedef struct avr32uc3a_tc {                                   /* +------------|---------------------|--------------------+ */
    CPU_INT32U  CCR;                                            /* |0X0000      | Channel Control     |CCR                 | */
    CPU_INT32U  CMR;                                            /* |0X0004      | Channel Mode        |CMR                 | */
    CPU_INT32U  RESERVED0;                                      /* |0X0008      | Reserved            |-------             | */
    CPU_INT32U  RESERVED1;                                      /* |0X000C      | Reserved            |-------             | */
    CPU_INT32U  CV;                                             /* |0X0010      | Counter Value       |CV                  | */
    CPU_INT32U  RA;                                             /* |0X0014      | Register A          |RA                  | */
    CPU_INT32U  RB;                                             /* |0X0018      | Register B          |RB                  | */
    CPU_INT32U  RC;                                             /* |0X001C      | Register C          |RC                  | */
    CPU_INT32U  SR;                                             /* |0X0020      | Status Register     |SR                  | */
    CPU_INT32U  IER;                                            /* |0X0024      | Interrupt Enable    |IER                 | */
    CPU_INT32U  IDR;                                            /* |0X0028      | Interrupt Disable   |IDR                 | */
    CPU_INT32U  IMR;                                            /* |0X002C      | Interrupt Mask      |IMR                 | */
} AVR32UC3A_TC;                                                 /* +------------|---------------------|--------------------+ */

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
* Description : Terminates Modbus communications.  All Modbus channels are closed.
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
* Description : Initializes the serial port to the desired baud rate and the UART will be
*               configured for N, 8, 1 (No parity, 8 bits, 1 stop).
*
* Argument(s) : pch        is a pointer to the Modbus channel
*               port_nbr   is the desired serial port number.  This argument allows you to assign a
*                          specific serial port to a specific Modbus channel.
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
              CPU_INT32U         dev_mclk_freq;
              CPU_INT32U         dev_usart_mr; 
              CPU_INT16U         dev_usart_brgr;    
    volatile  AVR32UC3A_USART   *p_dev_usart;


    if (pch != (MODBUS_CH *)0){
        pch->PortNbr        = port_nbr;                               /* Store configuration in channel              */
        pch->BaudRate       = baud;
        pch->Parity         = parity;
        pch->Bits           = bits;
        pch->Stops          = stops;        
        
                                                                      /* ------ USART CONFIGURATION SETUP ----------  */
        switch (parity) {                                             /* (1) Setup USART parity                       */
            case MODBUS_PARITY_ODD:
                 dev_usart_mr = AVR32UC3A_USART_MR_PAR_ODD;
                 break;
                 
            case MODBUS_PARITY_EVEN:            
                 dev_usart_mr = AVR32UC3A_USART_MR_PAR_EVEN;
                 break;

            case MODBUS_PARITY_NONE:
            default:
                 dev_usart_mr = AVR32UC3A_USART_MR_PAR_NONE;
                 break;
        }
        
        if (bits == 7) {                                              /* (2) Setup #bits                               */
            dev_usart_mr |= AVR32UC3A_USART_MR_CHRL_7;
        } else {
            dev_usart_mr |= AVR32UC3A_USART_MR_CHRL_8;
        }
            
        if (stops == 2) {                                             /* (3) Setup # stops bits                        */
            dev_usart_mr |= AVR32UC3A_USART_MR_NBSTOP_2;
        }
                                                                      /* (4) Setup USART baud rate                     */
        dev_mclk_freq  = BSP_PBA_ClkFreq();                            
        dev_usart_brgr = (CPU_INT16U)(dev_mclk_freq / baud / 16);
        
        switch (port_nbr) {                                           /* ----------- USART PORT SELECT --------------- */ 
            default:
            case MB_BSP_UART_00:
#if (MB_BSP_CFG_UART_00_MODE == MB_BSP_UART_RS232_MODE)
           	    p_dev_usart  = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);
                BSP_GPIO_SetFnct(BSP_USART0_RXD_PIN, BSP_USART0_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART0_TXD_PIN, BSP_USART0_TXD_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_0_Handler, AVR32_USART0_IRQ, 2);
#endif

#if (MB_BSP_CFG_UART_00_MODE == MB_BSP_UART_RS485_MODE)
                dev_usart_mr |= AVR32UC3A_USART_MR_MODE_RS485;
                p_dev_usart   = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);
                BSP_GPIO_SetFnct(BSP_USART0_RXD_PIN, BSP_USART0_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART0_TXD_PIN, BSP_USART0_TXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART0_RTS_PIN, BSP_USART0_RTS_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_0_Handler, AVR32_USART0_IRQ, 2);
#endif                 
                 break;
                 
            case MB_BSP_UART_01:
#if (MB_BSP_CFG_UART_01_MODE == MB_BSP_UART_RS232_MODE)
            	p_dev_usart  = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);
            	BSP_GPIO_SetFnct(BSP_USART1_RXD_PIN, BSP_USART1_RXD_FUNCTION);
            	BSP_GPIO_SetFnct(BSP_USART1_TXD_PIN, BSP_USART1_TXD_FUNCTION);
            	BSP_INTC_IntReg(&MB_CommRxTxISR_1_Handler, AVR32_USART1_IRQ, 2);
#endif

#if (MB_BSP_CFG_UART_01_MODE == MB_BSP_UART_RS485_MODE)
                dev_usart_mr |= AVR32UC3A_USART_MR_MODE_RS485;
                p_dev_usart   = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);
                BSP_GPIO_SetFnct(BSP_USART1_RXD_PIN, BSP_USART1_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART1_TXD_PIN, BSP_USART1_TXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART1_RTS_PIN, BSP_USART1_RTS_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_1_Handler, AVR32_USART1_IRQ, 2);
#endif                                  
                 break;
                 
            case MB_BSP_UART_02:
#if (MB_BSP_CFG_UART_02_MODE == MB_BSP_UART_RS232_MODE)
                p_dev_usart  = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);
            	BSP_GPIO_SetFnct(BSP_USART2_RXD_PIN, BSP_USART2_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART2_TXD_PIN, BSP_USART2_TXD_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_2_Handler, AVR32_USART2_IRQ, 2);
#endif

#if (MB_BSP_CFG_UART_02_MODE == MB_BSP_UART_RS485_MODE)
                dev_usart_mr |= AVR32UC3A_USART_MR_MODE_RS485;
                p_dev_usart   = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);
                BSP_GPIO_SetFnct(BSP_USART2_RXD_PIN, BSP_USART2_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART2_TXD_PIN, BSP_USART2_TXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART2_RTS_PIN, BSP_USART2_RTS_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_2_Handler, AVR32_USART2_IRQ, 2);
#endif
                 break;
                 
            case MB_BSP_UART_03:
#if (MB_BSP_CFG_UART_03_MODE == MB_BSP_UART_RS232_MODE)
                p_dev_usart  = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);
           	    BSP_GPIO_SetFnct(BSP_USART3_RXD_PIN, BSP_USART3_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART3_TXD_PIN, BSP_USART3_TXD_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_3_Handler, AVR32_USART3_IRQ, 2);
#endif
#if (MB_BSP_CFG_UART_03_MODE == MB_BSP_UART_RS485_MODE)
                dev_usart_mr |= AVR32UC3A_USART_MR_MODE_RS485;
                p_dev_usart   = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);
                BSP_GPIO_SetFnct(BSP_USART3_RXD_PIN, BSP_USART3_RXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART3_TXD_PIN, BSP_USART3_TXD_FUNCTION);
                BSP_GPIO_SetFnct(BSP_USART3_RTS_PIN, BSP_USART3_RTS_FUNCTION);
                BSP_INTC_IntReg(&MB_CommRxTxISR_3_Handler, AVR32_USART3_IRQ, 2);
#endif 
                 break;
}

        p_dev_usart->IDR     = AVR32UC3A_USART_IDR_DIS_ALL;

        p_dev_usart->CR      = AVR32UC3A_USART_CR_RSTSTA      /* Reset status bits, Tx and Rx                              */
                             | AVR32UC3A_USART_CR_RSTRX
                             | AVR32UC3A_USART_CR_RSTTX;

        p_dev_usart->CR      = AVR32UC3A_USART_CR_RXEN        /* Enable receiver and transmitter                           */
                             | AVR32UC3A_USART_CR_TXEN;
        
        p_dev_usart->IER     = AVR32UC3A_USART_CSR_RXRDY;     /* Enable Rx interrupts                                      */

        p_dev_usart->MR      = dev_usart_mr;                  /* Set the mode register                                     */
        p_dev_usart->BRGR    = dev_usart_brgr;                /* Set the baud rate generator register                      */
    }
}


 
/*
*********************************************************************************************************
*                                         MB_CommRxIntDis()
*
* Description : Disables Rx interrupts.
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
    volatile  AVR32UC3A_USART  *p_dev_usart;  
    
    switch (pch->PortNbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }
    
    p_dev_usart->IDR = AVR32UC3A_USART_CSR_RXRDY;
}

 
/*
*********************************************************************************************************
*                                          MB_CommRxIntEn()
*
* Description : Enables Rx interrupts.
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
    volatile AVR32UC3A_USART  *p_dev_usart;  
    
    switch (pch->PortNbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }

    p_dev_usart->IER = AVR32UC3A_USART_CSR_RXRDY;
}

 
/*
*********************************************************************************************************
*                                       MB_CommRxTxISR_Handler()
*
* Description : ISR for either a received or transmitted character.
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
    volatile  AVR32UC3A_USART  *p_dev_usart;  
              MODBUS_CH          *pch;
              CPU_INT08U          ch;
              CPU_INT08U          rx_data;

    
    pch                = &MB_ChTbl[0];
    
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {       /* Find the channel assigned to this port       */
        if (pch->PortNbr == port_nbr) {
            ch = MODBUS_CFG_MAX_CH;
        } else {
            pch++;
        }
    }
    
    switch (port_nbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }

    if ((DEF_BIT_IS_SET(p_dev_usart->IMR, AVR32UC3A_USART_CSR_RXRDY)) && \
        (DEF_BIT_IS_SET(p_dev_usart->CSR, AVR32UC3A_USART_CSR_RXRDY))) {
    
        pch->RxCtr++;                                           /* Increment the Rx counter                                  */
        rx_data = (CPU_INT08U)(p_dev_usart->RHR & 0x00FF);
        MB_RxByte(pch, rx_data);
    }
    
    if ((DEF_BIT_IS_SET(p_dev_usart->IMR, AVR32UC3A_USART_CSR_TXRDY)) && \
        (DEF_BIT_IS_SET(p_dev_usart->CSR, AVR32UC3A_USART_CSR_TXRDY))) {
    
        pch->TxCtr++;
        MB_TxByte(pch);                                         /* Send next byte                                            */
    }
    
    if ((DEF_BIT_IS_SET(p_dev_usart->IMR, AVR32UC3A_USART_CSR_OVRE))) {   
        p_dev_usart->CR = AVR32UC3A_USART_CR_RSTSTA;           /* if an overrun occurs, reset the status bits                */
    }
}

 
/*
*********************************************************************************************************
*                                UART #0 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_0_Handler (void)
{
    MB_CommRxTxISR_Handler(MB_BSP_UART_00);
}


/*
*********************************************************************************************************
*                                UART #1 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_1_Handler (void)
{
    MB_CommRxTxISR_Handler(MB_BSP_UART_01);
}

/*
*********************************************************************************************************
*                                UART #2 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_2_Handler (void)
{
    MB_CommRxTxISR_Handler(MB_BSP_UART_02);
}

/*
*********************************************************************************************************
*                                UART #3 Rx/Tx Communication handler for Modbus
*********************************************************************************************************
*/

void  MB_CommRxTxISR_3_Handler (void)
{
    MB_CommRxTxISR_Handler(MB_BSP_UART_03);
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
    volatile  AVR32UC3A_USART  *p_dev_usart;  

    
    switch (pch->PortNbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }

    p_dev_usart->THR = c;
}

 
/*
*********************************************************************************************************
*                                         MB_CommTxIntDis()
*
* Description : Disables Tx interrupts.
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
    volatile  AVR32UC3A_USART  *p_dev_usart;  

    
    switch (pch->PortNbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }

    p_dev_usart->IDR = AVR32UC3A_USART_CSR_TXRDY;
}

 
/*
*********************************************************************************************************
*                                         MB_CommTxIntEn()
*
* Description : Enables Tx interrupts.
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
    volatile  AVR32UC3A_USART  *p_dev_usart;  

    
    switch (pch->PortNbr) {
        default:
        case MB_BSP_UART_00:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART0_BASE_ADDR);;
             break;

        case MB_BSP_UART_01:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART1_BASE_ADDR);;
             break;
             
        case MB_BSP_UART_02:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART2_BASE_ADDR);;
             break;

        case MB_BSP_UART_03:
             p_dev_usart = (AVR32UC3A_USART *)(AVR32UC3A_USART3_BASE_ADDR);;                          
             break;
    }

    p_dev_usart->IER = AVR32UC3A_USART_CSR_TXRDY;
}


 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrInit()
*
* Description : Initializes RTU timeout timer.
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
              CPU_INT32U       dev_mclk_freq;
    volatile  AVR32UC3A_TC   *p_dev_tc;


    dev_mclk_freq        = BSP_PBA_ClkFreq();
    MB_Tmr_ReloadCnts    = (dev_mclk_freq / 2) / MB_RTU_Freq;

#if   (MS_BSP_CFG_TMR == MB_BSP_TMR_0)
    p_dev_tc = (AVR32UC3A_TC *)(AVR32UC3A_TC_BASE_ADDR);
#endif
    
    p_dev_tc->CCR     = AVR32UC3A_TC_CCR_CLKDIS;              /* TCx Clk disable                                             */
    p_dev_tc->CMR     = AVR32UC3A_TC_CMR_CPCTRG_EN;           /* TCx Compare trigger enable                                  */
    p_dev_tc->RC      = MB_Tmr_ReloadCnts;                    /* TCx RC compare trigger value                                */

    
    p_dev_tc->CCR     = AVR32UC3A_TC_CCR_CLKEN;               /* TCx clk enable                                              */            
    p_dev_tc->CCR     = AVR32UC3A_TC_CCR_SWTRG;               /* TCx Software trigger                                        */ 
    p_dev_tc->IER     = AVR32UC3A_TC_IER_CPCS;                /* TCx RC compare interrupt enable                             */ 


    MB_RTU_TmrResetAll();                                     /* Reset all the RTU timers.                                   */
}
#endif


 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrExit()
*
* Description : Disables RTU timeout timer.
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
    volatile  AVR32UC3A_TC   *p_dev_tc;
   
#if   (MS_BSP_CFG_TMR == MB_BSP_TMR_0)
    p_dev_tc = (AVR32UC3A_TC *)(AVR32UC3A_TC_BASE_ADDR);    
#endif

    p_dev_tc->CCR = AVR32UC3A_TC_CCR_CLKDIS;    
}
#endif

 
/*
*********************************************************************************************************
*                                       MB_RTU_TmrISR_Handler()
*
* Description : Handles the case when the RTU timeout timer expires.
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
    volatile  AVR32UC3A_TC   *p_dev_tc;
    

#if   (MS_BSP_CFG_TMR == MB_BSP_TMR_0)
    p_dev_tc = (AVR32UC3A_TC *)(AVR32UC3A_TC_BASE_ADDR);    
#endif

    if (DEF_BIT_IS_SET(p_dev_tc->SR , AVR32UC3A_TC_SR_CPCS)) {
        p_dev_tc->RC     = MB_Tmr_ReloadCnts;                 /* Load the match value to the RC register                     */
    }

    MB_RTU_TmrCtr++;                                          /* Indicate that we had activities on this interrupt.          */
    MB_RTU_TmrUpdate();                                       /* Check for RTU timers that have expired                      */
}

#endif
