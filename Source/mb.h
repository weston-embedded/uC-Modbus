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
*                                       uC/MODBUS Header File
*
* Filename : mb.h
* Version  : V2.14.00
*********************************************************************************************************
*/

 
/*
*********************************************************************************************************
*                                               MODULE
*
* Note(s) : (1) This main modbus header file is protected from multiple pre-processor
*               inclusion through use of the modbus module present pre-processor macro definition.
*
*********************************************************************************************************
*/

#ifndef  MODBUS_MODULE_PRESENT                                  /* See Note #1.                                         */
#define  MODBUS_MODULE_PRESENT


 
/*
*********************************************************************************************************
*                                       MODBUS VERSION NUMBER
*
* Note(s) : (1) (a) The Modbus software version is denoted as follows :
*
*                       Vx.yy.zz
*
*                           where
*                                   V               denotes 'Version' label
*                                   x               denotes major software version revision number
*                                   yy              denotes minor software version revision number
*                                   zz              minor revision
*
*               (b) The software version label #define is formatted as follows :
*
*                       ver = x.yy.zz * 10000
*
*                           where
*                                   ver             denotes software version number scaled as an integer value
*                                   x.yy.zz         denotes software version number
*********************************************************************************************************
*/

#define  MODBUS_VERSION                                  21400u   /* See Note #1.                      */


 
/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef    MB_MODULE
#define   MB_EXT
#else
#define   MB_EXT  extern
#endif


 
/*
*********************************************************************************************************
*                                        MODBUS PROTOCOL INCLUDE FILES
*
* Note(s) : (1) The modbus protocol suite files are located in the following directories :
*
*               (a) (1) \<Your Product Application>\app_cfg.h
*                   (2)                            \mb_cfg.h
*                   (3)                            \mb_data.c
*
*               (b) \<Modbus Protocol Suite>\Source\mb.h
*                                                  \mb.c
*                                                  \mb_def.c
*                                                  \mb_util.c
*                                                  \mbm_core.c
*                                                  \mbs_core.c
*
*               (c) \<Modbus Protocol Suite>\Ports\<cpu>\mb_bsp.*
*
*               (d) \<Modbus Protocol Suite>\OS\<os>\mb_os.*
*
*                       where
*                               <Your Product Application>      directory path for Your Product's Application
*                               <Modbus Protocol Suite>         directory path for modbus protocol suite
*                               <cpu>                           directory name for specific processor              (CPU)
*                               <compiler>                      directory name for specific compiler
*                               <os>                            directory name for specific operating system       (OS)
*
*           (2) CPU-configuration software files are located in the following directories :
*
*               (a) \<CPU-Compiler Directory>\cpu_def.h
*
*               (b) \<CPU-Compiler Directory>\<cpu>\<compiler>\cpu*.*
*
*                       where
*                               <CPU-Compiler Directory>        directory path for common   CPU-compiler software
*                               <cpu>                           directory name for specific processor (CPU)
*                               <compiler>                      directory name for specific compiler
*
*           (3) NO compiler-supplied standard library functions are used by the modbus protocol suite.
*
*               (a) Standard library functions are implemented in the custom library module(s) :
*
*                       \<Custom Library Directory>\lib*.*
*
*                           where
*                                   <Custom Library Directory>      directory path for custom library software
*
*
*           (4) Compiler MUST be configured to include the '\<Custom Library Directory>\uC-LIB\',
*               '\<CPU-Compiler Directory>\' directory, & the specific CPU-compiler directory as
*               additional include path directories.
**********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <app_cfg.h>

#include  <mb_cfg.h>
#include  <mb_def.h>

#include  <mb_os.h>


 
/*
*********************************************************************************************************
*                                               DATA TYPES
*********************************************************************************************************
*/

typedef  struct  modbus_ch {
    CPU_INT08U       Ch;                               /* Channel number                                                   */
    CPU_BOOLEAN      WrEn;                             /* Indicates whether MODBUS writes are enabled for the channel      */
    CPU_INT32U       WrCtr;                            /* Incremented each time a write command is performed               */

    CPU_INT08U       NodeAddr;                         /* Modbus node address of the channel                               */

    CPU_INT08U       PortNbr;                          /* UART port number                                                 */
    CPU_INT32U       BaudRate;                         /* Baud Rate                                                        */
    CPU_INT08U       Parity;                           /* UART's parity settings (MODBUS_PARITY_NONE, _ODD or _EVEN)       */
    CPU_INT08U       Bits;                             /* UART's number of bits (7 or 8)                                   */
    CPU_INT08U       Stops;                            /* UART's number of stop bits (1 or 2)                              */

    CPU_INT08U       Mode;                             /* Modbus mode: MODBUS_MODE_ASCII or MODBUS_MODE_RTU                */

    CPU_INT08U       MasterSlave;                      /* Slave when set to MODBUS_SLAVE, Master when set to MODBUS_MASTER */

    CPU_INT16U       Err;                              /* Internal code to indicate the source of MBS_ErrRespSet()         */

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
    CPU_INT16U       RTU_TimeoutCnts;                  /* Counts to reload in .RTU_TimeoutCtr when byte received           */
    CPU_INT16U       RTU_TimeoutCtr;                   /* Counts left before RTU timer times out for the channel           */
    CPU_BOOLEAN      RTU_TimeoutEn;                    /* Enable (when TRUE) or Disable (when FALSE) RTU timer             */
#endif

#if (MODBUS_CFG_FC08_EN == DEF_ENABLED)
    CPU_INT16U       StatMsgCtr;                       /* Statistics                                                       */
    CPU_INT16U       StatCRCErrCtr;
    CPU_INT16U       StatExceptCtr;
    CPU_INT16U       StatSlaveMsgCtr;
    CPU_INT16U       StatNoRespCtr;
#endif

    CPU_INT32U       RxTimeout;                        /* Amount of time Master is willing to wait for response from slave */

    CPU_INT32U       RxCtr;                            /* Incremented every time a character is received                   */
    CPU_INT16U       RxBufByteCtr;                     /* Number of bytes received or to send                              */
    CPU_INT08U      *RxBufPtr;                         /* Pointer to current position in buffer                            */
    CPU_INT08U       RxBuf[MODBUS_CFG_BUF_SIZE];       /* Storage of received characters or characters to send             */

    CPU_INT32U       TxCtr;                            /* Incremented every time a character is transmitted                */
    CPU_INT16U       TxBufByteCtr;                     /* Number of bytes received or to send                              */
    CPU_INT08U      *TxBufPtr;                         /* Pointer to current position in buffer                            */
    CPU_INT08U       TxBuf[MODBUS_CFG_BUF_SIZE];       /* Storage of received characters or characters to send             */

    CPU_INT08U       RxFrameData[MODBUS_CFG_BUF_SIZE]; /* Additional data for function requested.                          */
    CPU_INT16U       RxFrameNDataBytes;                /* Number of bytes in the data field.                               */
    CPU_INT16U       RxFrameCRC;                       /* Error check value (LRC or CRC-16).                               */
    CPU_INT16U       RxFrameCRC_Calc;                  /* Error check value computed from packet received                  */

    CPU_INT08U       TxFrameData[MODBUS_CFG_BUF_SIZE]; /* Additional data for function requested.                          */
    CPU_INT16U       TxFrameNDataBytes;                /* Number of bytes in the data field.                               */
    CPU_INT16U       TxFrameCRC;                       /* Error check value (LRC or CRC-16).                               */
} MODBUS_CH;

 
/*
*********************************************************************************************************
*                                           GLOBAL VARIABLES
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
MB_EXT   CPU_INT16U      MB_RTU_Freq;                  /* Frequency at which RTU timer is running                          */
MB_EXT   CPU_INT32U      MB_RTU_TmrCtr;                /* Incremented every Modbus RTU timer interrupt                     */
#endif

MB_EXT   CPU_INT08U      MB_ChCtr;                     /* Modbus channel counter (0..MODBUS_MAX_CH)                        */
MB_EXT   MODBUS_CH       MB_ChTbl[MODBUS_CFG_MAX_CH];  /* Modbus channels                                                  */

/*
*********************************************************************************************************
*                                           GLOBAL VARIABLES
*********************************************************************************************************
*/

extern  CPU_INT32U  const  MB_TotalRAMSize;
extern  CPU_INT16U  const  MB_ChSize;

/*
*********************************************************************************************************
*                                  MODBUS INTERFACE FUNCTION PROTOTYPES
*                                               (MB.C)
*********************************************************************************************************
*/

void          MB_Init                   (CPU_INT32U  freq);

void          MB_Exit                   (void);

MODBUS_CH    *MB_CfgCh                  (CPU_INT08U  node_addr,
                                         CPU_INT08U  master_slave,
                                         CPU_INT32U  rx_timeout,
                                         CPU_INT08U  modbus_mode,
                                         CPU_INT08U  port_nbr,
                                         CPU_INT32U  baud,
                                         CPU_INT08U  bits,
                                         CPU_INT08U  parity,
                                         CPU_INT08U  stops,
                                         CPU_INT08U  wr_en);

void          MB_MasterTimeoutSet       (MODBUS_CH  *pch,
                                         CPU_INT32U  timeout);

void          MB_ModeSet                (MODBUS_CH  *pch,
                                         CPU_INT08U  master_slave,
                                         CPU_INT08U  mode);

void          MB_NodeAddrSet            (MODBUS_CH  *pch,
                                         CPU_INT08U  addr);

void          MB_WrEnSet                (MODBUS_CH  *pch,
                                         CPU_INT08U  wr_en);

void          MB_ChToPortMap            (MODBUS_CH  *pch,
                                         CPU_INT08U  port_nbr);

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
void          MB_ASCII_RxByte           (MODBUS_CH   *pch,
                                         CPU_INT08U   rx_byte);
#endif

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void          MB_RTU_RxByte             (MODBUS_CH   *pch,
                                         CPU_INT08U   rx_byte);

void          MB_RTU_TmrReset           (MODBUS_CH   *pch);       /* Resets the Frame Sync timer.                                 */

void          MB_RTU_TmrResetAll        (void);                   /* Resets all the RTU timers                                    */

void          MB_RTU_TmrUpdate          (void);
#endif

void          MB_RxByte                 (MODBUS_CH   *pch,
                                         CPU_INT08U   rx_byte);

void          MB_RxTask                 (MODBUS_CH   *pch);

void          MB_Tx                     (MODBUS_CH   *pch);

void          MB_TxByte                 (MODBUS_CH   *pch);

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_ASCII_Rx                (MODBUS_CH   *pch);
void         MB_ASCII_Tx                (MODBUS_CH   *pch);
#endif


#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_RTU_Rx                  (MODBUS_CH   *pch);
void         MB_RTU_Tx                  (MODBUS_CH   *pch);
#endif

/*
*********************************************************************************************************
*                                   RTOS INTERFACE FUNCTION PROTOTYPES
*                                         (defined in mb_os.c)
*********************************************************************************************************
*/

void          MB_OS_Init                (void);

void          MB_OS_Exit                (void);

void          MB_OS_RxSignal            (MODBUS_CH   *pch);

void          MB_OS_RxWait              (MODBUS_CH   *pch,
                                         CPU_INT16U  *perr);

/*
*********************************************************************************************************
*                            COMMON MODBUS ASCII INTERFACE FUNCTION PROTOTYPES
*                                       (defined in mb_util.c)
*********************************************************************************************************
*/

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
CPU_INT08U   *MB_ASCII_BinToHex         (CPU_INT08U   value,
                                         CPU_INT08U  *pbuf);

CPU_INT08U    MB_ASCII_HexToBin         (CPU_INT08U  *phex);

CPU_INT08U    MB_ASCII_RxCalcLRC        (MODBUS_CH   *pch);

CPU_INT08U    MB_ASCII_TxCalcLRC        (MODBUS_CH   *pch,
                                         CPU_INT16U   tx_bytes);
#endif

/*
*********************************************************************************************************
*                             COMMON MODBUS RTU INTERFACE FUNCTION PROTOTYPES
*                                       (defined in mb_util.C)
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_INT16U   MB_RTU_CalcCRC             (MODBUS_CH  *pch);

CPU_INT16U   MB_RTU_TxCalcCRC           (MODBUS_CH  *pch);

CPU_INT16U   MB_RTU_RxCalcCRC           (MODBUS_CH  *pch);
#endif

/*
*********************************************************************************************************
*                                    INTERFACE TO APPLICATION DATA
*                                       (defined in mb_data.C)
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC01_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_CoilRd                  (CPU_INT16U   coil,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC05_EN == DEF_ENABLED)
void         MB_CoilWr                  (CPU_INT16U   coil,
                                         CPU_BOOLEAN  coil_val,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC02_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_DIRd                    (CPU_INT16U   di,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC04_EN == DEF_ENABLED)
CPU_INT16U   MB_InRegRd                 (CPU_INT16U   reg,
                                         CPU_INT16U  *perr);

CPU_FP32     MB_InRegRdFP               (CPU_INT16U   reg,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC03_EN == DEF_ENABLED)
CPU_INT16U   MB_HoldingRegRd            (CPU_INT16U   reg,
                                         CPU_INT16U  *perr);

CPU_FP32     MB_HoldingRegRdFP          (CPU_INT16U   reg,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC06_EN == DEF_ENABLED) || \
    (MODBUS_CFG_FC16_EN == DEF_ENABLED)
void         MB_HoldingRegWr            (CPU_INT16U   reg,
                                         CPU_INT16U   reg_val_16,
                                         CPU_INT16U  *perr);

void         MB_HoldingRegWrFP          (CPU_INT16U   reg,
                                         CPU_FP32     reg_val_fp,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC20_EN == DEF_ENABLED)
CPU_INT16U   MB_FileRd                  (CPU_INT16U   file_nbr,
                                         CPU_INT16U   record_nbr,
                                         CPU_INT16U   ix,
                                         CPU_INT08U   record_len,
                                         CPU_INT16U  *perr);
#endif

#if (MODBUS_CFG_FC21_EN == DEF_ENABLED)
void         MB_FileWr                  (CPU_INT16U   file_nbr,
                                         CPU_INT16U   record_nbr,
                                         CPU_INT16U   ix,
                                         CPU_INT08U   record_len,
                                         CPU_INT16U   value,
                                         CPU_INT16U  *perr);
#endif

/*
*********************************************************************************************************
*                                        BSP FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         MB_CommExit                (void);                   /* Exit       Modbus Communications                             */

void         MB_CommPortCfg             (MODBUS_CH   *pch,
                                         CPU_INT08U   port_nbr,
                                         CPU_INT32U   baud,
                                         CPU_INT08U   bits,
                                         CPU_INT08U   parity,
                                         CPU_INT08U   stops);

void         MB_CommRxTxISR_0_Handler   (void);
void         MB_CommRxTxISR_1_Handler   (void);
void         MB_CommRxTxISR_2_Handler   (void);
void         MB_CommRxTxISR_3_Handler   (void);
void         MB_CommRxTxISR_4_Handler   (void);
void         MB_CommRxTxISR_5_Handler   (void);
void         MB_CommRxTxISR_6_Handler   (void);
void         MB_CommRxTxISR_7_Handler   (void);
void         MB_CommRxTxISR_8_Handler   (void);
void         MB_CommRxTxISR_9_Handler   (void);

void         MB_CommRxIntEn             (MODBUS_CH   *pch);           /* Enable  Rx interrupts                                        */

void         MB_CommRxIntDis            (MODBUS_CH   *pch);           /* Disable Rx interrupts                                        */

void         MB_CommTx1                 (MODBUS_CH   *pch,
                                         CPU_INT08U   c);

void         MB_CommTxIntEn             (MODBUS_CH   *pch);           /* Enable  Tx interrupts                                        */

void         MB_CommTxIntDis            (MODBUS_CH   *pch);           /* Disable Tx interrupts                                        */


#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void         MB_RTU_TmrInit             (void);                       /* Initialize the timer used for RTU framing                    */

void         MB_RTU_TmrExit             (void);

void         MB_RTU_TmrISR_Handler      (void);
#endif

/*
*********************************************************************************************************
*                                            MODBUS SLAVE
*                                      GLOBAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
CPU_BOOLEAN  MBS_FCxx_Handler           (MODBUS_CH   *pch);

void         MBS_RxTask                 (MODBUS_CH   *pch);

#if (MODBUS_CFG_FC08_EN == DEF_ENABLED)
void         MBS_StatInit               (MODBUS_CH   *pch);
#endif
#endif

/*
*********************************************************************************************************
*                                            MODBUS MASTER
*                                      GLOBAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)

#if (MODBUS_CFG_FC01_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC01_CoilRd          (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_addr,
                                      CPU_INT16U   start_addr,
                                      CPU_INT08U  *p_coil_tbl,
                                      CPU_INT16U   nbr_coils);
#endif

#if (MODBUS_CFG_FC02_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC02_DIRd            (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT08U  *p_di_tbl,
                                      CPU_INT16U   nbr_di);
#endif

#if (MODBUS_CFG_FC03_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC03_HoldingRegRd    (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT16U  *p_reg_tbl,
                                      CPU_INT16U   nbr_regs);
#endif

#if (MODBUS_CFG_FC03_EN == DEF_ENABLED) && \
    (MODBUS_CFG_FP_EN   == DEF_ENABLED)
CPU_INT16U  MBM_FC03_HoldingRegRdFP  (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_FP32    *p_reg_tbl,
                                      CPU_INT16U   nbr_regs);
#endif

#if (MODBUS_CFG_FC04_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC04_InRegRd         (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT16U  *p_reg_tbl,
                                      CPU_INT16U   nbr_regs);
#endif

#if (MODBUS_CFG_FC05_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC05_CoilWr          (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_BOOLEAN  coil_val);
#endif

#if (MODBUS_CFG_FC06_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC06_HoldingRegWr    (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT16U   reg_val);
#endif

#if (MODBUS_CFG_FC06_EN == DEF_ENABLED) && \
    (MODBUS_CFG_FP_EN   == DEF_ENABLED)
CPU_INT16U  MBM_FC06_HoldingRegWrFP  (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_FP32     reg_val_fp);
#endif

#if (MODBUS_CFG_FC08_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC08_Diag            (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   fnct,
                                      CPU_INT16U   fnct_data,
                                      CPU_INT16U  *pval);
#endif

#if (MODBUS_CFG_FC15_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC15_CoilWr          (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT08U  *p_coil_tbl,
                                      CPU_INT16U   nbr_coils);
#endif

#if (MODBUS_CFG_FC16_EN == DEF_ENABLED)
CPU_INT16U  MBM_FC16_HoldingRegWrN   (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_INT16U  *p_reg_tbl,
                                      CPU_INT16U   nbr_regs);
#endif

#if (MODBUS_CFG_FC16_EN == DEF_ENABLED) && \
    (MODBUS_CFG_FP_EN   == DEF_ENABLED)
CPU_INT16U  MBM_FC16_HoldingRegWrNFP (MODBUS_CH   *pch,
                                      CPU_INT08U   slave_node,
                                      CPU_INT16U   slave_addr,
                                      CPU_FP32    *p_reg_tbl,
                                      CPU_INT16U   nbr_regs);
#endif

#endif
/*
*********************************************************************************************************
*                                            SYMBOL ERRORS
*********************************************************************************************************
*/

#ifndef  MODBUS_CFG_MASTER_EN
#error  "MODBUS_CFG_MASTER_EN                   not #defined                                           "
#error  "... Defines wheteher your product will support Modbus Master                                  "
#endif

#ifndef  MODBUS_CFG_SLAVE_EN
#error  "MODBUS_CFG_SLAVE_EN                   not #defined                                             "
#error  "... Defines wheteher your product will support Modbus Slave                                    "
#endif

#ifndef  MODBUS_CFG_MAX_CH
#error  "MODBUS_CFG_MAX_CH                       not #defined                                           "
#error  "... Defines the number of Modbus ports supported.  Should be 1 to N.                           "
#endif

#ifndef  MODBUS_CFG_MAX_CH
#error  "MODBUS_CFG_MAX_CH                       not #defined                                           "
#error  "... Defines the number of Modbus ports supported.  Should be 1 to N.                           "
#endif


#ifndef  MODBUS_CFG_MAX_CH
#error  "MODBUS_CFG_MAX_CH                       not #defined                                           "
#error  "... Defines the number of Modbus ports supported.  Should be 1 to N.                           "
#endif

#ifndef  MODBUS_CFG_ASCII_EN
#error  "MODBUS_CFG_ASCII_EN                     not #defined                                           "
#error  "... Defines whether your product will support Modbus ASCII.                                    "
#endif

#ifndef  MODBUS_CFG_RTU_EN
#error  "MODBUS_CFG_RTU_EN                       not #defined                                           "
#error  "... Defines whether your product will support Modbus RTU.                                      "
#endif

#ifndef  MODBUS_CFG_FP_EN
#error  "MODBUS_CFG_FP_EN                        not #defined                                           "
#error  "... Defines whether your product will support Daniels Flow Meter Floating-Point extensions.    "
#endif

#ifndef  MODBUS_CFG_FP_START_IX
#error  "MODBUS_CFG_FP_START_IX                  not #defined                                           "
#error  "... Defines the starting register number for floating-point registers.                         "
#endif

#ifndef  MODBUS_CFG_FC01_EN
#error  "MODBUS_CFG_FC01_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC02_EN
#error  "MODBUS_CFG_FC02_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC03_EN
#error  "MODBUS_CFG_FC03_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC04_EN
#error  "MODBUS_CFG_FC04_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC05_EN
#error  "MODBUS_CFG_FC05_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC06_EN
#error  "MODBUS_CFG_FC06_EN                      not #defined                                           "
#endif

#ifndef  MODBUS_CFG_FC08_EN
#error  "MODBUS_CFG_FC08_EN                      not #defined                                            "
#endif

#ifndef  MODBUS_CFG_FC15_EN
#error  "MODBUS_CFG_FC15_EN                      not #defined                                            "
#endif

#ifndef  MODBUS_CFG_FC16_EN
#error  "MODBUS_CFG_FC16_EN                      not #defined                                            "
#endif

#ifndef  MODBUS_CFG_FC20_EN
#error  "MODBUS_CFG_FC20_EN                      not #defined                                            "
#endif

#ifndef  MODBUS_CFG_FC21_EN
#error  "MODBUS_CFG_FC21_EN                      not #defined                                            "
#endif


 
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of modbus module include.                             */

