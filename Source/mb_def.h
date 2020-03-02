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
*                                  uC/MODBUS DEFINITIONS HEADER FILE
*
* Filename : mb_def.h
* Version  : V2.14.00
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        GLOBAL MODBUS CONSTANTS
*********************************************************************************************************
*/

#define  MODBUS_MODE_ASCII                          1
#define  MODBUS_MODE_RTU                            0


#define  MODBUS_WR_EN                               1
#define  MODBUS_WR_DIS                              0


#define  MODBUS_PARITY_NONE                         0
#define  MODBUS_PARITY_ODD                          1
#define  MODBUS_PARITY_EVEN                         2


#define  MODBUS_COIL_OFF                            0
#define  MODBUS_COIL_ON                             1

#define  MODBUS_SLAVE                               0
#define  MODBUS_MASTER                              1

#define  MODBUS_MASTER_STATE_RX                     0
#define  MODBUS_MASTER_STATE_TX                     1
#define  MODBUS_MASTER_STATE_WAITING                2

#define  MODBUS_FALSE                               0
#define  MODBUS_TRUE                                1

 
/*
*********************************************************************************************************
*                                              CONSTANTS
*********************************************************************************************************
*/

#define  MODBUS_FC01_COIL_RD                        1       /* COIL Status.                            */
#define  MODBUS_FC02_DI_RD                          2       /* Read Discrete Input                     */
#define  MODBUS_FC03_HOLDING_REG_RD                 3       /* Holding registers.                      */
#define  MODBUS_FC04_IN_REG_RD                      4       /* Read Only registers.                    */
#define  MODBUS_FC05_COIL_WR                        5       /* Set a single COIL value.                */
#define  MODBUS_FC06_HOLDING_REG_WR                 6       /* Holding registers.                      */
#define  MODBUS_FC08_LOOPBACK                       8
#define  MODBUS_FC15_COIL_WR_MULTIPLE              15       /* Set multiple COIL values.               */
#define  MODBUS_FC16_HOLDING_REG_WR_MULTIPLE       16       /* Holding registers                       */
#define  MODBUS_FC20_FILE_RD                       20       /* Read contents of a File/Record          */
#define  MODBUS_FC21_FILE_WR                       21       /* Write data to a File/Record             */

#define  MODBUS_FC08_LOOPBACK_QUERY                 0       /* Loopback sub-function codes             */
#define  MODBUS_FC08_LOOPBACK_CLR_CTR              10
#define  MODBUS_FC08_LOOPBACK_BUS_MSG_CTR          11
#define  MODBUS_FC08_LOOPBACK_BUS_CRC_CTR          12
#define  MODBUS_FC08_LOOPBACK_BUS_EXCEPT_CTR       13
#define  MODBUS_FC08_LOOPBACK_SLAVE_MSG_CTR        14
#define  MODBUS_FC08_LOOPBACK_SLAVE_NO_RESP_CTR    15


#define  MODBUS_COIL_OFF_CODE                  0x0000
#define  MODBUS_COIL_ON_CODE                   0xFF00
/*
*********************************************************************************************************
*                                              ERROR CODES
*********************************************************************************************************
*/

#define  MODBUS_ERR_NONE                            0

#define  MODBUS_ERR_ILLEGAL_FC                      1
#define  MODBUS_ERR_ILLEGAL_DATA_ADDR               2
#define  MODBUS_ERR_ILLEGAL_DATA_QTY                3
#define  MODBUS_ERR_ILLEGAL_DATA_VAL                4

#define  MODBUS_ERR_FC01_01                       101
#define  MODBUS_ERR_FC01_02                       102
#define  MODBUS_ERR_FC01_03                       103

#define  MODBUS_ERR_FC02_01                       201
#define  MODBUS_ERR_FC02_02                       202

#define  MODBUS_ERR_FC03_01                       301
#define  MODBUS_ERR_FC03_02                       302
#define  MODBUS_ERR_FC03_03                       303
#define  MODBUS_ERR_FC03_04                       304

#define  MODBUS_ERR_FC04_01                       401
#define  MODBUS_ERR_FC04_02                       402
#define  MODBUS_ERR_FC04_03                       403
#define  MODBUS_ERR_FC04_04                       404

#define  MODBUS_ERR_FC05_01                       501
#define  MODBUS_ERR_FC05_02                       502

#define  MODBUS_ERR_FC06_01                       601

#define  MODBUS_ERR_FC08_01                       801

#define  MODBUS_ERR_FC15_01                      1501
#define  MODBUS_ERR_FC15_02                      1502
#define  MODBUS_ERR_FC15_03                      1503

#define  MODBUS_ERR_FC16_01                      1601
#define  MODBUS_ERR_FC16_02                      1602
#define  MODBUS_ERR_FC16_03                      1603
#define  MODBUS_ERR_FC16_04                      1604
#define  MODBUS_ERR_FC16_05                      1605

#define  MODBUS_ERR_FC20_01                      2001
#define  MODBUS_ERR_FC20_02                      2002
#define  MODBUS_ERR_FC20_03                      2003
#define  MODBUS_ERR_FC20_04                      2004
#define  MODBUS_ERR_FC20_05                      2005

#define  MODBUS_ERR_FC21_01                      2101
#define  MODBUS_ERR_FC21_02                      2102
#define  MODBUS_ERR_FC21_03                      2103
#define  MODBUS_ERR_FC21_04                      2104
#define  MODBUS_ERR_FC21_05                      2105

#define  MODBUS_ERR_TIMED_OUT                    3000
#define  MODBUS_ERR_NOT_MASTER                   3001
#define  MODBUS_ERR_INVALID                      3002
#define  MODBUS_ERR_NULLPTR                      3003

#define  MODBUS_ERR_RANGE                        4000
#define  MODBUS_ERR_FILE                         4001
#define  MODBUS_ERR_RECORD                       4002
#define  MODBUS_ERR_IX                           4003
#define  MODBUS_ERR_VALUE                        4004

#define  MODBUS_ERR_COIL_ADDR                    5000
#define  MODBUS_ERR_COIL_WR                      5001
#define  MODBUS_ERR_SLAVE_ADDR                   5002
#define  MODBUS_ERR_FC                           5003
#define  MODBUS_ERR_BYTE_COUNT                   5004
#define  MODBUS_ERR_COIL_QTY                     5005
#define  MODBUS_ERR_REG_ADDR                     5006
#define  MODBUS_ERR_NBR_REG                      5007
#define  MODBUS_ERR_SUB_FNCT                     5008
#define  MODBUS_ERR_DIAG                         5009
#define  MODBUS_ERR_WR                           5010

#define  MODBUS_ERR_RX                           6000

/*
*********************************************************************************************************
*                                        MODBUS ASCII CONSTANTS
*********************************************************************************************************
*/

#if     (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
#define  MODBUS_ASCII_START_FRAME_CHAR            ':'       /* Start of frame delimiter                */
#define  MODBUS_ASCII_END_FRAME_CHAR1            0x0D       /* ASCII character: Carriage return        */
#define  MODBUS_ASCII_END_FRAME_CHAR2            0x0A       /* ASCII character: Line Feed              */
#define  MODBUS_ASCII_MIN_MSG_SIZE                 11
#endif

/*
*********************************************************************************************************
*                                         MODBUS RTU CONSTANTS
*********************************************************************************************************
*/

#if     (MODBUS_CFG_RTU_EN == DEF_ENABLED)
#define  MODBUS_RTU_MIN_MSG_SIZE                    4
#endif

#define  MODBUS_CRC16_POLY                     0xA001       /* CRC-16 Generation Polynomial value.     */

