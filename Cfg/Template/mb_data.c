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
*                         uC/MODBUS TARGET SPECIFIC DATA ACCESS FUNCTIONS (Template)
*
* Filename : mb_data.c
* Version  : V2.14.00
*********************************************************************************************************
* Note(s)  :
*********************************************************************************************************
*/


#include <mb.h>

/*
*********************************************************************************************************
*                                     GET THE VALUE OF A SINGLE COIL
*
* Description: This function returns the value of a single coil.
*              It is called by 'MBS_FC01_CoilRd()'.
*              You must 'map' the 'coil' to the actual application's coil.
*
* Arguments  : coil     is the coil number that is being requested.
*
*              perr     is a pointer to an error code variable.  You must either return:
*
*                       MODBUS_ERR_NONE     the specified coil is valid and you are returning its value.
*                       MODBUS_ERR_RANGE    the specified coil is an invalid coil number in your
*                                           application (i.e. product).  YOUR product defines what the
*                                           valid range of values is for the 'coil' argument.
*
* Note(s)    : 1) You can perform the mapping of coil number to application coils directly in this
*                 function or via a table lookup.  A table lookup would make sense if you had a lot of
*                 coils in your product.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC01_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_CoilRd (CPU_INT16U   coil,
                        CPU_INT16U  *perr)
{
    CPU_BOOLEAN  coil_val;

    switch (coil) {
        case 0:
             coil_val = DEF_TRUE;
             break;

        case 1:
             coil_val = DEF_FALSE;
             break;

        case 2:
             coil_val = DEF_TRUE;
             break;

        default:
        case 3:
             coil_val = DEF_FALSE;
             break;
    }

    *perr = MODBUS_ERR_NONE;
    return (coil_val);
}
#endif

/*
*********************************************************************************************************
*                                     SET THE VALUE OF A SINGLE COIL
*
* Description: This function changes the value of a single coil.
*              It is called by 'MBS_FC05_CoilWr()' and 'MBS_FC15_CoilWrMultiple()'.
*              You must 'map' the 'coil' to the actual application's coil.
*
* Arguments  : coil      is the coil number that needs to be changed.
*
*              coil_val  is the desired value of the coil.  This value can be either DEF_TRUE or DEF_FALSE with
*                        DEF_TRUE indicating an energized coil.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified coil is valid and your code changed the value
*                                            of the coil.
*                        MODBUS_ERR_RANGE    the specified coil is an invalid coil number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'coil' argument.
*                        MODBUS_ERR_WR       if the device is not able to write or accept the value
*
* Note(s)    : 1) You can perform the mapping of coil number to application coils directly in this
*                 function or via a table lookup.  A table lookup would make sense if you had a lot of
*                 coils in your product.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC05_EN == DEF_ENABLED) || \
    (MODBUS_CFG_FC15_EN == DEF_ENABLED)
void  MB_CoilWr (CPU_INT16U    coil,
                 CPU_BOOLEAN   coil_val,
                 CPU_INT16U   *perr)
{
    (void)coil;
    (void)coil_val;
    *perr = MODBUS_ERR_NONE;
}
#endif

/*
*********************************************************************************************************
*                                GET THE VALUE OF A SINGLE DISCRETE INPUT
*
* Description: This function reads the value of a single DI (DI means Discrete Input).
*              It is called by 'MBS_FC02_DIRd()'.
*              You must 'map' the 'di'  to the actual application's DI.
*
* Arguments  : di        is the Discrete Input number that needs to be read.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified DI is valid and your code is returning its
*                                            current value.
*                        MODBUS_ERR_RANGE    the specified DI is an invalid Discrete Input number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'di' argument.
*
* Note(s)    : 1) You can perform the mapping of DI number to the application DIs directly in this function
*                 or via a table lookup.  A table lookup would make sense if you had a lot of Discrete
*                 Inputs in your product.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC02_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_DIRd (CPU_INT16U   di,
                      CPU_INT16U  *perr)
{
    (void)di;
    *perr = MODBUS_ERR_NONE;
    return (DEF_FALSE);
}
#endif

/*
*********************************************************************************************************
*                               GET THE VALUE OF A SINGLE INPUT REGISTER
*
* Description: This function reads the value of a single Input Register.
*              It is called by 'MBS_FC04_InRegRd()' when the argument 'reg' is BELOW the value set by
*              the configuration constant MODBUS_CFG_FP_START_IX (see MB_CFG.H).
*              You must 'map' the Input Register to the actual application's corresponding integer register.
*
* Arguments  : reg       is the Input Register number that needs to be read.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified input register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified input register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*
* Note(s)    : 1) You can perform the mapping of input register number to the application's input registers
*                 directly in this function or via a table lookup.  A table lookup would make sense if you
*                 had a lot of Input Registers in your product.
*              2) If your product doesn't have input registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return 0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC04_EN == DEF_ENABLED)
CPU_INT16U  MB_InRegRd (CPU_INT16U   reg,
                        CPU_INT16U  *perr)
{
    CPU_INT16U  val;
    CPU_SR      cpu_sr;


    switch (reg) {
        case 10:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)OSCPUUsage;
             CPU_CRITICAL_EXIT();
             break;

        case 11:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)OSCtxSwCtr;
             CPU_CRITICAL_EXIT();
             break;

        case 12:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(OSTime >> 16);
             CPU_CRITICAL_EXIT();
             break;

        case 13:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(OSTime & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 14:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)MB_ChSize;
             CPU_CRITICAL_EXIT();
             break;

        case 15:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(MB_TotalRAMSize & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        default:
             val = 0;
             break;
    }
    *perr = MODBUS_ERR_NONE;
    return (val);
}
#endif

/*
*********************************************************************************************************
*                     GET THE VALUE OF A SINGLE 'FLOATING-POINT' INPUT REGISTER
*
* Description: This function reads the value of a single Input Register.
*              It is called by 'MBS_FC04_InRegRd()' when the argument 'reg' is ABOVE or equal to the
*              value set the configuration constant MODBUS_CFG_FP_START_IX (see MB_CFG.H).
*              You must 'map' the Input Register to the actual application's corresponding floating-point
*              register.
*
* Arguments  : reg       is the Input Register number that needs to be read.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified input register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified input register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*
* Note(s)    : 1) You can perform the mapping of input register number to the application's input registers
*                 directly in this function or via a table lookup.  A table lookup would make sense if you
*                 had a lot of Input Registers in your product.
*              2) If your product doesn't have input registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return (CPU_FP32)0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FP_EN   == DEF_ENABLED)
#if (MODBUS_CFG_FC04_EN == DEF_ENABLED)
CPU_FP32  MB_InRegRdFP (CPU_INT16U   reg,
                        CPU_INT16U  *perr)
{
    (void)reg;
    *perr = MODBUS_ERR_NONE;
    return ((CPU_FP32)0);
}
#endif
#endif

/*
*********************************************************************************************************
*                             GET THE VALUE OF A SINGLE HOLDING REGISTER
*
* Description: This function reads the value of a single Holding Register.
*              It is called by 'MBS_FC03_HoldingRegRd()' when the argument 'reg' is BELOW the value set
*              by the configuration constant MODBUS_CFG_FP_START_IX (see MB_CFG.H).
*              You must 'map' the Holding Register to the actual application's corresponding integer register.
*
* Arguments  : reg       is the Holding Register number that needs to be read.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified holding register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified holding register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*
* Note(s)    : 1) You can perform the mapping of holding register number to the application's holding
*                 registers directly in this function or via a table lookup.  A table lookup would make
*                 sense if you had a lot of Holding Registers in your product.
*              2) If your product doesn't have holding registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return 0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC03_EN == DEF_ENABLED)
CPU_INT16U  MB_HoldingRegRd (CPU_INT16U   reg,
                             CPU_INT16U  *perr)
{
    CPU_INT16U  val;
    CPU_SR      cpu_sr;


    switch (reg) {
        case 0:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)OSCPUUsage;
             CPU_CRITICAL_EXIT();
             break;

        case 1:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)OSCtxSwCtr;
             CPU_CRITICAL_EXIT();
             break;

        case 2:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(OSTime >> 16);
             CPU_CRITICAL_EXIT();
             break;

        case 3:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(OSTime & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 4:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)MB_ChSize;
             CPU_CRITICAL_EXIT();
             break;

        case 5:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)(MB_TotalRAMSize & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 6:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)((MB_ChTbl[0].RxCtr / 1000) & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 7:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)((MB_ChTbl[0].RxCtr % 1000) & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 8:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)((MB_ChTbl[0].TxCtr / 1000) & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        case 9:
             CPU_CRITICAL_ENTER();
             val = (CPU_INT16U)((MB_ChTbl[0].TxCtr % 1000) & 0x0000FFFF);
             CPU_CRITICAL_EXIT();
             break;

        default:
             val = 0;
             break;
    }
    *perr = MODBUS_ERR_NONE;
    return (val);
}
#endif

/*
*********************************************************************************************************
*                     GET THE VALUE OF A SINGLE 'FLOATING-POINT' HOLDING REGISTER
*
* Description: This function reads the value of a single Floating-Point Holding Register.
*              It is called by 'MBS_FC03_HoldingRegRd()' when the argument 'reg' is ABOVE or equal to the
*              value set by the configuration constant MODBUS_CFG_FP_START_IX (see MB_CFG.H).
*              You must 'map' the Holding Register to the actual application's corresponding floating-point
*              register.
*
* Arguments  : reg       is the Holding Register number that needs to be read.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified holding register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified holding register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*
* Note(s)    : 1) You can perform the mapping of holding register number to the application's holding
*                 registers directly in this function or via a table lookup.  A table lookup would make
*                 sense if you had a lot of Holding Registers in your product.
*              2) If your product doesn't have holding registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return 0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FP_EN   == DEF_ENABLED)
#if (MODBUS_CFG_FC03_EN == DEF_ENABLED)
CPU_FP32  MB_HoldingRegRdFP (CPU_INT16U   reg,
                             CPU_INT16U  *perr)
{
    (void)reg;
    *perr = MODBUS_ERR_NONE;
    return ((CPU_FP32)0);
}
#endif
#endif

/*
*********************************************************************************************************
*                            SET THE VALUE OF A SINGLE HOLDING REGISTER
*
* Description: This function is called to change the value of a single Integer Holding Register.
*              It is called by 'MBS_FC06_HoldingRegWr()' and 'MBS_FC16_HoldingRegWrMultiple()' when the argument
*              'reg' is BELOW to the value set by the configuration constant MODBUS_CFG_FP_START_IX (see MB_CFG.H).
*              You must 'map' the Holding Register to the actual application's corresponding integer register.
*
* Arguments  : reg       is the Holding Register number that needs to be read.
*
*              reg_val   is the desired value of the holding register.
*                        The value is specified as an unsigned integer even though it could actually be
*                        represented by a signed integer.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified holding register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified holding register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*                        MODBUS_ERR_WR       if the device is not able to write or accept the value
*
* Note(s)    : 1) You can perform the mapping of holding register number to the application's holding
*                 registers directly in this function or via a table lookup.  A table lookup would make
*                 sense if you had a lot of Holding Registers in your product.
*              2) If your product doesn't have holding registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return 0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC06_EN == DEF_ENABLED) || \
    (MODBUS_CFG_FC16_EN == DEF_ENABLED)
void  MB_HoldingRegWr (CPU_INT16U   reg,
                       CPU_INT16U   reg_val_16,
                       CPU_INT16U  *perr)
{
    /* Access to your variable here! */
    (void)reg;
    (void)reg_val_16;
    *perr = MODBUS_ERR_NONE;
}
#endif

/*
*********************************************************************************************************
*                     SET THE VALUE OF A SINGLE 'FLOATING-POINT' HOLDING REGISTER
*
* Description: This function is called to change the value of a single Floating-Point Holding Register.
*              It is called by 'MBS_FC06_HoldingRegWr()' and 'MBS_FC16_HoldingRegWrMultiple()' when the argument
*              'reg' is ABOVE or equal to the value set by the configuration constant MODBUS_CFG_FP_START_IX
*              (see MB_CFG.H).
*              You must 'map' the Holding Register to the actual application's corresponding floating-point
*              register.
*
* Arguments  : reg       is the Holding Register number that needs to be read.
*
*              reg_val   is the desired value of the holding register.
*                        The value is specified as an unsigned integer even though it could actually be
*                        represented by a signed integer.
*
*              perr      is a pointer to an error code variable.  You must either return:
*
*                        MODBUS_ERR_NONE     the specified holding register is valid and your code is
*                                            returning its current value.
*                        MODBUS_ERR_RANGE    the specified holding register is an invalid number in your
*                                            application (i.e. product).  YOUR product defines what the
*                                            valid range of values is for the 'reg' argument.
*                        MODBUS_ERR_WR       if the device is not able to write or accept the value
*
* Note(s)    : 1) You can perform the mapping of holding register number to the application's holding
*                 registers directly in this function or via a table lookup.  A table lookup would make
*                 sense if you had a lot of Holding Registers in your product.
*              2) If your product doesn't have holding registers, you could simply set '*err' to
*                 MODBUS_ERR_NONE and return 0.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FP_EN    == DEF_ENABLED)
#if (MODBUS_CFG_FC06_EN == DEF_ENABLED) || \
    (MODBUS_CFG_FC16_EN == DEF_ENABLED)
void  MB_HoldingRegWrFP (CPU_INT16U   reg,
                         CPU_FP32     reg_val_fp,
                         CPU_INT16U  *perr)
{
    (void)reg;
    (void)reg_val_fp;
    *perr = MODBUS_ERR_RANGE;
}
#endif
#endif

/*
*********************************************************************************************************
*                              GET A SINGLE ENTRY FROM A RECORD IN A FILE
*
* Description: This function is called to read a single integer from a file.
*              As mentionned in the Modbus specifications, a file is an organization of records.
*              Each file can contain up to 10,000 records (addressed from 0 to 9999).
*              You must 'map' the File/Record/Ix to the actual application's corresponding data.
*
* Arguments  : file_nbr    is the number of the desired file.
*
*              record_nbr  is the desired record within the file
*
*              ix          is the desired entry in the specified record.
*
*              record_len  is the desired length of the record.  Note that this parameter is passed to
*                          this function to provide the 'requested' requested length from the MODBUS command.
*
*              perr        is a pointer to an error code variable.  You must either return:
*
*                          MODBUS_ERR_NONE     the specified file/record/entry is valid and your code is
*                                              returning its current value.
*                          MODBUS_ERR_FILE     if the specified 'file_nbr' is not a valid file number in
*                                              your product.
*                          MODBUS_ERR_RECORD   if the specified 'record_nbr' is not a valid record in the
*                                              specified file.
*                          MODBUS_ERR_IX       if the specified 'ix' is not a valid index into the specified
*                                              record.
*
* Note(s)    : 1) You can perform the mapping of file/record/ix to the application's data directly in
*                 this function or via a table lookup.  A table lookup would make sense if you had a lot
*                 data in your files.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC20_EN == DEF_ENABLED)
CPU_INT16U  MB_FileRd (CPU_INT16U   file_nbr,
                       CPU_INT16U   record_nbr,
                       CPU_INT16U   ix,
                       CPU_INT08U   record_len,
                       CPU_INT16U  *perr)
{
    (void)file_nbr;
    (void)record_nbr;
    (void)ix;
    (void)record_len;
    *perr  = MODBUS_ERR_NONE;
    return (0);
}
#endif

/*
*********************************************************************************************************
*                               SET A SINGLE ENTRY OF A RECORD IN A FILE
*
* Description: This function is called to change a single integer value in a file.
*              As mentionned in the Modbus specifications, a file is an organization of records.
*              Each file can contain up to 10,000 records (addressed from 0 to 9999).
*              You must 'map' the File/Record/Ix to the actual application's corresponding data.
*
* Arguments  : file_nbr    is the number of the desired file.
*
*              record_nbr  is the desired record within the file
*
*              ix          is the desired entry in the specified record.
*
*              record_len  is the desired length of the record.  Note that this parameter is passed to
*                          this function to provide the 'requested' requested length from the MODBUS command.
*
*              val         is the new value to place in the file.
*
*              perr        is a pointer to an error code variable.  You must either return:
*
*                          MODBUS_ERR_NONE     the specified file/record/entry is valid and your code is
*                                              returning its current value.
*                          MODBUS_ERR_FILE     if the specified 'file_nbr' is not a valid file number in
*                                              your product.
*                          MODBUS_ERR_RECORD   if the specified 'record_nbr' is not a valid record in the
*                                              specified file.
*                          MODBUS_ERR_IX       if the specified 'ix' is not a valid index into the specified
*                                              record.
*
* Note(s)    : 1) You can perform the mapping of file/record/ix to the application's data directly in
*                 this function or via a table lookup.  A table lookup would make sense if you had a lot
*                 data in your files.
*********************************************************************************************************
*/

#if (MODBUS_CFG_FC21_EN == DEF_ENABLED)
void  MB_FileWr (CPU_INT16U   file_nbr,
                 CPU_INT16U   record_nbr,
                 CPU_INT16U   ix,
                 CPU_INT08U   record_len,
                 CPU_INT16U   val,
                 CPU_INT16U  *perr)
{
    (void)file_nbr;
    (void)record_nbr;
    (void)ix;
    (void)record_len;
    (void)val;
    *perr = MODBUS_ERR_NONE;
}
#endif
