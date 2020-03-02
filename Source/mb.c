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
*                                        uC/MODBUS Source Code
*
* Filename : mb.c
* Version  : V2.14.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#define    MB_MODULE
#include  <mb.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


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

                                                                /* RAM Storage Requirements.                            */
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_INT32U  const  MB_TotalRAMSize = sizeof(MB_RTU_Freq)
                                   + sizeof(MB_RTU_TmrCtr)
                                   + sizeof(MB_ChTbl);
#else
CPU_INT32U  const  MB_TotalRAMSize = sizeof(MB_ChTbl);
#endif

CPU_INT16U  const  MB_ChSize       = sizeof(MODBUS_CH);


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MB_Init()
*
* Description : Handle either Modbus ASCII or Modbus RTU received packets.
*
* Argument(s) : freq       Specifies the Modbus RTU timer frequency (in Hz)
*
* Return(s)   : none.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_Init (CPU_INT32U freq)
{
    CPU_INT08U   ch;
    MODBUS_CH   *pch;

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
    MB_RTU_Freq = freq;                                         /* Save the RTU frequency                             */
#endif

    pch         = &MB_ChTbl[0];                                 /* Save Modbus channel number in data structure       */
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {                /* Initialize default values                          */
        pch->Ch            = ch;
        pch->NodeAddr      = 1;
        pch->MasterSlave   = MODBUS_SLAVE;                      /* Channel defaults to MODBUS_SLAVE mode              */
        pch->Mode          = MODBUS_MODE_ASCII;
        pch->RxBufByteCtr  = 0;
        pch->RxBufPtr      = &pch->RxBuf[0];
        pch->WrEn          = MODBUS_WR_EN;
        pch->WrCtr         = 0;
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
        pch->RTU_TimeoutEn = DEF_TRUE;
#endif

#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)  && \
    (MODBUS_CFG_FC08_EN  == DEF_ENABLED)
        MBS_StatInit(pch);
#endif
        pch++;
    }

    MB_ChCtr = 0;

    MB_OS_Init();                                               /* Initialize OS interface functions                  */


#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)                         /* MODBUS 'RTU' Initialization                         */
    MB_RTU_TmrInit();
#else
    (void)&freq;
#endif
}

/*
*********************************************************************************************************
*                                               MB_Exit()
*
* Description : This function is called to terminate all Modbus communications
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_Exit (void)
{
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
    MB_RTU_TmrExit();                                           /* Stop the RTU timer interrupts                      */
#endif

    MB_CommExit();                                              /* Disable all communications                         */

    MB_OS_Exit();                                               /* Stop RTOS services                                 */
}


/*
*********************************************************************************************************
*                                              MB_CfgCh()
*
* Description : This function must be called after calling MB_Init() to initialize each of the Modbus
*               channels in your system.
*
* Argument(s) : node_addr     is the Modbus node address that the channel is assigned to.
*
*               master_slave  specifies whether the channel is a MODBUS_MASTER or a MODBUS_SLAVE
*
*               rx_timeout    amount of time Master will wait for a response from the slave.
*
*               modbus_mode   specifies the type of modbus channel.  The choices are:
*                             MODBUS_MODE_ASCII
*                             MODBUS_MODE_RTU
*
*               port_nbr      is the UART port number associated with the channel
*
*               baud          is the desired baud rate
*
*               parity        is the UART's parity setting:
*                             MODBUS_PARITY_NONE
*                             MODBUS_PARITY_ODD
*                             MODBUS_PARITY_EVEN
*
*               bits          UART's number of bits (7 or 8)
*
*               stops         Number of stops bits (1 or 2)
*
*               wr_en         This argument determines whether a Modbus WRITE request will be accepted.
*                             The choices are:
*                             MODBUS_WR_EN
*                             MODBUS_WR_DIS
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

MODBUS_CH  *MB_CfgCh (CPU_INT08U  node_addr,
                      CPU_INT08U  master_slave,
                      CPU_INT32U  rx_timeout,
                      CPU_INT08U  modbus_mode,
                      CPU_INT08U  port_nbr,
                      CPU_INT32U  baud,
                      CPU_INT08U  bits,
                      CPU_INT08U  parity,
                      CPU_INT08U  stops,
                      CPU_INT08U  wr_en)
{
    MODBUS_CH   *pch;
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
    CPU_INT16U   cnts;
#endif

    if (MB_ChCtr < MODBUS_CFG_MAX_CH) {
        pch = &MB_ChTbl[MB_ChCtr];
        MB_MasterTimeoutSet(pch, rx_timeout);
        MB_NodeAddrSet(pch, node_addr);
        MB_ModeSet(pch, master_slave, modbus_mode);
        MB_WrEnSet(pch, wr_en);
        MB_ChToPortMap(pch, port_nbr);
        MB_CommPortCfg(pch, port_nbr, baud, bits, parity, stops);
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
        if (pch->MasterSlave == MODBUS_MASTER) {
            pch->RTU_TimeoutEn = DEF_FALSE;
        }

        cnts = ((CPU_INT32U)MB_RTU_Freq * 5L * 10L) / baud;     /* Freq * 5 char * 10 bits/char * 1/BaudRate          */
        if (cnts <= 1) {
            cnts = 2;
        }
        pch->RTU_TimeoutCnts = cnts;
        pch->RTU_TimeoutCtr  = cnts;
#endif
        MB_ChCtr++;
        return (pch);
    } else {
        return ((MODBUS_CH *)0);
    }
}

/*
*********************************************************************************************************
*                                         MB_MasterTimeoutSet()
*
* Description : This function is called to change the operating mode of a Modbus channel.
*
* Argument(s) : pch          is a pointer to the Modbus channel to change
*
*               modbus_mode  specifies the type of modbus channel.  The choices are:
*                            MODBUS_MODE_ASCII
*                            MODBUS_MODE_RTU
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_MasterTimeoutSet (MODBUS_CH  *pch,
                           CPU_INT32U  timeout)
{
    if (pch != (MODBUS_CH *)0) {
        pch->RxTimeout = timeout;
    }
}

/*
*********************************************************************************************************
*                                             MB_ModeSet()
*
* Description : This function is called to change the operating mode of a Modbus channel.
*
* Argument(s) : pch          is a pointer to the Modbus channel to change
*
*               modbus_mode  specifies the type of modbus channel.  The choices are:
*                            MODBUS_MODE_ASCII
*                            MODBUS_MODE_RTU
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_ModeSet (MODBUS_CH  *pch,
                  CPU_INT08U  master_slave,
                  CPU_INT08U  mode)
{
    if (pch != (MODBUS_CH *)0) {

        switch (master_slave) {
            case MODBUS_MASTER:
                 pch->MasterSlave = MODBUS_MASTER;
                 break;

            case MODBUS_SLAVE:
            default:
                 pch->MasterSlave = MODBUS_SLAVE;
                 break;
        }

        switch (mode) {
#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
            case MODBUS_MODE_ASCII:
                 pch->Mode = MODBUS_MODE_ASCII;
                 break;
#endif

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
            case MODBUS_MODE_RTU:
                 pch->Mode = MODBUS_MODE_RTU;
                 break;
#endif

            default:
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
                 pch->Mode = MODBUS_MODE_RTU;
#else
                 pch->Mode = MODBUS_MODE_ASCII;
#endif
                 break;
        }
    }
}

/*
*********************************************************************************************************
*                                           MB_NodeAddrSet()
*
* Description : This function is called to change the Modbus node address that the channel will respond to.
*
* Argument(s) : pch          is a pointer to the Modbus channel to change
*
*               node_addr    is the Modbus node address that the channel is assigned to.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_NodeAddrSet (MODBUS_CH  *pch,
                      CPU_INT08U  node_addr)
{
    if (pch != (MODBUS_CH *)0) {
        pch->NodeAddr = node_addr;
    }
}

/*
*********************************************************************************************************
*                                             MB_WrEnSet()
*
* Description : This function is called to enable or disable write accesses to the data.
*
* Argument(s) : ch           is the Modbus channel to change
*
*               wr_en        This argument determines whether a Modbus WRITE request will be accepted.
*                            The choices are:
*                            MODBUS_WR_EN
*                            MODBUS_WR_DIS
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_WrEnSet (MODBUS_CH  *pch,
                  CPU_INT08U  wr_en)
{
    if (pch != (MODBUS_CH *)0) {
        pch->WrEn = wr_en;
    }
}


/*
*********************************************************************************************************
*                                           MB_ChToPortMap()
*
* Description : This function is called to change the physical port number of the Modbus channel.
*
* Argument(s) : pch          is a pointer to the Modbus channel to change
*
*               port_nbr     This argument determines the physical port number of the Modbus channel
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_ChToPortMap (MODBUS_CH  *pch,
                      CPU_INT08U  port_nbr)
{
    if (pch != (MODBUS_CH *)0) {
        pch->PortNbr = port_nbr;
    }
}

 
/*
*********************************************************************************************************
*                                              MB_RxByte()
*
* Description : A byte has been received from a serial port.  We just store it in the buffer for processing
*               when a complete packet has been received.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
*               rx_byte     Is the byte received.
*
* Return(s)   : none.
*
* Caller(s)   : MB_CommRxTxISR_Handler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_RxByte (MODBUS_CH  *pch,
                 CPU_INT08U  rx_byte)
{
    switch (pch->Mode) {
#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
        case MODBUS_MODE_ASCII:
             MB_ASCII_RxByte(pch, rx_byte & 0x7F);
             break;
#endif

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
        case MODBUS_MODE_RTU:
             MB_RTU_RxByte(pch, rx_byte);
             break;
#endif

        default:
             break;
    }
}

 
/*
*********************************************************************************************************
*                                              MB_RxTask()
*
* Description : This function is called when a packet needs to be processed.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : none.
*
* Caller(s)   : MB_OS_RxTask().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_RxTask (MODBUS_CH *pch)
{
#if (MODBUS_CFG_SLAVE_EN == DEF_ENABLED)
    if (pch != (MODBUS_CH *)0) {
        if (pch->MasterSlave == MODBUS_SLAVE) {
            MBS_RxTask(pch);
        }
    }
#endif
}
/*
*********************************************************************************************************
*                                                MB_Tx()
*
* Description : This function is called to start transmitting a packet to a modbus channel.
*
* Argument(s) : pch      Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : none.
*
* Caller(s)   : MB_ASCII_Tx(),
*               MB_RTU_Tx().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_Tx (MODBUS_CH  *pch)
{
    pch->TxBufPtr = &pch->TxBuf[0];
    MB_TxByte(pch);
    MB_CommRxIntDis(pch);
    MB_CommTxIntEn(pch);
}

 
/*
*********************************************************************************************************
*                                              MB_TxByte()
*
* Description : This function is called to obtain the next byte to send from the transmit buffer.  When
*               all bytes in the reply have been sent, transmit interrupts are disabled and the receiver
*               is enabled to accept the next Modbus request.
*
* Argument(s) : pch      Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : none.
*
* Caller(s)   : MB_CommRxTxISR_Handler(),
*               MB_Tx().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  MB_TxByte (MODBUS_CH  *pch)
{
    CPU_INT08U  c;


    if (pch->TxBufByteCtr > 0) {
        pch->TxBufByteCtr--;
        pch->TxCtr++;
        c = *pch->TxBufPtr++;
        MB_CommTx1(pch,                                         /* Write one byte to the serial port                  */
                   c);
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
        if (pch->MasterSlave == MODBUS_MASTER) {
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
            pch->RTU_TimeoutEn = MODBUS_FALSE;                  /* Disable RTU timeout timer until we start receiving */
#endif
            pch->RxBufByteCtr  = 0;                             /* Flush Rx buffer                                    */
        }
#endif
    } else {                                                    /* If there is nothing to do end transmission         */
        pch->TxBufPtr = &pch->TxBuf[0];                         /* Reset at beginning of buffer                       */
        MB_CommTxIntDis(pch);                                   /* No more data to send, disable Tx interrupts        */
        MB_CommRxIntEn(pch);                                    /* Re-enable the receiver for the next packet         */
    }
}

 
/*
*********************************************************************************************************
*                                           MB_ASCII_RxByte()
*
* Description : A byte has been received from a serial port.  We just store it in the buffer for processing
*               when a complete packet has been received.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
*               rx_byte     Is the byte received.
*
* Return(s)   : none.
*
* Caller(s)   : MB_RxByte().
*
* Return(s)   : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
void  MB_ASCII_RxByte (MODBUS_CH  *pch,
                       CPU_INT08U  rx_byte)
{
    CPU_INT08U   node_addr;
    CPU_INT08U  *phex;


    pch->RxCtr++;                                               /* Increment the number of bytes received             */
    if (rx_byte == ':') {                                       /* Is it the start of frame character?                */
        pch->RxBufPtr     = &pch->RxBuf[0];                     /* Yes, Restart a new frame                           */
        pch->RxBufByteCtr = 0;
    }
    if (pch->RxBufByteCtr < MODBUS_CFG_BUF_SIZE) {              /* No, add received byte to buffer                    */
        *pch->RxBufPtr++  = rx_byte;
        pch->RxBufByteCtr++;                                    /* Increment byte counter to see if we have Rx ...    */
                                                                /* ... activity                                       */
    }
    if (rx_byte == MODBUS_ASCII_END_FRAME_CHAR2) {              /* See if we received a complete ASCII frame          */
        phex      = &pch->RxBuf[1];
        node_addr = MB_ASCII_HexToBin(phex);
        if ((node_addr == pch->NodeAddr) ||                     /* Is the address for us?                             */
            (node_addr == 0)) {                                 /* ... or a 'broadcast'?                              */
            MB_OS_RxSignal(pch);                                /* Yes, Let task handle reply                         */
        } else {
            pch->RxBufPtr     = &pch->RxBuf[0];                 /* No,  Wipe out anything, we have to re-synchronize. */
            pch->RxBufByteCtr = 0;
        }
    }
}
#endif

 
/*
*********************************************************************************************************
*                                             MB_ASCII_Rx()
*
* Description : Parses and converts an ASCII style message into a Modbus frame.  A check is performed
*               to verify that the Modbus packet is valid.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : DEF_TRUE        If all checks pass.
*               DEF_FALSE       If any checks fail.
*
* Caller(s)   : MBM_RxReply().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_ASCII_Rx (MODBUS_CH  *pch)
{
    CPU_INT08U  *pmsg;
    CPU_INT08U  *prx_data;
    CPU_INT16U   rx_size;


    pmsg      = &pch->RxBuf[0];
    rx_size   =  pch->RxBufByteCtr;
    prx_data  = &pch->RxFrameData[0];
    if ((rx_size & 0x01)                                     &&        /* Message should have an ODD nbr of bytes.        */
        (rx_size            > MODBUS_ASCII_MIN_MSG_SIZE)     &&        /* Check if message is long enough                 */
        (pmsg[0]           == MODBUS_ASCII_START_FRAME_CHAR) &&        /* Check the first char.                           */
        (pmsg[rx_size - 2] == MODBUS_ASCII_END_FRAME_CHAR1)  &&        /* Check the last two.                             */
        (pmsg[rx_size - 1] == MODBUS_ASCII_END_FRAME_CHAR2)) {
        rx_size               -= 3;                                    /* Take away for the ':', CR, and LF               */
        pmsg++;                                                        /* Point past the ':' to the address.              */
        pch->RxFrameNDataBytes = 0;                                    /* Get the data from the message                   */
        while (rx_size > 2) {
            *prx_data++  = MB_ASCII_HexToBin(pmsg);
            pmsg        += 2;
            rx_size     -= 2;
            pch->RxFrameNDataBytes++;                                  /* Increment the number of Modbus packets received */
        }
        pch->RxFrameNDataBytes -= 2;                                   /* Subtract the Address and function code          */
        pch->RxFrameCRC         = (CPU_INT16U)MB_ASCII_HexToBin(pmsg); /* Extract the message's LRC                       */
        return (DEF_TRUE);
    } else {
        return (DEF_FALSE);
    }
}
#endif

 
/*
*********************************************************************************************************
*                                             MB_ASCII_Tx()
*
* Description : The format of the message is ASCII.  The actual information is taken from the given
*               MODBUS frame.
*
* Argument(s) : pch      Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : none.
*
* Caller(s)   : MBM_TxCmd(),
*               MBS_ASCII_Task().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_ASCII_EN == DEF_ENABLED)
void  MB_ASCII_Tx (MODBUS_CH  *pch)
{
    CPU_INT08U  *ptx_data;
    CPU_INT08U  *pbuf;
    CPU_INT16U   i;
    CPU_INT16U   tx_bytes;
    CPU_INT08U   lrc;


    ptx_data = &pch->TxFrameData[0];
    pbuf     = &pch->TxBuf[0];
    *pbuf++  = MODBUS_ASCII_START_FRAME_CHAR;                   /* Place the start-of-frame character into output buffer  */
    pbuf     = MB_ASCII_BinToHex(*ptx_data++,
                                 pbuf);
    pbuf     = MB_ASCII_BinToHex(*ptx_data++,
                                 pbuf);
    tx_bytes = 5;
    i        = (CPU_INT08U)pch->TxFrameNDataBytes;              /* Transmit the actual data                               */
    while (i > 0) {
        pbuf      = MB_ASCII_BinToHex(*ptx_data++,
                                      pbuf);
        tx_bytes += 2;
        i--;
    }
    lrc               = MB_ASCII_TxCalcLRC(pch,                 /* Compute outbound packet LRC                            */
                                           tx_bytes);
    pbuf              = MB_ASCII_BinToHex(lrc,                  /* Add the LRC checksum in the packet                     */
                                          pbuf);
    *pbuf++           = MODBUS_ASCII_END_FRAME_CHAR1;           /* Add 1st end-of-frame character (0x0D) to output buffer */
    *pbuf++           = MODBUS_ASCII_END_FRAME_CHAR2;           /* Add 2nd end-of-frame character (0x0A) to output buffer */
    tx_bytes         += 4;
    pch->TxFrameCRC   = (CPU_INT16U)lrc;                        /* Save the computed LRC into the channel                 */
    pch->TxBufByteCtr = tx_bytes;                               /* Update the total number of bytes to send               */
    MB_Tx(pch);                                                 /* Send it out the communication driver.                  */
}
#endif

 
/*
*********************************************************************************************************
*                                            MB_RTU_RxByte()
*
* Description : A byte has been received from a serial port.  We just store it in the buffer for processing
*               when a complete packet has been received.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
*               rx_byte     Is the byte received.
*
* Return(s)   : none.
*
* Caller(s)   : MB_RxByte().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_RxByte (MODBUS_CH  *pch,
                     CPU_INT08U  rx_byte)
{
    MB_RTU_TmrReset(pch);                                       /* Reset the timeout timer on a new character             */
#if (MODBUS_CFG_MASTER_EN == DEF_ENABLED)
    if (pch->MasterSlave == MODBUS_MASTER) {
        pch->RTU_TimeoutEn = MODBUS_TRUE;
    }
#endif
    if (pch->RxBufByteCtr < MODBUS_CFG_BUF_SIZE) {              /* No, add received byte to buffer                        */
        pch->RxCtr++;                                           /* Increment the number of bytes received                 */
        *pch->RxBufPtr++ = rx_byte;
        pch->RxBufByteCtr++;                                    /* Increment byte counter to see if we have Rx activity   */
    }
}
#endif

 
/*
*********************************************************************************************************
*                                              MB_RTU_Rx()
*
* Description : Parses a Modbus RTU packet and processes the request if the packet is valid.
*
* Argument(s) : pch         Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : DEF_TRUE    If all checks pass.
*               DEF_FALSE   If any checks fail.
*
* Caller(s)   : MBM_RxReply(),
*               MBS_RTU_Task().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
CPU_BOOLEAN  MB_RTU_Rx (MODBUS_CH  *pch)
{
    CPU_INT08U  *prx_data;
    CPU_INT08U  *pmsg;
    CPU_INT16U   rx_size;
    CPU_INT16U   crc;


    pmsg    = &pch->RxBuf[0];
    rx_size =  pch->RxBufByteCtr;
    if (rx_size >= MODBUS_RTU_MIN_MSG_SIZE) {         /* Is the message long enough?                        */
        if (rx_size <= MODBUS_CFG_BUF_SIZE) {
            prx_data    = &pch->RxFrameData[0];
            *prx_data++ = *pmsg++;                    /* Transfer the node address                          */
            rx_size--;

            *prx_data++ = *pmsg++;                    /* Transfer the function code                         */
            rx_size--;

            pch->RxFrameNDataBytes = 0;               /* Transfer the data                                  */
            while (rx_size > 2) {
                *prx_data++ = *pmsg++;
                pch->RxFrameNDataBytes++;
                rx_size--;
            }

            crc              = (CPU_INT16U)*pmsg++;   /* Transfer the CRC over.  It's LSB first, then MSB.  */
            crc             += (CPU_INT16U)*pmsg << 8;
            pch->RxFrameCRC  = crc;
            return (DEF_TRUE);
        } else {
            return (DEF_FALSE);
        }
    } else {
        return (DEF_FALSE);
    }
}
#endif

 
/*
*********************************************************************************************************
*                                              MB_RTU_Tx()
*
* Description : A MODBUS message is formed into a buffer and sent to the appropriate communication port.
*               The actual reply is taken from the given MODBUS Frame.
*
* Argument(s) : pch      Is a pointer to the Modbus channel's data structure.
*
* Return(s)   : none.
*
* Caller(s)   : MBM_TxCmd(),
*               MBS_RTU_Task().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_Tx (MODBUS_CH  *pch)
{
    CPU_INT08U  *ptx_data;
    CPU_INT08U  *pbuf;
    CPU_INT08U   i;
    CPU_INT16U   tx_bytes;
    CPU_INT16U   crc;


    tx_bytes  = 0;
    pbuf      = &pch->TxBuf[0];                                    /* Point to the beginning of the output buffer.             */
    ptx_data  = &(pch->TxFrameData[0]);
    i         = (CPU_INT08U)pch->TxFrameNDataBytes + 2;            /* Include the actual data in the buffer                    */
    while (i > 0) {
        *pbuf++ = *ptx_data++;
        tx_bytes++;
        i--;
    }
    crc               = MB_RTU_TxCalcCRC(pch);
    *pbuf++           = (CPU_INT08U)(crc & 0x00FF);                /* Add in the CRC checksum.  Low byte first!                */
    *pbuf             = (CPU_INT08U)(crc >> 8);
    tx_bytes         += 2;
    pch->TxFrameCRC   = crc;                                       /* Save the calculated CRC in the channel                   */
    pch->TxBufByteCtr = tx_bytes;

    MB_Tx(pch);                                                    /* Send it out the communication driver.                    */
}
#endif

 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrReset()
*
* Description : This function is called when a byte a received and thus, we reset the RTU timeout timer value
*               indicating that we are not done receiving a complete RTU packet.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_RTU_TmrResetAll().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_TmrReset (MODBUS_CH  *pch)
{
    pch->RTU_TimeoutCtr = pch->RTU_TimeoutCnts;
}
#endif

 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrResetAll()
*
* Description : This function is used to reset all the RTU timers for all Modbus channels.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_RTU_TmrInit().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_TmrResetAll (void)
{
    CPU_INT08U   ch;
    MODBUS_CH   *pch;


    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {
        if (pch->Mode == MODBUS_MODE_RTU) {
            MB_RTU_TmrReset(pch);
        }
        pch++;
    }
}
#endif

 
/*
*********************************************************************************************************
*                                           MB_RTU_TmrUpdate()
*
* Description : This function is called when the application supplied RTU framing timer expires.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : MB_RTU_TmrISR_Handler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
void  MB_RTU_TmrUpdate (void)
{
    CPU_INT08U   ch;
    MODBUS_CH   *pch;


    pch = &MB_ChTbl[0];
    for (ch = 0; ch < MODBUS_CFG_MAX_CH; ch++) {
        if (pch->Mode == MODBUS_MODE_RTU) {
            if (pch->RTU_TimeoutEn == DEF_TRUE) {
                if (pch->RTU_TimeoutCtr > 0) {
                    pch->RTU_TimeoutCtr--;
                    if (pch->RTU_TimeoutCtr == 0) {
#if (MODBUS_CFG_RTU_EN == DEF_ENABLED)
                        if (pch->MasterSlave == MODBUS_MASTER) {
                            pch->RTU_TimeoutEn = DEF_FALSE;
                        }
#endif
                        MB_OS_RxSignal(pch);          /* RTU Timer expired for this Modbus channel         */
                    }
                }
            } else {
                pch->RTU_TimeoutCtr = pch->RTU_TimeoutCnts;
            }
        }
        pch++;
    }
}
#endif
