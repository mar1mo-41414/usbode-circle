//
// btuarttransport.h
//
// UART (PL011) transport to the onboard Bluetooth controller.
//
// Derived from Circle's former Bluetooth support:
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2014-2016  R. Stange <rsta2@o2online.de>
//
// Extended for USBODE with HCI ACL data support (needed for BLE GATT).
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _bleservice_btuarttransport_h
#define _bleservice_btuarttransport_h

#include <circle/device.h>
#include <circle/gpiopin.h>
#include <circle/interrupt.h>
#include <circle/types.h>
#include "blehci.h"

typedef void TBTHCIEventHandler (const void *pBuffer, unsigned nLength, void *pParam);
typedef void TBTHCIACLHandler (const void *pBuffer, unsigned nLength, void *pParam);

class CBTUARTTransport : public CDevice
{
public:
    CBTUARTTransport (CInterruptSystem *pInterruptSystem);
    ~CBTUARTTransport (void);

    boolean Initialize (unsigned nBaudrate = 115200);

    boolean SendHCICommand (const void *pBuffer, unsigned nLength);
    boolean SendHCIACLData (const void *pBuffer, unsigned nLength);

    void RegisterHCIEventHandler (TBTHCIEventHandler *pHandler, void *pParam);
    void RegisterHCIACLHandler (TBTHCIACLHandler *pHandler, void *pParam);

    unsigned GetRxByteCount (void) const    { return m_nRxTotal; }
    unsigned GetRxErrorCount (void) const   { return m_nRxErrors; }
    boolean GetLoopbackOK (void) const      { return m_bLoopbackOK; }

    // first raw bytes received, for diagnostics (returns count copied)
    unsigned CopyRxTrace (u8 *pBuffer, unsigned nMaxLength) const;

    // direct polled access, bypassing the RX interrupt/queue - used to
    // probe whether the controller answers at all
    void SetRawMode (boolean bRaw);         // disable/enable the RX interrupt
    int ReadRawByte (void);                 // returns 0..255, or -1 if none

    // drain the RX FIFO and dispatch complete HCI packets - call regularly
    // from the task, since the RX interrupt does not reach us on this board
    void Poll (void);

    // discard any buffered RX bytes and reset the framing state machine,
    // e.g. after the controller reboots into patched firmware
    void FlushRx (void);

    // reprogram the UART to a new baud rate (the controller may switch
    // baud after loading patched firmware)
    void SetBaud (unsigned nBaudrate);

private:
    void Write (u8 nChar);
    void ProcessRxByte (u8 nData);
    void IRQHandler (void);
    static void IRQStub (void *pParam);

private:
    CGPIOPin m_GPIO14;
    CGPIOPin m_GPIO15;
    CGPIOPin m_TxDPin;
    CGPIOPin m_RxDPin;
    CGPIOPin m_CTSPin;
    CGPIOPin m_RTSPin;

    CInterruptSystem *m_pInterruptSystem;
    boolean m_bIRQConnected;

    TBTHCIEventHandler *m_pEventHandler;
    void *m_pEventParam;
    TBTHCIACLHandler *m_pACLHandler;
    void *m_pACLParam;

    u8 m_RxBuffer[BT_MAX_ACL_SIZE];
    volatile unsigned m_nRxTotal;
    volatile unsigned m_nRxErrors;
    boolean m_bLoopbackOK;
#define BT_RX_TRACE_SIZE 32
    u8 m_RxTrace[BT_RX_TRACE_SIZE];
    unsigned m_nRxState;
    unsigned m_nRxInPtr;
    unsigned m_nRxParamLength;
    unsigned m_nRxHeaderLength;
};

#endif
