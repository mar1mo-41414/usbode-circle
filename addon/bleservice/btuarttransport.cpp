//
// btuarttransport.cpp
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
#include "btuarttransport.h"
#include <circle/devicenameservice.h>
#include <circle/bcm2835.h>
#include <circle/memio.h>
#include <circle/machineinfo.h>
#include <circle/synchronize.h>
#include <circle/logger.h>
#include <circle/util.h>
#include <assert.h>

#define FR_TXFF_MASK            (1 << 5)
#define FR_RXFE_MASK            (1 << 4)

#define LCRH_WLEN8_MASK         (3 << 5)
#define LCRH_FEN_MASK           (1 << 4)

#define CR_RTSEN_MASK           (1 << 14)
#define CR_RXE_MASK             (1 << 9)
#define CR_TXE_MASK             (1 << 8)
#define CR_LBE_MASK             (1 << 7)
#define CR_UART_EN_MASK         (1 << 0)

#define DR_ERROR_MASK           (0xF << 8)      // OE | BE | PE | FE

#define FR_BUSY_MASK            (1 << 3)

#define IFLS_RXIFSEL_SHIFT      3
#define IFLS_IFSEL_1_4          1

#define INT_OE                  (1 << 10)
#define INT_RT                  (1 << 6)
#define INT_RX                  (1 << 4)

enum TBTUARTRxState
{
    RxStateStart,
    RxStateEventCode,
    RxStateEventLength,
    RxStateEventParam,
    RxStateACLHeader,
    RxStateACLData,
    RxStateUnknown
};

static const char FromBTUART[] = "btuart";

CBTUARTTransport::CBTUARTTransport (CInterruptSystem *pInterruptSystem)
:       // to be sure there is no collision with the UART GPIO interface
    m_GPIO14 (14, GPIOModeInput),
    m_GPIO15 (15, GPIOModeInput),
    m_TxDPin (32, GPIOModeAlternateFunction3),
    m_RxDPin (33, GPIOModeAlternateFunction3),
    // Flow control lines to the BT controller. RTS (GPIO 31) is driven by
    // the PL011 (RTSEN): it deasserts automatically when our RX FIFO fills,
    // telling the controller to pause, so bursty ACL traffic can't overflow
    // the FIFO while the cooperative scheduler is busy elsewhere.
    // GPIO 30 (the chip's RTS -> our CTS) is read only, for diagnostics.
    m_CTSPin (30, GPIOModeInput),
    m_RTSPin (31, GPIOModeAlternateFunction3),
    m_pInterruptSystem (pInterruptSystem),
    m_bIRQConnected (FALSE),
    m_pEventHandler (0),
    m_pEventParam (0),
    m_pACLHandler (0),
    m_pACLParam (0),
    m_nRxTotal (0),
    m_nRxErrors (0),
    m_bLoopbackOK (FALSE),
    m_nRxState (RxStateStart)
{
}

CBTUARTTransport::~CBTUARTTransport (void)
{
    PeripheralEntry ();
    write32 (ARM_UART0_IMSC, 0);
    write32 (ARM_UART0_CR, 0);
    PeripheralExit ();

    m_pEventHandler = 0;
    m_pACLHandler = 0;

    if (m_bIRQConnected)
    {
        assert (m_pInterruptSystem != 0);
        m_pInterruptSystem->DisconnectIRQ (ARM_IRQ_UART);
    }

    m_pInterruptSystem = 0;
}

boolean CBTUARTTransport::Initialize (unsigned nBaudrate)
{
    unsigned nClockRate = CMachineInfo::Get ()->GetClockRate (CLOCK_ID_UART);
    assert (nClockRate > 0);
    CLogger::Get ()->Write (FromBTUART, LogNotice,
                "UART clock %u Hz, baud %u", nClockRate, nBaudrate);

    assert (300 <= nBaudrate && nBaudrate <= 3000000);
    unsigned nBaud16 = nBaudrate * 16;
    unsigned nIntDiv = nClockRate / nBaud16;
    assert (1 <= nIntDiv && nIntDiv <= 0xFFFF);
    unsigned nFractDiv2 = (nClockRate % nBaud16) * 8 / nBaudrate;
    unsigned nFractDiv = nFractDiv2 / 2 + nFractDiv2 % 2;
    assert (nFractDiv <= 0x3F);

    // We do not connect the PL011 RX interrupt: on this board it never
    // reaches us (verified - the controller replies but the IRQ never
    // fires). Receiving is done by polling Poll() from the task instead.
    m_bIRQConnected = FALSE;

    PeripheralEntry ();

    // The PL011 may already be enabled (USBODE initializes it for serial
    // logging). The divisor and line control registers only latch while
    // the UART is disabled, so disable it first, wait for the current
    // character, and flush the FIFOs - otherwise the baud rate silently
    // stays unprogrammed and TX never shifts anything out.
    write32 (ARM_UART0_CR, 0);
    for (unsigned i = 0; i < 100000 && (read32 (ARM_UART0_FR) & FR_BUSY_MASK); i++)
    {
        // wait for the end of the current character
    }
    write32 (ARM_UART0_LCRH, 0);            // FEN=0 flushes the FIFOs

    write32 (ARM_UART0_IMSC, 0);
    write32 (ARM_UART0_ICR,  0x7FF);
    write32 (ARM_UART0_IBRD, nIntDiv);
    write32 (ARM_UART0_FBRD, nFractDiv);
    write32 (ARM_UART0_IFLS, IFLS_IFSEL_1_4 << IFLS_RXIFSEL_SHIFT);
    write32 (ARM_UART0_LCRH, LCRH_WLEN8_MASK | LCRH_FEN_MASK);              // 8N1

    // internal loopback self test (before interrupts are enabled):
    // proves baud generation and the TX/RX data path work
    write32 (ARM_UART0_CR,   CR_UART_EN_MASK | CR_TXE_MASK | CR_RXE_MASK | CR_LBE_MASK);
    write32 (ARM_UART0_DR,   0x55);
    for (unsigned i = 0; i < 10000; i++)
    {
        if (!(read32 (ARM_UART0_FR) & FR_RXFE_MASK))
        {
            u32 nData = read32 (ARM_UART0_DR);
            m_bLoopbackOK = (nData & 0xFF) == 0x55
                    && !(nData & DR_ERROR_MASK) ? TRUE : FALSE;
            break;
        }
    }
    while (read32 (ARM_UART0_FR) & FR_BUSY_MASK)
    {
        // wait for the loopback byte to drain
    }
    while (!(read32 (ARM_UART0_FR) & FR_RXFE_MASK))
    {
        read32 (ARM_UART0_DR);          // flush RX FIFO
    }
    write32 (ARM_UART0_CR, 0);
    write32 (ARM_UART0_ICR, 0x7FF);

    // Enable RTS hardware flow control (RTSEN): the PL011 deasserts nRTS
    // when the RX FIFO is nearly full, pausing the controller so a burst
    // cannot overflow the FIFO while we are scheduled out. CTSEN stays off
    // so our (small, tightly-polled) TX never stalls on a floating CTS.
    write32 (ARM_UART0_CR,
         CR_UART_EN_MASK | CR_TXE_MASK | CR_RXE_MASK | CR_RTSEN_MASK);
    write32 (ARM_UART0_IMSC, 0);            // RX handled by polling, not IRQ

    PeripheralExit ();

    // pull RxD up so an idle line reads high
    m_RxDPin.SetPullMode (GPIOPullModeUp);

    CDeviceNameService::Get ()->AddDevice ("ttyBT1", this, FALSE);

    return TRUE;
}

boolean CBTUARTTransport::SendHCICommand (const void *pBuffer, unsigned nLength)
{
    PeripheralEntry ();

    Write (HCI_PACKET_COMMAND);

    const u8 *pChar = (const u8 *) pBuffer;
    assert (pChar != 0);

    while (nLength--)
    {
        Write (*pChar++);
    }

    PeripheralExit ();

    return TRUE;
}

boolean CBTUARTTransport::SendHCIACLData (const void *pBuffer, unsigned nLength)
{
    PeripheralEntry ();

    Write (HCI_PACKET_ACL_DATA);

    const u8 *pChar = (const u8 *) pBuffer;
    assert (pChar != 0);

    while (nLength--)
    {
        Write (*pChar++);
    }

    PeripheralExit ();

    return TRUE;
}

void CBTUARTTransport::RegisterHCIEventHandler (TBTHCIEventHandler *pHandler, void *pParam)
{
    assert (m_pEventHandler == 0);
    m_pEventHandler = pHandler;
    m_pEventParam = pParam;
    assert (m_pEventHandler != 0);
}

void CBTUARTTransport::RegisterHCIACLHandler (TBTHCIACLHandler *pHandler, void *pParam)
{
    assert (m_pACLHandler == 0);
    m_pACLHandler = pHandler;
    m_pACLParam = pParam;
    assert (m_pACLHandler != 0);
}

void CBTUARTTransport::Write (u8 nChar)
{
    while (read32 (ARM_UART0_FR) & FR_TXFF_MASK)
    {
        // do nothing
    }

    write32 (ARM_UART0_DR, nChar);
}

// Drain the RX FIFO and run the framing state machine. Called by polling
// from the task context - the PL011 RX interrupt does not reach us on this
// board, so we do not rely on it.
void CBTUARTTransport::Poll (void)
{
    PeripheralEntry ();

    while (!(read32 (ARM_UART0_FR) & FR_RXFE_MASK))
    {
        u32 nDR = read32 (ARM_UART0_DR);
        u8 nData = nDR & 0xFF;
        if (nDR & DR_ERROR_MASK)
        {
            m_nRxErrors++;
        }
        if (m_nRxTotal < BT_RX_TRACE_SIZE)
        {
            m_RxTrace[m_nRxTotal] = nData;
        }
        m_nRxTotal++;

        ProcessRxByte (nData);
    }

    PeripheralExit ();
}

void CBTUARTTransport::IRQHandler (void)
{
    Poll ();
}

void CBTUARTTransport::FlushRx (void)
{
    PeripheralEntry ();
    while (!(read32 (ARM_UART0_FR) & FR_RXFE_MASK))
    {
        read32 (ARM_UART0_DR);
    }
    PeripheralExit ();

    m_nRxState = RxStateStart;
}

void CBTUARTTransport::SetBaud (unsigned nBaudrate)
{
    unsigned nClockRate = CMachineInfo::Get ()->GetClockRate (CLOCK_ID_UART);
    unsigned nBaud16 = nBaudrate * 16;
    unsigned nIntDiv = nClockRate / nBaud16;
    unsigned nFractDiv2 = (nClockRate % nBaud16) * 8 / nBaudrate;
    unsigned nFractDiv = nFractDiv2 / 2 + nFractDiv2 % 2;

    PeripheralEntry ();

    write32 (ARM_UART0_CR, 0);
    for (unsigned i = 0; i < 100000 && (read32 (ARM_UART0_FR) & FR_BUSY_MASK); i++)
    {
        // wait for the current character to drain
    }
    write32 (ARM_UART0_LCRH, 0);            // flush FIFOs
    write32 (ARM_UART0_ICR,  0x7FF);
    write32 (ARM_UART0_IBRD, nIntDiv);
    write32 (ARM_UART0_FBRD, nFractDiv);
    write32 (ARM_UART0_LCRH, LCRH_WLEN8_MASK | LCRH_FEN_MASK);      // 8N1
    write32 (ARM_UART0_CR,
         CR_UART_EN_MASK | CR_TXE_MASK | CR_RXE_MASK | CR_RTSEN_MASK);

    PeripheralExit ();

    m_nRxState = RxStateStart;
}

void CBTUARTTransport::ProcessRxByte (u8 nData)
{
    {
        switch (m_nRxState)
        {
        case RxStateStart:
            if (nData == HCI_PACKET_EVENT)
            {
                m_nRxInPtr = 0;
                m_nRxState = RxStateEventCode;
            }
            else if (nData == HCI_PACKET_ACL_DATA)
            {
                m_nRxInPtr = 0;
                m_nRxHeaderLength = 0;
                m_nRxState = RxStateACLHeader;
            }
            break;

        case RxStateEventCode:
            m_RxBuffer[m_nRxInPtr++] = nData;
            m_nRxState = RxStateEventLength;
            break;

        case RxStateEventLength:
            m_RxBuffer[m_nRxInPtr++] = nData;
            if (nData > 0)
            {
                m_nRxParamLength = nData;
                m_nRxState = RxStateEventParam;
            }
            else
            {
                if (m_pEventHandler != 0)
                {
                    (*m_pEventHandler) (m_RxBuffer, m_nRxInPtr, m_pEventParam);
                }

                m_nRxState = RxStateStart;
            }
            break;

        case RxStateEventParam:
            assert (m_nRxInPtr < BT_MAX_HCI_EVENT_SIZE);
            m_RxBuffer[m_nRxInPtr++] = nData;
            if (--m_nRxParamLength == 0)
            {
                if (m_pEventHandler != 0)
                {
                    (*m_pEventHandler) (m_RxBuffer, m_nRxInPtr, m_pEventParam);
                }

                m_nRxState = RxStateStart;
            }
            break;

        case RxStateACLHeader:
            m_RxBuffer[m_nRxInPtr++] = nData;
            if (++m_nRxHeaderLength == 4)
            {
                m_nRxParamLength = m_RxBuffer[2] | (m_RxBuffer[3] << 8);
                if (m_nRxParamLength == 0)
                {
                    if (m_pACLHandler != 0)
                    {
                        (*m_pACLHandler) (m_RxBuffer, m_nRxInPtr, m_pACLParam);
                    }

                    m_nRxState = RxStateStart;
                }
                else if (m_nRxParamLength > BT_MAX_ACL_SIZE - 4)
                {
                    // oversized packet, drop it
                    m_nRxState = RxStateStart;
                }
                else
                {
                    m_nRxState = RxStateACLData;
                }
            }
            break;

        case RxStateACLData:
            assert (m_nRxInPtr < BT_MAX_ACL_SIZE);
            m_RxBuffer[m_nRxInPtr++] = nData;
            if (--m_nRxParamLength == 0)
            {
                if (m_pACLHandler != 0)
                {
                    (*m_pACLHandler) (m_RxBuffer, m_nRxInPtr, m_pACLParam);
                }

                m_nRxState = RxStateStart;
            }
            break;

        default:
            assert (0);
            break;
        }
    }
}

void CBTUARTTransport::SetRawMode (boolean bRaw)
{
    // RX interrupts are never enabled (we poll), so this only needs to keep
    // them masked. Kept for API symmetry with the diagnostic probe.
    PeripheralEntry ();
    write32 (ARM_UART0_IMSC, 0);
    PeripheralExit ();
}

int CBTUARTTransport::ReadRawByte (void)
{
    int nResult = -1;

    PeripheralEntry ();
    if (!(read32 (ARM_UART0_FR) & FR_RXFE_MASK))
    {
        u32 nDR = read32 (ARM_UART0_DR);
        m_nRxTotal++;
        if (nDR & DR_ERROR_MASK)
        {
            m_nRxErrors++;
        }
        nResult = nDR & 0xFF;
    }
    PeripheralExit ();

    return nResult;
}

unsigned CBTUARTTransport::CopyRxTrace (u8 *pBuffer, unsigned nMaxLength) const
{
    unsigned nCount = m_nRxTotal;
    if (nCount > BT_RX_TRACE_SIZE)
    {
        nCount = BT_RX_TRACE_SIZE;
    }
    if (nCount > nMaxLength)
    {
        nCount = nMaxLength;
    }

    memcpy (pBuffer, m_RxTrace, nCount);

    return nCount;
}

void CBTUARTTransport::IRQStub (void *pParam)
{
    CBTUARTTransport *pThis = (CBTUARTTransport *) pParam;
    assert (pThis != 0);

    pThis->IRQHandler ();
}
