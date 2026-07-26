//
// bleservice.cpp
//
// BLE serial (Nordic UART Service) console for USBODE.
//
// Implements a minimal BLE peripheral directly on HCI: firmware download
// to the onboard Broadcom/Cypress controller, LE advertising, and an ATT
// server exposing the Nordic UART Service (NUS). A tiny command shell on
// top of NUS lets any BLE serial terminal app list and switch disc images
// without WiFi.
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
#include "bleservice.h"

#include <circle/bcm2835.h>
#include <circle/bcmpropertytags.h>
#include <circle/gpioclock.h>
#include <circle/memio.h>
#include <circle/gpiopin.h>
#include <circle/interrupt.h>
#include <circle/logger.h>
#include <circle/machineinfo.h>
#include <circle/sched/scheduler.h>
#include <circle/synchronize.h>
#include <circle/timer.h>
#include <circle/util.h>
#include <fatfs/ff.h>
#include <gitinfo/gitinfo.h>
#include <scsitbservice/scsitbservice.h>
#include <devicestate/devicestate.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LOGMODULE("bleservice");

#define FIRMWARE_DIR            "0:/firmware/"

#define BLE_DEVICE_NAME         "USBODE"

// attribute handles
#define HANDLE_GAP_SERVICE      0x0001
#define HANDLE_NAME_DECL        0x0002
#define HANDLE_NAME_VALUE       0x0003
#define HANDLE_APPEARANCE_DECL  0x0004
#define HANDLE_APPEARANCE_VALUE 0x0005
#define HANDLE_GAP_END          0x0005
#define HANDLE_NUS_SERVICE      0x0010
#define HANDLE_NUS_RX_DECL      0x0011
#define HANDLE_NUS_RX_VALUE     0x0012
#define HANDLE_NUS_TX_DECL      0x0013
#define HANDLE_NUS_TX_VALUE     0x0014
#define HANDLE_NUS_TX_CCCD      0x0015
#define HANDLE_NUS_END          0x0015
#define HANDLE_MAX              HANDLE_NUS_END

// Nordic UART Service UUIDs in little-endian byte order
static const u8 UUID_NUS_SERVICE[16] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

static const u8 UUID_NUS_RX[16] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

static const u8 UUID_NUS_TX[16] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

CBLEService *CBLEService::s_pThis = 0;

CBLEService::CBLEService (void)
:       m_pTransport (0),
    m_bRunning (FALSE),
    m_nLEBufferSize (27),
    m_nLEBufferCount (4),
    m_nPacketsInFlight (0),
    m_bConnected (FALSE),
    m_nConnectionHandle (0),
    m_nATTMTU (23),
    m_bNotificationsEnabled (FALSE),
    m_nL2CAPReceived (0),
    m_nL2CAPExpected (0),
    m_nTxHead (0),
    m_nTxTail (0),
    m_nCmdLength (0),
    m_pFirmware (0),
    m_nFirmwareSize (0),
    m_pBTPowerPin (0),
    m_pLPOClockPin (0),
    m_pLPOClock (0)
{
    assert (s_pThis == 0);
    s_pThis = this;

    memset (&m_EventQueue, 0, sizeof m_EventQueue);
    memset (&m_ACLQueue, 0, sizeof m_ACLQueue);
    memset (m_LocalBDAddr, 0, sizeof m_LocalBDAddr);

    SetName ("bleservice");
}

CBLEService::~CBLEService (void)
{
    delete m_pLPOClockPin;
    m_pLPOClockPin = 0;

    delete m_pLPOClock;
    m_pLPOClock = 0;

    delete m_pBTPowerPin;
    m_pBTPowerPin = 0;

    delete m_pTransport;
    m_pTransport = 0;

    delete [] m_pFirmware;
    m_pFirmware = 0;

    s_pThis = 0;
}

CBLEService *CBLEService::Get (void)
{
    return s_pThis;
}

//
// packet queues (producer: IRQ, consumer: task)
//

void CBLEService::EventStub (const void *pBuffer, unsigned nLength, void *pParam)
{
    CBLEService *pThis = (CBLEService *) pParam;
    pThis->EnqueuePacket (&pThis->m_EventQueue, pBuffer, nLength);
}

void CBLEService::ACLStub (const void *pBuffer, unsigned nLength, void *pParam)
{
    CBLEService *pThis = (CBLEService *) pParam;
    pThis->EnqueuePacket (&pThis->m_ACLQueue, pBuffer, nLength);
}

void CBLEService::EnqueuePacket (TPacketQueue *pQueue, const void *pBuffer, unsigned nLength)
{
    if (nLength > BT_MAX_ACL_SIZE)
    {
        return;
    }

    unsigned nNext = (pQueue->nHead + 1) % BLE_RX_QUEUE_SIZE;
    if (nNext == pQueue->nTail)
    {
        return;         // queue full, drop packet
    }

    TPacket *pPacket = &pQueue->Packet[pQueue->nHead];
    pPacket->nLength = (u16) nLength;
    memcpy (pPacket->Data, pBuffer, nLength);

    DataMemBarrier ();

    pQueue->nHead = nNext;
}

boolean CBLEService::DequeuePacket (TPacketQueue *pQueue, TPacket *pPacket)
{
    if (pQueue->nTail == pQueue->nHead)
    {
        return FALSE;
    }

    EnterCritical ();

    memcpy (pPacket, &pQueue->Packet[pQueue->nTail], sizeof (TPacket));
    pQueue->nTail = (pQueue->nTail + 1) % BLE_RX_QUEUE_SIZE;

    LeaveCritical ();

    return TRUE;
}

//
// HCI helpers
//

boolean CBLEService::SendCommand (const void *pBuffer, unsigned nLength)
{
    assert (m_pTransport != 0);
    return m_pTransport->SendHCICommand (pBuffer, nLength);
}

boolean CBLEService::WaitForCommandComplete (u16 nOpCode, unsigned nTimeoutMs,
                         u8 *pReturn, unsigned nReturnSize)
{
    // Busy-poll without yielding. Circle's scheduler is cooperative, so a
    // MsSleep() here hands the CPU to other tasks (USB gadget, etc.) that
    // can run for several ms - long enough for the 32-byte PL011 FIFO to
    // overflow mid-reply at 115200 (it fills in ~2.8 ms). A tight 250 us
    // busy-wait keeps the FIFO drained so even a 255-byte reply survives.
    unsigned nElapsedUs = 0;
    unsigned nTimeoutUs = nTimeoutMs * 1000;

    while (nElapsedUs < nTimeoutUs)
    {
        m_pTransport->Poll ();

        TPacket Packet;
        while (DequeuePacket (&m_EventQueue, &Packet))
        {
            TBTHCIEventHeader *pHeader = (TBTHCIEventHeader *) Packet.Data;

            if (   pHeader->EventCode == EVENT_CODE_COMMAND_COMPLETE
                && Packet.nLength >= sizeof (TBTHCIEventCommandComplete))
            {
                TBTHCIEventCommandComplete *pComplete =
                    (TBTHCIEventCommandComplete *) Packet.Data;

                if (pComplete->CommandOpCode == nOpCode)
                {
                    if (pComplete->Status != BT_STATUS_SUCCESS)
                    {
                        LOGERR ("Command 0x%04X failed (status 0x%02X)",
                            (unsigned) nOpCode,
                            (unsigned) pComplete->Status);
                        return FALSE;
                    }

                    if (pReturn != 0 && nReturnSize > 0)
                    {
                        unsigned nAvail = Packet.nLength
                            - sizeof (TBTHCIEventCommandComplete);
                        if (nAvail > nReturnSize)
                        {
                            nAvail = nReturnSize;
                        }
                        memcpy (pReturn, pComplete->ReturnParameter, nAvail);
                    }

                    return TRUE;
                }
            }
            else
            {
                // process unrelated events (e.g. disconnects)
                ProcessEvent (Packet.Data, Packet.nLength);
            }
        }

        CTimer::SimpleusDelay (250);
        nElapsedUs += 250;
    }

    LOGERR ("Timeout waiting for command 0x%04X", (unsigned) nOpCode);

    return FALSE;
}

//
// controller bring-up
//

boolean CBLEService::HasOnboardBluetooth (void)
{
    switch (CMachineInfo::Get ()->GetMachineModel ())
    {
    case MachineModelZeroW:
    case MachineModelZero2W:
    case MachineModel3B:
    case MachineModel3APlus:
    case MachineModel3BPlus:
    case MachineModel4B:
    case MachineModel400:
    case MachineModelCM4:
        return TRUE;

    default:
        return FALSE;           // no onboard Bluetooth
    }
}

boolean CBLEService::LoadFirmware (const char *pFileName)
{
    // release any previously loaded patch
    delete [] m_pFirmware;
    m_pFirmware = 0;
    m_nFirmwareSize = 0;

    char Path[128];
    snprintf (Path, sizeof Path, FIRMWARE_DIR "%s", pFileName);

    FIL File;
    if (f_open (&File, Path, FA_READ) != FR_OK)
    {
        LOGERR ("Cannot open BT firmware: %s", Path);
        return FALSE;
    }

    m_nFirmwareSize = f_size (&File);
    if (m_nFirmwareSize == 0 || m_nFirmwareSize > 0x40000)
    {
        LOGERR ("Invalid BT firmware size: %u", m_nFirmwareSize);
        f_close (&File);
        return FALSE;
    }

    m_pFirmware = new u8[m_nFirmwareSize];
    assert (m_pFirmware != 0);

    UINT nBytesRead = 0;
    FRESULT Result = f_read (&File, m_pFirmware, m_nFirmwareSize, &nBytesRead);
    f_close (&File);

    if (Result != FR_OK || nBytesRead != m_nFirmwareSize)
    {
        LOGERR ("Cannot read BT firmware: %s", Path);
        delete [] m_pFirmware;
        m_pFirmware = 0;
        return FALSE;
    }

    LOGNOTE ("Loaded BT firmware %s (%u bytes)", Path, m_nFirmwareSize);

    return TRUE;
}

void CBLEService::PowerOnController (void)
{
    // Assert BT_REG_ON, matching the Linux device trees for each board.
    // Without this the controller may be held in reset and never answer.
    unsigned nPowerGPIO = 0;
    boolean bNeedsLPO = FALSE;
    switch (CMachineInfo::Get ()->GetMachineModel ())
    {
    case MachineModelZeroW:                         // bcm2835-rpi-zero-w.dts
        nPowerGPIO = 45;
        bNeedsLPO = TRUE;
        break;

    case MachineModelZero2W:                        // bcm2710-rpi-zero-2-w.dts
        nPowerGPIO = 42;
        bNeedsLPO = TRUE;
        break;

    case MachineModel3B:
    case MachineModel3APlus:
    case MachineModel3BPlus:
        bNeedsLPO = TRUE;
        // fall through - BT_ON is on the firmware GPIO expander
    case MachineModel4B:
    case MachineModel400:
    case MachineModelCM4: {
        // BT_ON is pin 0 of the firmware GPIO expander - power cycle it
        CBcmPropertyTags Tags;
        TPropertyTagGPIOState GPIOState;
        GPIOState.nGPIO = EXP_GPIO_BASE + 0;
        GPIOState.nState = 0;
        Tags.GetTag (PROPTAG_SET_SET_GPIO_STATE, &GPIOState, sizeof GPIOState, 8);
        CScheduler::Get ()->MsSleep (100);
        GPIOState.nGPIO = EXP_GPIO_BASE + 0;
        GPIOState.nState = 1;
        if (Tags.GetTag (PROPTAG_SET_SET_GPIO_STATE, &GPIOState, sizeof GPIOState, 8))
        {
            LOGNOTE ("Asserted BT_ON (expander GPIO 0)");
        }
        else
        {
            LOGWARN ("Cannot set BT_ON via firmware GPIO expander");
        }
        } break;

    default:
        break;
    }

    if (bNeedsLPO)
    {
        // The controller needs a 32.768 kHz LPO ("sleep") clock on its
        // LPO input, wired to GPCLK2 / GPIO 43 (bt_pins in the Linux
        // device trees). Without it the chip does not start up.
        // 19.2 MHz oscillator / 585.9375 = 32768.0 Hz
        m_pLPOClock = new CGPIOClock (GPIOClock2, GPIOClockSourceOscillator);
        if (m_pLPOClock->Start (585, 3840, 1))
        {
            m_pLPOClockPin = new CGPIOPin (43, GPIOModeAlternateFunction0);
            LOGNOTE ("Started 32.768 kHz LPO clock (GPCLK2 on GPIO 43)");
        }
        else
        {
            LOGWARN ("Cannot start LPO clock for BT controller");
        }
    }

    if (nPowerGPIO != 0)
    {
        // power cycle BT_REG_ON for a clean controller boot
        m_pBTPowerPin = new CGPIOPin (nPowerGPIO, GPIOModeOutput);
        m_pBTPowerPin->Write (LOW);
        CScheduler::Get ()->MsSleep (300);
        m_pBTPowerPin->Write (HIGH);
        LOGNOTE ("Asserted BT_REG_ON (GPIO %u)", nPowerGPIO);
    }

    // give the controller time to boot after power-on
    CScheduler::Get ()->MsSleep (1000);
}

void CBLEService::LogPinDiagnostics (void)
{
    // raw pin/clock state, to verify the setup actually took effect
    u32 nGPFSEL3 = read32 (ARM_GPIO_BASE + 0x0C);   // GPIOs 30-39
    u32 nGPFSEL4 = read32 (ARM_GPIO_BASE + 0x10);   // GPIOs 40-49

    // expected: 30-33 = ALT3 (0b111), 42 = output (0b001), 43 = ALT0 (0b100)
    LOGNOTE ("GPFSEL: 30=%u 31=%u 32=%u 33=%u 42=%u 43=%u",
         (nGPFSEL3 >> 0) & 7, (nGPFSEL3 >> 3) & 7,
         (nGPFSEL3 >> 6) & 7, (nGPFSEL3 >> 9) & 7,
         (nGPFSEL4 >> 6) & 7, (nGPFSEL4 >> 9) & 7);

    // GPCLK2 control/divisor (BUSY bit 7 means the clock is running)
    u32 nGP2Ctl = read32 (ARM_CM_BASE + 0x80);
    u32 nGP2Div = read32 (ARM_CM_BASE + 0x84);
    LOGNOTE ("GPCLK2: CTL=0x%X DIV=0x%X (busy=%u)",
         nGP2Ctl, nGP2Div, (nGP2Ctl >> 7) & 1);

    // actual pin levels: BT_REG_ON must be high, and the chip's RTS
    // (GPIO 30) tells us whether the controller's UART is alive
    u32 nLev0 = read32 (ARM_GPIO_BASE + 0x34);      // GPLEV0: GPIOs 0-31
    u32 nLev1 = read32 (ARM_GPIO_BASE + 0x38);      // GPLEV1: GPIOs 32-53
    LOGNOTE ("Levels: BT_REG_ON(42)=%u chipRTS(30)=%u ourRTS(31)=%u",
         (nLev1 >> (42 - 32)) & 1, (nLev0 >> 30) & 1, (nLev0 >> 31) & 1);

    // sample the controller's TX line (GPIO 33): a powered controller
    // drives its UART TX idle-high; constant low means it is dead
    u32 nLevel = read32 (ARM_GPIO_BASE + 0x38);     // GPLEV1: GPIOs 32-53
    unsigned nHigh = (nLevel >> 1) & 1;             // GPIO 33
    unsigned nTransitions = 0;
    unsigned nLast = nHigh;
    for (unsigned i = 0; i < 10000; i++)
    {
        unsigned nNow = (read32 (ARM_GPIO_BASE + 0x38) >> 1) & 1;
        if (nNow != nLast)
        {
            nTransitions++;
            nLast = nNow;
        }
        if (nNow)
        {
            nHigh++;
        }
    }
    LOGNOTE ("BT TX line (GPIO 33): high %u/10001 samples, %u transitions",
         nHigh, nTransitions);
}

boolean CBLEService::InitializeController (void)
{
    // HCI reset - retried, since the controller may still be starting up
    // after BT_REG_ON was asserted. The reply is captured by polling
    // (Poll() inside WaitForCommandComplete), not by an interrupt.
    TBTHCICommandHeader Cmd;
    boolean bResetOK = FALSE;
    for (unsigned nTry = 1; nTry <= 5 && !bResetOK; nTry++)
    {
        Cmd.OpCode = OP_CODE_RESET;
        Cmd.ParameterTotalLength = 0;
        SendCommand (&Cmd, sizeof Cmd);
        bResetOK = WaitForCommandComplete (OP_CODE_RESET, 1000);

        if (!bResetOK)
        {
            LOGWARN ("HCI reset attempt %u failed (%u bytes received so far)",
                 nTry, m_pTransport->GetRxByteCount ());
            CScheduler::Get ()->MsSleep (200);
        }
    }
    if (!bResetOK)
    {
        LOGERR ("Controller not responding (%u bytes received, %u RX errors)",
            m_pTransport->GetRxByteCount (),
            m_pTransport->GetRxErrorCount ());

        // hex dump of whatever arrived, for diagnosis
        u8 Trace[BT_RX_TRACE_SIZE];
        unsigned nTrace = m_pTransport->CopyRxTrace (Trace, sizeof Trace);
        if (nTrace > 0)
        {
            char Line[3 * BT_RX_TRACE_SIZE + 1];
            for (unsigned i = 0; i < nTrace; i++)
            {
                snprintf (&Line[i * 3], 4, "%02X ", (unsigned) Trace[i]);
            }
            LOGERR ("RX trace: %s", Line);
        }

        LogPinDiagnostics ();

        return FALSE;
    }

    LOGNOTE ("HCI reset OK, controller responding");

    // Ask the controller its name - Broadcom parts return their exact part
    // number here (e.g. "BCM43430B0"), which is what selects the matching
    // .hcd firmware. Logged so we can confirm we load the right file.
    {
        u8 Name[64];
        memset (Name, 0, sizeof Name);
        TBTHCICommandHeader NameCmd;
        NameCmd.OpCode = OGF_HCI_CONTROL_BASEBAND | 0x014;      // Read_Local_Name
        NameCmd.ParameterTotalLength = 0;
        SendCommand (&NameCmd, sizeof NameCmd);
        if (WaitForCommandComplete (NameCmd.OpCode, 1000, Name, sizeof Name - 1))
        {
            // stop at the first NUL (keeps the C-string terminated);
            // sanitise any non-printable bytes before it
            for (unsigned i = 0; i < sizeof Name - 1; i++)
            {
                if (Name[i] == 0)
                {
                    break;
                }
                if (Name[i] < 0x20 || Name[i] > 0x7E)
                {
                    Name[i] = '?';
                }
            }
            LOGNOTE ("Controller name: '%s'", Name);

            // Select the patch firmware from the chip's own part
            // number - the reliable way, since board-model guesses
            // were wrong (the Zero 2 W reports BCM43430A1, not B0).
            char FwFile[80];
            snprintf (FwFile, sizeof FwFile, "%s.hcd", (const char *) Name);
            if (LoadFirmware (FwFile))
            {
                LOGNOTE ("Using firmware %s for '%s'", FwFile, Name);
            }
            else
            {
                LOGWARN ("No matching firmware %s - trying without patch",
                     FwFile);
            }
        }
    }

    // download controller patch firmware (.hcd)
    if (m_pFirmware != 0)
    {
        Cmd.OpCode = OP_CODE_DOWNLOAD_MINIDRIVER;
        Cmd.ParameterTotalLength = 0;
        SendCommand (&Cmd, sizeof Cmd);
        WaitForCommandComplete (OP_CODE_DOWNLOAD_MINIDRIVER, 500);
        CScheduler::Get ()->MsSleep (50);

        unsigned nOffset = 0;
        while (nOffset + 3 <= m_nFirmwareSize)
        {
            u16 nOpCode  = m_pFirmware[nOffset++];
            nOpCode     |= m_pFirmware[nOffset++] << 8;
            u8 nLength   = m_pFirmware[nOffset++];

            if (nOffset + nLength > m_nFirmwareSize)
            {
                LOGERR ("Corrupt BT firmware file");
                return FALSE;
            }

            TBTHCIBcmVendorCommand VendorCmd;
            VendorCmd.Header.OpCode = nOpCode;
            VendorCmd.Header.ParameterTotalLength = nLength;
            memcpy (VendorCmd.Data, &m_pFirmware[nOffset], nLength);
            nOffset += nLength;

            SendCommand (&VendorCmd, sizeof VendorCmd.Header + nLength);

            if (!WaitForCommandComplete (nOpCode, 1000))
            {
                return FALSE;
            }

            if (nOpCode == OP_CODE_LAUNCH_RAM)
            {
                break;
            }
        }

        LOGNOTE ("BT firmware download complete");

        // the controller reboots with the patch applied
        CScheduler::Get ()->MsSleep (500);

        delete [] m_pFirmware;
        m_pFirmware = 0;
        m_nFirmwareSize = 0;

        // The rebooted patched firmware answers, is powered and ready
        // (its RTS is asserted), yet stays silent at 115200 - it has
        // switched its UART to a different baud rate. Probe the common
        // operational rates until the reset reply appears, then keep that
        // baud. Our UART clock is 48 MHz, so up to 3 MHz is reachable.
        static const unsigned CandidateBauds[] =
            {115200, 3000000, 921600, 2000000, 1500000, 460800, 230400};

        boolean bPostResetOK = FALSE;
        for (unsigned nBaudIdx = 0;
             nBaudIdx < sizeof CandidateBauds / sizeof CandidateBauds[0]
            && !bPostResetOK;
             nBaudIdx++)
        {
            unsigned nBaud = CandidateBauds[nBaudIdx];
            m_pTransport->SetBaud (nBaud);
            CScheduler::Get ()->MsSleep (20);

            for (unsigned nTry = 1; nTry <= 2 && !bPostResetOK; nTry++)
            {
                m_pTransport->FlushRx ();

                TBTHCICommandHeader Reset;
                Reset.OpCode = OP_CODE_RESET;
                Reset.ParameterTotalLength = 0;
                m_pTransport->SendHCICommand (&Reset, sizeof Reset);

                u8 Raw[48];
                unsigned nRaw = 0;
                // busy-poll ~300 ms without yielding, so a reply
                // cannot overflow the FIFO during a task switch
                for (unsigned t = 0; t < 300000 / 250 && nRaw < sizeof Raw; t++)
                {
                    int nByte = m_pTransport->ReadRawByte ();
                    if (nByte >= 0)
                    {
                        Raw[nRaw++] = (u8) nByte;
                    }
                    else
                    {
                        CTimer::SimpleusDelay (250);
                    }
                }

                if (nRaw > 0)
                {
                    char Line[3 * 48 + 1];
                    for (unsigned i = 0; i < nRaw; i++)
                    {
                        snprintf (&Line[i * 3], 4, "%02X ",
                              (unsigned) Raw[i]);
                    }
                    LOGNOTE ("baud %u try %u: %u bytes: %s",
                         nBaud, nTry, nRaw, Line);

                    for (unsigned i = 0; i + 2 < nRaw; i++)
                    {
                        if (Raw[i] == 0x03 && Raw[i + 1] == 0x0C
                            && Raw[i + 2] == 0x00)
                        {
                            bPostResetOK = TRUE;
                            break;
                        }
                    }
                }
            }

            if (bPostResetOK)
            {
                LOGNOTE ("Controller UART baud is %u after firmware", nBaud);
            }
        }
        if (!bPostResetOK)
        {
            LOGERR ("Controller did not come back after firmware load");
            return FALSE;
        }

        m_pTransport->FlushRx ();
        LOGNOTE ("Controller running patched firmware");
    }

    // enable LE meta events (bit 61 of the event mask)
    TBTHCISetEventMaskCommand MaskCmd;
    MaskCmd.Header.OpCode = OP_CODE_SET_EVENT_MASK;
    MaskCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (MaskCmd);
    static const u8 EventMask[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x20};
    memcpy (MaskCmd.EventMask, EventMask, sizeof MaskCmd.EventMask);
    SendCommand (&MaskCmd, sizeof MaskCmd);
    if (!WaitForCommandComplete (OP_CODE_SET_EVENT_MASK, 1000))
    {
        return FALSE;
    }

    // write LE host support
    TBTHCIWriteLEHostSupportCommand LEHostCmd;
    LEHostCmd.Header.OpCode = OP_CODE_WRITE_LE_HOST_SUPPORT;
    LEHostCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (LEHostCmd);
    LEHostCmd.LESupportedHost = 1;
    LEHostCmd.SimultaneousLEHost = 0;
    SendCommand (&LEHostCmd, sizeof LEHostCmd);
    WaitForCommandComplete (OP_CODE_WRITE_LE_HOST_SUPPORT, 500);    // optional

    // read local address (for logging)
    Cmd.OpCode = OP_CODE_READ_BD_ADDR;
    Cmd.ParameterTotalLength = 0;
    SendCommand (&Cmd, sizeof Cmd);
    if (WaitForCommandComplete (OP_CODE_READ_BD_ADDR, 1000,
                    m_LocalBDAddr, sizeof m_LocalBDAddr))
    {
        LOGNOTE ("BD address is %02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned) m_LocalBDAddr[5], (unsigned) m_LocalBDAddr[4],
             (unsigned) m_LocalBDAddr[3], (unsigned) m_LocalBDAddr[2],
             (unsigned) m_LocalBDAddr[1], (unsigned) m_LocalBDAddr[0]);
    }

    // read LE buffer characteristics
    u8 BufferInfo[3];
    Cmd.OpCode = OP_CODE_LE_READ_BUFFER_SIZE;
    Cmd.ParameterTotalLength = 0;
    SendCommand (&Cmd, sizeof Cmd);
    if (WaitForCommandComplete (OP_CODE_LE_READ_BUFFER_SIZE, 1000,
                    BufferInfo, sizeof BufferInfo))
    {
        unsigned nSize = BufferInfo[0] | (BufferInfo[1] << 8);
        unsigned nCount = BufferInfo[2];
        if (nSize >= 27 && nCount > 0)
        {
            m_nLEBufferSize = nSize;
            m_nLEBufferCount = nCount;
        }
    }

    LOGNOTE ("LE buffers: %u x %u bytes", m_nLEBufferCount, m_nLEBufferSize);

    return TRUE;
}

boolean CBLEService::StartAdvertising (void)
{
    // advertising parameters
    TBTHCILESetAdvParamsCommand ParamsCmd;
    memset (&ParamsCmd, 0, sizeof ParamsCmd);
    ParamsCmd.Header.OpCode = OP_CODE_LE_SET_ADV_PARAMS;
    ParamsCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (ParamsCmd);
    ParamsCmd.AdvIntervalMin = 0x00A0;              // 100ms
    ParamsCmd.AdvIntervalMax = 0x00F0;              // 150ms
    ParamsCmd.AdvType = ADV_TYPE_ADV_IND;
    ParamsCmd.OwnAddressType = ADDR_TYPE_PUBLIC;
    ParamsCmd.AdvChannelMap = 0x07;
    ParamsCmd.AdvFilterPolicy = 0x00;
    SendCommand (&ParamsCmd, sizeof ParamsCmd);
    if (!WaitForCommandComplete (OP_CODE_LE_SET_ADV_PARAMS, 1000))
    {
        return FALSE;
    }

    // advertising data: flags + NUS service UUID
    TBTHCILESetAdvDataCommand DataCmd;
    memset (&DataCmd, 0, sizeof DataCmd);
    DataCmd.Header.OpCode = OP_CODE_LE_SET_ADV_DATA;
    DataCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (DataCmd);

    unsigned i = 0;
    DataCmd.AdvData[i++] = 2;                       // flags
    DataCmd.AdvData[i++] = 0x01;
    DataCmd.AdvData[i++] = 0x06;                    // LE general discoverable, no BR/EDR
    DataCmd.AdvData[i++] = 17;                      // complete list of 128-bit UUIDs
    DataCmd.AdvData[i++] = 0x07;
    memcpy (&DataCmd.AdvData[i], UUID_NUS_SERVICE, 16);
    i += 16;
    unsigned nNameLen = strlen (BLE_DEVICE_NAME);
    DataCmd.AdvData[i++] = nNameLen + 1;            // complete local name
    DataCmd.AdvData[i++] = 0x09;
    memcpy (&DataCmd.AdvData[i], BLE_DEVICE_NAME, nNameLen);
    i += nNameLen;
    DataCmd.AdvDataLength = i;

    SendCommand (&DataCmd, sizeof DataCmd);
    if (!WaitForCommandComplete (OP_CODE_LE_SET_ADV_DATA, 1000))
    {
        return FALSE;
    }

    // scan response data: complete local name
    TBTHCILESetAdvDataCommand ScanRspCmd;
    memset (&ScanRspCmd, 0, sizeof ScanRspCmd);
    ScanRspCmd.Header.OpCode = OP_CODE_LE_SET_SCAN_RSP_DATA;
    ScanRspCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (ScanRspCmd);
    i = 0;
    ScanRspCmd.AdvData[i++] = nNameLen + 1;
    ScanRspCmd.AdvData[i++] = 0x09;
    memcpy (&ScanRspCmd.AdvData[i], BLE_DEVICE_NAME, nNameLen);
    i += nNameLen;
    ScanRspCmd.AdvDataLength = i;

    SendCommand (&ScanRspCmd, sizeof ScanRspCmd);
    if (!WaitForCommandComplete (OP_CODE_LE_SET_SCAN_RSP_DATA, 1000))
    {
        return FALSE;
    }

    // enable advertising
    TBTHCILESetAdvEnableCommand EnableCmd;
    EnableCmd.Header.OpCode = OP_CODE_LE_SET_ADV_ENABLE;
    EnableCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (EnableCmd);
    EnableCmd.AdvEnable = 1;
    SendCommand (&EnableCmd, sizeof EnableCmd);
    if (!WaitForCommandComplete (OP_CODE_LE_SET_ADV_ENABLE, 1000))
    {
        return FALSE;
    }

    LOGNOTE ("BLE advertising as \"%s\"", BLE_DEVICE_NAME);

    return TRUE;
}

//
// event processing
//

void CBLEService::ProcessEvent (const u8 *pBuffer, unsigned nLength)
{
    if (nLength < sizeof (TBTHCIEventHeader))
    {
        return;
    }

    const TBTHCIEventHeader *pHeader = (const TBTHCIEventHeader *) pBuffer;

    switch (pHeader->EventCode)
    {
    case EVENT_CODE_LE_META: {
        if (nLength < sizeof (TBTHCIEventLEMeta))
        {
            break;
        }
        const TBTHCIEventLEMeta *pMeta = (const TBTHCIEventLEMeta *) pBuffer;

        if (   pMeta->SubEventCode == LE_SUB_EVENT_CONNECTION_COMPLETE
            || pMeta->SubEventCode == LE_SUB_EVENT_ENHANCED_CONN_COMPLETE)
        {
            if (nLength >= sizeof (TBTHCIEventLEConnectionComplete))
            {
                const TBTHCIEventLEConnectionComplete *pConn =
                    (const TBTHCIEventLEConnectionComplete *) pBuffer;
                if (pConn->Status == BT_STATUS_SUCCESS)
                {
                    // clear any stale RX bytes left in the framing state
                    // machine from a previous connection, so the first
                    // packet of this one is not corrupted
                    m_pTransport->FlushRx ();

                    m_bConnected = TRUE;
                    m_nConnectionHandle = pConn->ConnectionHandle & 0x0FFF;
                    m_nATTMTU = 23;
                    m_bNotificationsEnabled = FALSE;
                    m_nL2CAPReceived = 0;
                    m_nL2CAPExpected = 0;
                    m_nCmdLength = 0;
                    m_nTxHead = m_nTxTail = 0;
                    m_nPacketsInFlight = 0;

                    LOGNOTE ("BLE central connected (handle 0x%03X)",
                         (unsigned) m_nConnectionHandle);
                }
            }
        }
        } break;

    case EVENT_CODE_DISCONNECTION_COMPLETE: {
        if (nLength >= sizeof (TBTHCIEventDisconnectionComplete))
        {
            const TBTHCIEventDisconnectionComplete *pDisc =
                (const TBTHCIEventDisconnectionComplete *) pBuffer;

            if ((pDisc->ConnectionHandle & 0x0FFF) == m_nConnectionHandle)
            {
                LOGNOTE ("BLE central disconnected (reason 0x%02X)",
                     (unsigned) pDisc->Reason);

                m_bConnected = FALSE;
                m_bNotificationsEnabled = FALSE;
                m_nPacketsInFlight = 0;

                // resume advertising
                TBTHCILESetAdvEnableCommand EnableCmd;
                EnableCmd.Header.OpCode = OP_CODE_LE_SET_ADV_ENABLE;
                EnableCmd.Header.ParameterTotalLength = PARM_TOTAL_LEN (EnableCmd);
                EnableCmd.AdvEnable = 1;
                SendCommand (&EnableCmd, sizeof EnableCmd);
            }
        }
        } break;

    case EVENT_CODE_NUM_COMPLETED_PACKETS: {
        if (nLength >= sizeof (TBTHCIEventNumCompletedPackets))
        {
            const TBTHCIEventNumCompletedPackets *pEvent =
                (const TBTHCIEventNumCompletedPackets *) pBuffer;

            const u8 *pData = pEvent->Data;
            for (unsigned i = 0; i < pEvent->NumHandles; i++)
            {
                u16 nCompleted =   pData[pEvent->NumHandles*2 + i*2]
                         | (pData[pEvent->NumHandles*2 + i*2 + 1] << 8);

                if (m_nPacketsInFlight >= nCompleted)
                {
                    m_nPacketsInFlight -= nCompleted;
                }
                else
                {
                    m_nPacketsInFlight = 0;
                }
            }
        }
        } break;

    default:
        break;
    }
}

//
// ACL / L2CAP / ATT
//

void CBLEService::ProcessACL (const u8 *pBuffer, unsigned nLength)
{
    if (nLength < sizeof (TBTHCIACLHeader))
    {
        return;
    }

    const TBTHCIACLHeader *pHeader = (const TBTHCIACLHeader *) pBuffer;
    unsigned nPBFlag = (pHeader->Handle >> 12) & 0x03;
    const u8 *pData = pBuffer + sizeof (TBTHCIACLHeader);
    unsigned nDataLength = nLength - sizeof (TBTHCIACLHeader);

    if (nPBFlag == 0x02 || nPBFlag == 0x00)         // first fragment
    {
        if (nDataLength < sizeof (TBTL2CAPHeader))
        {
            return;
        }

        const TBTL2CAPHeader *pL2CAP = (const TBTL2CAPHeader *) pData;
        unsigned nPDULength = pL2CAP->Length + sizeof (TBTL2CAPHeader);
        if (nPDULength > BLE_MAX_L2CAP_PDU)
        {
            m_nL2CAPExpected = 0;
            return;
        }

        memcpy (m_L2CAPBuffer, pData, nDataLength);
        m_nL2CAPReceived = nDataLength;
        m_nL2CAPExpected = nPDULength;
    }
    else if (nPBFlag == 0x01)                       // continuing fragment
    {
        if (   m_nL2CAPExpected == 0
            || m_nL2CAPReceived + nDataLength > BLE_MAX_L2CAP_PDU)
        {
            m_nL2CAPExpected = 0;
            return;
        }

        memcpy (&m_L2CAPBuffer[m_nL2CAPReceived], pData, nDataLength);
        m_nL2CAPReceived += nDataLength;
    }
    else
    {
        return;
    }

    if (m_nL2CAPExpected != 0 && m_nL2CAPReceived >= m_nL2CAPExpected)
    {
        const TBTL2CAPHeader *pL2CAP = (const TBTL2CAPHeader *) m_L2CAPBuffer;
        ProcessL2CAP (pL2CAP->CID,
                  m_L2CAPBuffer + sizeof (TBTL2CAPHeader),
                  pL2CAP->Length);

        m_nL2CAPExpected = 0;
        m_nL2CAPReceived = 0;
    }
}

void CBLEService::ProcessL2CAP (u16 nCID, const u8 *pPDU, unsigned nLength)
{
    switch (nCID)
    {
    case L2CAP_CID_ATT:
        ProcessATT (pPDU, nLength);
        break;

    case L2CAP_CID_SMP:
        if (nLength >= 1 && pPDU[0] == SMP_PAIRING_REQUEST)
        {
            u8 Rsp[2] = {SMP_PAIRING_FAILED, SMP_REASON_PAIRING_NOT_SUPPORTED};
            SendL2CAP (L2CAP_CID_SMP, Rsp, sizeof Rsp);
        }
        break;

    case L2CAP_CID_LE_SIGNALING:
        if (nLength >= 4)
        {
            u8 nCode = pPDU[0];
            u8 nId = pPDU[1];

            // respond with command reject to any request
            if (nCode == 0x12 || nCode == 0x14)     // conn param update / le credit req
            {
                u8 Rsp[6] = {0x01, nId, 0x02, 0x00, 0x00, 0x00};
                SendL2CAP (L2CAP_CID_LE_SIGNALING, Rsp, sizeof Rsp);
            }
        }
        break;

    default:
        break;
    }
}

void CBLEService::SendL2CAP (u16 nCID, const u8 *pPDU, unsigned nLength)
{
    if (!m_bConnected)
    {
        return;
    }

    // wait for a free controller buffer
    unsigned nElapsed = 0;
    while (m_nPacketsInFlight >= m_nLEBufferCount && nElapsed < 1000)
    {
        m_pTransport->Poll ();

        TPacket Packet;
        while (DequeuePacket (&m_EventQueue, &Packet))
        {
            ProcessEvent (Packet.Data, Packet.nLength);
        }

        if (m_nPacketsInFlight < m_nLEBufferCount)
        {
            break;
        }

        CScheduler::Get ()->MsSleep (5);
        nElapsed += 5;
    }

    u8 Buffer[sizeof (TBTHCIACLHeader) + sizeof (TBTL2CAPHeader) + BLE_MAX_L2CAP_PDU];

    unsigned nTotal = sizeof (TBTL2CAPHeader) + nLength;
    if (nTotal > m_nLEBufferSize)
    {
        LOGWARN ("L2CAP PDU too large: %u", nTotal);
        return;
    }

    TBTHCIACLHeader *pACL = (TBTHCIACLHeader *) Buffer;
    pACL->Handle = m_nConnectionHandle;             // PB flag 00, BC flag 00
    pACL->DataLength = nTotal;

    TBTL2CAPHeader *pL2CAP = (TBTL2CAPHeader *) (Buffer + sizeof (TBTHCIACLHeader));
    pL2CAP->Length = nLength;
    pL2CAP->CID = nCID;

    memcpy (Buffer + sizeof (TBTHCIACLHeader) + sizeof (TBTL2CAPHeader), pPDU, nLength);

    m_nPacketsInFlight++;
    m_pTransport->SendHCIACLData (Buffer, sizeof (TBTHCIACLHeader) + nTotal);
}

void CBLEService::SendATTErrorResponse (u8 nReqOpcode, u16 nHandle, u8 nErrorCode)
{
    u8 Rsp[5];
    Rsp[0] = ATT_OP_ERROR_RSP;
    Rsp[1] = nReqOpcode;
    Rsp[2] = nHandle & 0xFF;
    Rsp[3] = nHandle >> 8;
    Rsp[4] = nErrorCode;
    SendL2CAP (L2CAP_CID_ATT, Rsp, sizeof Rsp);
}

unsigned CBLEService::GetEffectiveMTU (void) const
{
    unsigned nMTU = m_nATTMTU;

    // an ATT PDU must fit into one controller ACL buffer
    if (nMTU > m_nLEBufferSize - sizeof (TBTL2CAPHeader))
    {
        nMTU = m_nLEBufferSize - sizeof (TBTL2CAPHeader);
    }

    return nMTU;
}

void CBLEService::ProcessATT (const u8 *pPDU, unsigned nLength)
{
    if (nLength < 1)
    {
        return;
    }

    u8 nOpcode = pPDU[0];
    u8 Rsp[BLE_ATT_SERVER_MTU];

    switch (nOpcode)
    {
    case ATT_OP_MTU_REQ: {
        if (nLength < 3)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        unsigned nClientMTU = pPDU[1] | (pPDU[2] << 8);

        Rsp[0] = ATT_OP_MTU_RSP;
        Rsp[1] = BLE_ATT_SERVER_MTU & 0xFF;
        Rsp[2] = BLE_ATT_SERVER_MTU >> 8;
        SendL2CAP (L2CAP_CID_ATT, Rsp, 3);

        unsigned nMTU = nClientMTU < BLE_ATT_SERVER_MTU ? nClientMTU : BLE_ATT_SERVER_MTU;
        if (nMTU < 23)
        {
            nMTU = 23;
        }
        m_nATTMTU = nMTU;
        } break;

    case ATT_OP_READ_BY_GROUP_TYPE_REQ: {
        if (nLength < 7)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        u16 nStart = pPDU[1] | (pPDU[2] << 8);
        u16 nEnd   = pPDU[3] | (pPDU[4] << 8);
        u16 nType  = pPDU[5] | (pPDU[6] << 8);

        if (nType != UUID_PRIMARY_SERVICE || nLength != 7)
        {
            SendATTErrorResponse (nOpcode, nStart, ATT_ERR_ATTRIBUTE_NOT_FOUND);
            break;
        }

        if (nStart <= HANDLE_GAP_SERVICE && nEnd >= HANDLE_GAP_SERVICE)
        {
            Rsp[0] = ATT_OP_READ_BY_GROUP_TYPE_RSP;
            Rsp[1] = 6;                     // entry length
            Rsp[2] = HANDLE_GAP_SERVICE & 0xFF;
            Rsp[3] = HANDLE_GAP_SERVICE >> 8;
            Rsp[4] = HANDLE_GAP_END & 0xFF;
            Rsp[5] = HANDLE_GAP_END >> 8;
            Rsp[6] = UUID_GAP_SERVICE & 0xFF;
            Rsp[7] = UUID_GAP_SERVICE >> 8;
            SendL2CAP (L2CAP_CID_ATT, Rsp, 8);
        }
        else if (nStart <= HANDLE_NUS_SERVICE && nEnd >= HANDLE_NUS_SERVICE)
        {
            Rsp[0] = ATT_OP_READ_BY_GROUP_TYPE_RSP;
            Rsp[1] = 20;                    // entry length
            Rsp[2] = HANDLE_NUS_SERVICE & 0xFF;
            Rsp[3] = HANDLE_NUS_SERVICE >> 8;
            Rsp[4] = HANDLE_NUS_END & 0xFF;
            Rsp[5] = HANDLE_NUS_END >> 8;
            memcpy (&Rsp[6], UUID_NUS_SERVICE, 16);
            SendL2CAP (L2CAP_CID_ATT, Rsp, 22);
        }
        else
        {
            SendATTErrorResponse (nOpcode, nStart, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        } break;

    case ATT_OP_FIND_BY_TYPE_VALUE_REQ: {
        if (nLength < 7)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        u16 nStart = pPDU[1] | (pPDU[2] << 8);
        u16 nType  = pPDU[5] | (pPDU[6] << 8);
        const u8 *pValue = &pPDU[7];
        unsigned nValueLength = nLength - 7;

        u16 nFoundStart = 0;
        u16 nFoundEnd = 0;

        if (nType == UUID_PRIMARY_SERVICE)
        {
            if (   nValueLength == 2
                && (pValue[0] | (pValue[1] << 8)) == UUID_GAP_SERVICE
                && nStart <= HANDLE_GAP_SERVICE)
            {
                nFoundStart = HANDLE_GAP_SERVICE;
                nFoundEnd = HANDLE_GAP_END;
            }
            else if (   nValueLength == 16
                 && memcmp (pValue, UUID_NUS_SERVICE, 16) == 0
                 && nStart <= HANDLE_NUS_SERVICE)
            {
                nFoundStart = HANDLE_NUS_SERVICE;
                nFoundEnd = HANDLE_NUS_END;
            }
        }

        if (nFoundStart != 0)
        {
            Rsp[0] = ATT_OP_FIND_BY_TYPE_VALUE_RSP;
            Rsp[1] = nFoundStart & 0xFF;
            Rsp[2] = nFoundStart >> 8;
            Rsp[3] = nFoundEnd & 0xFF;
            Rsp[4] = nFoundEnd >> 8;
            SendL2CAP (L2CAP_CID_ATT, Rsp, 5);
        }
        else
        {
            SendATTErrorResponse (nOpcode, nStart, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        } break;

    case ATT_OP_READ_BY_TYPE_REQ: {
        if (nLength < 7)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        u16 nStart = pPDU[1] | (pPDU[2] << 8);
        u16 nEnd   = pPDU[3] | (pPDU[4] << 8);

        if (nLength == 7)
        {
            u16 nType = pPDU[5] | (pPDU[6] << 8);

            if (nType == UUID_CHARACTERISTIC)
            {
                // characteristic declarations in ascending handle order
                static const struct
                {
                    u16 nHandle;
                    u8 nProperties;
                    u16 nValueHandle;
                    const u8 *pUUID128;     // 0 for 16-bit
                    u16 nUUID16;
                }
                Decls[] =
                {
                    {HANDLE_NAME_DECL,       0x02, HANDLE_NAME_VALUE,       0, UUID_DEVICE_NAME},
                    {HANDLE_APPEARANCE_DECL, 0x02, HANDLE_APPEARANCE_VALUE, 0, UUID_APPEARANCE},
                    {HANDLE_NUS_RX_DECL,     0x0C, HANDLE_NUS_RX_VALUE,     UUID_NUS_RX, 0},
                    {HANDLE_NUS_TX_DECL,     0x10, HANDLE_NUS_TX_VALUE,     UUID_NUS_TX, 0},
                };

                boolean bSent = FALSE;
                for (unsigned i = 0; i < sizeof Decls / sizeof Decls[0]; i++)
                {
                    if (Decls[i].nHandle < nStart || Decls[i].nHandle > nEnd)
                    {
                        continue;
                    }

                    unsigned nUUIDLen = Decls[i].pUUID128 != 0 ? 16 : 2;
                    Rsp[0] = ATT_OP_READ_BY_TYPE_RSP;
                    Rsp[1] = 2 + 3 + nUUIDLen;      // entry length
                    Rsp[2] = Decls[i].nHandle & 0xFF;
                    Rsp[3] = Decls[i].nHandle >> 8;
                    Rsp[4] = Decls[i].nProperties;
                    Rsp[5] = Decls[i].nValueHandle & 0xFF;
                    Rsp[6] = Decls[i].nValueHandle >> 8;
                    if (Decls[i].pUUID128 != 0)
                    {
                        memcpy (&Rsp[7], Decls[i].pUUID128, 16);
                    }
                    else
                    {
                        Rsp[7] = Decls[i].nUUID16 & 0xFF;
                        Rsp[8] = Decls[i].nUUID16 >> 8;
                    }

                    SendL2CAP (L2CAP_CID_ATT, Rsp, 2 + Rsp[1]);
                    bSent = TRUE;
                    break;          // one entry per response
                }

                if (!bSent)
                {
                    SendATTErrorResponse (nOpcode, nStart,
                                  ATT_ERR_ATTRIBUTE_NOT_FOUND);
                }
                break;
            }
            else if (nType == UUID_DEVICE_NAME
                 && nStart <= HANDLE_NAME_VALUE && nEnd >= HANDLE_NAME_VALUE)
            {
                unsigned nNameLen = strlen (BLE_DEVICE_NAME);
                Rsp[0] = ATT_OP_READ_BY_TYPE_RSP;
                Rsp[1] = 2 + nNameLen;
                Rsp[2] = HANDLE_NAME_VALUE & 0xFF;
                Rsp[3] = HANDLE_NAME_VALUE >> 8;
                memcpy (&Rsp[4], BLE_DEVICE_NAME, nNameLen);
                SendL2CAP (L2CAP_CID_ATT, Rsp, 4 + nNameLen);
                break;
            }
        }

        SendATTErrorResponse (nOpcode, nStart, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        } break;

    case ATT_OP_FIND_INFO_REQ: {
        if (nLength < 5)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        u16 nStart = pPDU[1] | (pPDU[2] << 8);
        u16 nEnd   = pPDU[3] | (pPDU[4] << 8);

        static const struct
        {
            u16 nHandle;
            const u8 *pUUID128;     // 0 for 16-bit
            u16 nUUID16;
        }
        Attributes[] =
        {
            {HANDLE_GAP_SERVICE,      0, UUID_PRIMARY_SERVICE},
            {HANDLE_NAME_DECL,        0, UUID_CHARACTERISTIC},
            {HANDLE_NAME_VALUE,       0, UUID_DEVICE_NAME},
            {HANDLE_APPEARANCE_DECL,  0, UUID_CHARACTERISTIC},
            {HANDLE_APPEARANCE_VALUE, 0, UUID_APPEARANCE},
            {HANDLE_NUS_SERVICE,      0, UUID_PRIMARY_SERVICE},
            {HANDLE_NUS_RX_DECL,      0, UUID_CHARACTERISTIC},
            {HANDLE_NUS_RX_VALUE,     UUID_NUS_RX, 0},
            {HANDLE_NUS_TX_DECL,      0, UUID_CHARACTERISTIC},
            {HANDLE_NUS_TX_VALUE,     UUID_NUS_TX, 0},
            {HANDLE_NUS_TX_CCCD,      0, UUID_CCCD},
        };

        boolean bSent = FALSE;
        for (unsigned i = 0; i < sizeof Attributes / sizeof Attributes[0]; i++)
        {
            if (Attributes[i].nHandle < nStart || Attributes[i].nHandle > nEnd)
            {
                continue;
            }

            if (Attributes[i].pUUID128 != 0)
            {
                Rsp[0] = ATT_OP_FIND_INFO_RSP;
                Rsp[1] = 2;             // format: 128-bit UUIDs
                Rsp[2] = Attributes[i].nHandle & 0xFF;
                Rsp[3] = Attributes[i].nHandle >> 8;
                memcpy (&Rsp[4], Attributes[i].pUUID128, 16);
                SendL2CAP (L2CAP_CID_ATT, Rsp, 20);
            }
            else
            {
                Rsp[0] = ATT_OP_FIND_INFO_RSP;
                Rsp[1] = 1;             // format: 16-bit UUIDs
                Rsp[2] = Attributes[i].nHandle & 0xFF;
                Rsp[3] = Attributes[i].nHandle >> 8;
                Rsp[4] = Attributes[i].nUUID16 & 0xFF;
                Rsp[5] = Attributes[i].nUUID16 >> 8;
                SendL2CAP (L2CAP_CID_ATT, Rsp, 6);
            }

            bSent = TRUE;
            break;                  // one entry per response
        }

        if (!bSent)
        {
            SendATTErrorResponse (nOpcode, nStart, ATT_ERR_ATTRIBUTE_NOT_FOUND);
        }
        } break;

    case ATT_OP_READ_REQ: {
        if (nLength < 3)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            break;
        }

        u16 nHandle = pPDU[1] | (pPDU[2] << 8);

        switch (nHandle)
        {
        case HANDLE_NAME_VALUE: {
            unsigned nNameLen = strlen (BLE_DEVICE_NAME);
            Rsp[0] = ATT_OP_READ_RSP;
            memcpy (&Rsp[1], BLE_DEVICE_NAME, nNameLen);
            SendL2CAP (L2CAP_CID_ATT, Rsp, 1 + nNameLen);
            } break;

        case HANDLE_APPEARANCE_VALUE:
            Rsp[0] = ATT_OP_READ_RSP;
            Rsp[1] = 0x00;
            Rsp[2] = 0x00;
            SendL2CAP (L2CAP_CID_ATT, Rsp, 3);
            break;

        case HANDLE_NUS_TX_CCCD:
            Rsp[0] = ATT_OP_READ_RSP;
            Rsp[1] = m_bNotificationsEnabled ? 0x01 : 0x00;
            Rsp[2] = 0x00;
            SendL2CAP (L2CAP_CID_ATT, Rsp, 3);
            break;

        case HANDLE_NUS_TX_VALUE:
            Rsp[0] = ATT_OP_READ_RSP;
            SendL2CAP (L2CAP_CID_ATT, Rsp, 1);
            break;

        case HANDLE_NUS_RX_VALUE:
            SendATTErrorResponse (nOpcode, nHandle, ATT_ERR_READ_NOT_PERMITTED);
            break;

        default:
            SendATTErrorResponse (nOpcode, nHandle, ATT_ERR_INVALID_HANDLE);
            break;
        }
        } break;

    case ATT_OP_WRITE_REQ:
    case ATT_OP_WRITE_CMD: {
        if (nLength < 3)
        {
            if (nOpcode == ATT_OP_WRITE_REQ)
            {
                SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
            }
            break;
        }

        u16 nHandle = pPDU[1] | (pPDU[2] << 8);
        const u8 *pValue = &pPDU[3];
        unsigned nValueLength = nLength - 3;

        boolean bOK = FALSE;

        switch (nHandle)
        {
        case HANDLE_NUS_RX_VALUE:
            OnRxData (pValue, nValueLength);
            bOK = TRUE;
            break;

        case HANDLE_NUS_TX_CCCD:
            if (nValueLength >= 1)
            {
                m_bNotificationsEnabled = (pValue[0] & 0x01) ? TRUE : FALSE;
                LOGNOTE ("Notifications %s",
                     m_bNotificationsEnabled ? "enabled" : "disabled");
                bOK = TRUE;
            }
            break;

        default:
            break;
        }

        if (nOpcode == ATT_OP_WRITE_REQ)
        {
            if (bOK)
            {
                Rsp[0] = ATT_OP_WRITE_RSP;
                SendL2CAP (L2CAP_CID_ATT, Rsp, 1);
            }
            else
            {
                SendATTErrorResponse (nOpcode, nHandle,
                              ATT_ERR_WRITE_NOT_PERMITTED);
            }
        }
        } break;

    default:
        // only requests (even opcodes without the command flag) need an error response
        if ((nOpcode & 0x40) == 0 && nOpcode != ATT_OP_HANDLE_VALUE_NOTIFY)
        {
            SendATTErrorResponse (nOpcode, 0, ATT_ERR_REQUEST_NOT_SUPPORTED);
        }
        break;
    }
}

//
// NUS output stream
//

void CBLEService::QueueOutput (const char *pString)
{
    while (*pString != '\0')
    {
        unsigned nNext = (m_nTxHead + 1) % BLE_TX_STREAM_SIZE;
        if (nNext == m_nTxTail)
        {
            break;          // stream full, drop the rest
        }

        m_TxStream[m_nTxHead] = (u8) *pString++;
        m_nTxHead = nNext;
    }
}

void CBLEService::FlushOutput (void)
{
    if (!m_bConnected || !m_bNotificationsEnabled)
    {
        return;
    }

    unsigned nChunkSize = GetEffectiveMTU () - 3;
    u8 Notify[3 + BLE_ATT_SERVER_MTU];

    while (m_nTxTail != m_nTxHead)
    {
        unsigned nCount = 0;

        Notify[0] = ATT_OP_HANDLE_VALUE_NOTIFY;
        Notify[1] = HANDLE_NUS_TX_VALUE & 0xFF;
        Notify[2] = HANDLE_NUS_TX_VALUE >> 8;

        while (m_nTxTail != m_nTxHead && nCount < nChunkSize)
        {
            Notify[3 + nCount++] = m_TxStream[m_nTxTail];
            m_nTxTail = (m_nTxTail + 1) % BLE_TX_STREAM_SIZE;
        }

        SendL2CAP (L2CAP_CID_ATT, Notify, 3 + nCount);
    }
}

//
// command shell
//

void CBLEService::OnRxData (const u8 *pData, unsigned nLength)
{
    for (unsigned i = 0; i < nLength; i++)
    {
        char c = (char) pData[i];

        if (c == '\r' || c == '\n')
        {
            if (m_nCmdLength > 0)
            {
                m_CmdBuffer[m_nCmdLength] = '\0';
                ExecuteCommand (m_CmdBuffer);
                m_nCmdLength = 0;
            }
        }
        else if (m_nCmdLength < BLE_CMD_BUFFER_SIZE - 1)
        {
            m_CmdBuffer[m_nCmdLength++] = c;
        }
    }
}

void CBLEService::ExecuteCommand (char *pCommand)
{
    // trim leading spaces
    while (*pCommand == ' ')
    {
        pCommand++;
    }

    // split command and argument
    char *pArg = strchr (pCommand, ' ');
    if (pArg != 0)
    {
        *pArg++ = '\0';
        while (*pArg == ' ')
        {
            pArg++;
        }
    }

    LOGNOTE ("BLE command: %s %s", pCommand, pArg != 0 ? pArg : "");

    if (strcasecmp (pCommand, "help") == 0 || strcmp (pCommand, "?") == 0)
    {
        CommandHelp ();
    }
    else if (strcasecmp (pCommand, "info") == 0)
    {
        CommandInfo ();
    }
    else if (strcasecmp (pCommand, "list") == 0)
    {
        unsigned nPage = 1;
        if (pArg != 0 && pArg[0] != '\0')
        {
            nPage = (unsigned) atoi (pArg);
            if (nPage < 1)
            {
                nPage = 1;
            }
        }
        CommandList (nPage);
    }
    else if (strcasecmp (pCommand, "mount") == 0)
    {
        if (pArg == 0 || pArg[0] == '\0')
        {
            QueueOutput ("ERR usage: mount <index|filename>\r\n");
        }
        else
        {
            CommandMount (pArg);
        }
    }
    else if (strcasecmp (pCommand, "reboot") == 0)
    {
        QueueOutput ("OK rebooting\r\n");
        FlushOutput ();
        DeviceState::Get ().setShutdownMode (ShutdownReboot);
    }
    else if (strcasecmp (pCommand, "shutdown") == 0)
    {
        QueueOutput ("OK shutting down\r\n");
        FlushOutput ();
        DeviceState::Get ().setShutdownMode (ShutdownHalt);
    }
    else
    {
        QueueOutput ("ERR unknown command (try: help)\r\n");
    }
}

void CBLEService::CommandHelp (void)
{
    QueueOutput ("USBODE BLE console\r\n"
             "  help          - this help\r\n"
             "  info          - show current image and version\r\n"
             "  list [page]   - list disc images (15 per page)\r\n"
             "  mount <n>     - mount image by index from 'list'\r\n"
             "  mount <file>  - mount image by (relative) file name\r\n"
             "  reboot        - reboot USBODE\r\n"
             "  shutdown      - shut down USBODE\r\n"
             "OK\r\n");
}

void CBLEService::CommandInfo (void)
{
    SCSITBService *pSCSITB =
        static_cast<SCSITBService *> (CScheduler::Get ()->GetTask ("scsitbservice"));

    char Line[MAX_PATH_LEN + 64];

    snprintf (Line, sizeof Line, "USBODE %s\r\n",
          CGitInfo::Get ()->GetVersionWithBuildString ());
    QueueOutput (Line);

    if (pSCSITB != 0)
    {
        snprintf (Line, sizeof Line, "current: %s\r\n", pSCSITB->GetCurrentCDPath ());
        QueueOutput (Line);

        snprintf (Line, sizeof Line, "images: %u\r\n", (unsigned) pSCSITB->GetCount ());
        QueueOutput (Line);
    }

    QueueOutput ("OK\r\n");
}

#define LIST_PAGE_SIZE  15

void CBLEService::CommandList (unsigned nPage)
{
    SCSITBService *pSCSITB =
        static_cast<SCSITBService *> (CScheduler::Get ()->GetTask ("scsitbservice"));
    if (pSCSITB == 0)
    {
        QueueOutput ("ERR image service not available\r\n");
        return;
    }

    size_t nCount = pSCSITB->GetCount ();

    // count files (directories are not mountable)
    unsigned nFiles = 0;
    for (size_t i = 0; i < nCount; i++)
    {
        if (!pSCSITB->IsDirectory (i))
        {
            nFiles++;
        }
    }

    unsigned nPages = (nFiles + LIST_PAGE_SIZE - 1) / LIST_PAGE_SIZE;
    if (nPages == 0)
    {
        nPages = 1;
    }
    if (nPage > nPages)
    {
        nPage = nPages;
    }

    unsigned nSkip = (nPage - 1) * LIST_PAGE_SIZE;
    unsigned nShown = 0;
    char Line[MAX_PATH_LEN + 32];

    for (size_t i = 0; i < nCount && nShown < LIST_PAGE_SIZE; i++)
    {
        if (pSCSITB->IsDirectory (i))
        {
            continue;
        }

        if (nSkip > 0)
        {
            nSkip--;
            continue;
        }

        const char *pMark = "";
        const FileEntry *pEntry = pSCSITB->GetFileEntry (i);
        if (pEntry != 0 && pSCSITB->GetCurrentCD () == i)
        {
            pMark = " *";
        }

        snprintf (Line, sizeof Line, "%u: %s%s\r\n",
              (unsigned) i, pEntry != 0 ? pEntry->relativePath : "?", pMark);
        QueueOutput (Line);
        FlushOutput ();         // keep the stream buffer small

        nShown++;
    }

    snprintf (Line, sizeof Line, "page %u/%u (%u images)\r\nOK\r\n",
          nPage, nPages, nFiles);
    QueueOutput (Line);
}

void CBLEService::CommandMount (const char *pArg)
{
    SCSITBService *pSCSITB =
        static_cast<SCSITBService *> (CScheduler::Get ()->GetTask ("scsitbservice"));
    if (pSCSITB == 0)
    {
        QueueOutput ("ERR image service not available\r\n");
        return;
    }

    // numeric argument -> index from 'list'
    boolean bNumeric = TRUE;
    for (const char *p = pArg; *p != '\0'; p++)
    {
        if (*p < '0' || *p > '9')
        {
            bNumeric = FALSE;
            break;
        }
    }

    const char *pFileName = pArg;

    if (bNumeric)
    {
        unsigned nIndex = (unsigned) atoi (pArg);
        if (nIndex >= pSCSITB->GetCount ())
        {
            QueueOutput ("ERR index out of range\r\n");
            return;
        }

        if (pSCSITB->IsDirectory (nIndex))
        {
            QueueOutput ("ERR cannot mount a directory\r\n");
            return;
        }

        pFileName = pSCSITB->GetRelativePath (nIndex);
    }

    if (pSCSITB->SetNextCDByName (pFileName))
    {
        char Line[MAX_PATH_LEN + 32];
        snprintf (Line, sizeof Line, "OK mounting %s\r\n", pFileName);
        QueueOutput (Line);
    }
    else
    {
        QueueOutput ("ERR image not found\r\n");
    }
}

//
// main task
//

void CBLEService::Run (void)
{
    LOGNOTE ("BLE service starting");

    if (!HasOnboardBluetooth ())
    {
        LOGWARN ("No onboard Bluetooth on this board - BLE service disabled");
        return;
    }

    // The patch firmware is chosen from the chip's reported name after
    // reset (see InitializeController), so nothing is loaded here.

    PowerOnController ();

    m_pTransport = new CBTUARTTransport (CInterruptSystem::Get ());
    assert (m_pTransport != 0);

    m_pTransport->RegisterHCIEventHandler (EventStub, this);
    m_pTransport->RegisterHCIACLHandler (ACLStub, this);

    if (!m_pTransport->Initialize ())
    {
        LOGERR ("Cannot initialize BT UART transport");
        return;
    }

    if (!m_pTransport->GetLoopbackOK ())
    {
        LOGWARN ("UART loopback self test failed");
        LogPinDiagnostics ();
    }

    if (!InitializeController ())
    {
        LOGERR ("Cannot initialize BT controller - BLE service disabled");
        return;
    }

    if (!StartAdvertising ())
    {
        LOGERR ("Cannot start BLE advertising - BLE service disabled");
        return;
    }

    m_bRunning = TRUE;

    while (1)
    {
        // Poll the UART frequently: the RX interrupt does not reach us, so
        // draining the 32-byte FIFO in time depends on this cadence. At
        // 115200 baud the FIFO fills in ~2.8 ms, so keep the period short.
        m_pTransport->Poll ();

        TPacket Packet;

        while (DequeuePacket (&m_EventQueue, &Packet))
        {
            ProcessEvent (Packet.Data, Packet.nLength);
        }

        while (DequeuePacket (&m_ACLQueue, &Packet))
        {
            ProcessACL (Packet.Data, Packet.nLength);
        }

        FlushOutput ();

        CScheduler::Get ()->MsSleep (2);
    }
}
