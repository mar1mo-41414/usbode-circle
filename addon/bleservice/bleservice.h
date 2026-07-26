//
// bleservice.h
//
// BLE serial (Nordic UART Service) console for USBODE.
// Allows listing and switching disc images from a phone or PC over
// Bluetooth LE, with no WiFi network required.
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
#ifndef _bleservice_bleservice_h
#define _bleservice_bleservice_h

#include <circle/sched/task.h>
#include <circle/types.h>
#include "btuarttransport.h"
#include "blehci.h"

#define BLE_RX_QUEUE_SIZE       8
#define BLE_MAX_L2CAP_PDU       512
#define BLE_ATT_SERVER_MTU      247
#define BLE_TX_STREAM_SIZE      8192
#define BLE_CMD_BUFFER_SIZE     512

class CBLEService : public CTask
{
public:
    CBLEService (void);
    ~CBLEService (void);

    void Run (void);

    static CBLEService *Get (void);

private:
    // packet queue filled from IRQ context
    struct TPacket
    {
        u16 nLength;
        u8 Data[BT_MAX_ACL_SIZE];
    };

    struct TPacketQueue
    {
        TPacket Packet[BLE_RX_QUEUE_SIZE];
        volatile unsigned nHead;
        volatile unsigned nTail;
    };

    static void EventStub (const void *pBuffer, unsigned nLength, void *pParam);
    static void ACLStub (const void *pBuffer, unsigned nLength, void *pParam);
    void EnqueuePacket (TPacketQueue *pQueue, const void *pBuffer, unsigned nLength);
    boolean DequeuePacket (TPacketQueue *pQueue, TPacket *pPacket);

    // HCI helpers
    boolean SendCommand (const void *pBuffer, unsigned nLength);
    boolean WaitForCommandComplete (u16 nOpCode, unsigned nTimeoutMs,
                    u8 *pReturn = 0, unsigned nReturnSize = 0);
    void ProcessEvent (const u8 *pBuffer, unsigned nLength);
    void ProcessACL (const u8 *pBuffer, unsigned nLength);

    void PowerOnController (void);
    void LogPinDiagnostics (void);
    boolean InitializeController (void);
    boolean LoadFirmware (const char *pFileName);
    boolean HasOnboardBluetooth (void);
    boolean StartAdvertising (void);

    // L2CAP / ATT
    void ProcessL2CAP (u16 nCID, const u8 *pPDU, unsigned nLength);
    void ProcessATT (const u8 *pPDU, unsigned nLength);
    void SendL2CAP (u16 nCID, const u8 *pPDU, unsigned nLength);
    void SendATTErrorResponse (u8 nReqOpcode, u16 nHandle, u8 nErrorCode);
    unsigned GetEffectiveMTU (void) const;

    // NUS output stream
    void QueueOutput (const char *pString);
    void FlushOutput (void);

    // command shell
    void OnRxData (const u8 *pData, unsigned nLength);
    void ExecuteCommand (char *pCommand);
    void CommandHelp (void);
    void CommandInfo (void);
    void CommandList (unsigned nPage);
    void CommandMount (const char *pArg);

private:
    static CBLEService *s_pThis;

    CBTUARTTransport *m_pTransport;

    TPacketQueue m_EventQueue;
    TPacketQueue m_ACLQueue;

    // controller state
    boolean m_bRunning;
    unsigned m_nCommandPackets;             // HCI command flow control
    u8 m_LocalBDAddr[BT_BD_ADDR_SIZE];
    unsigned m_nLEBufferSize;               // max ACL payload the controller accepts
    unsigned m_nLEBufferCount;              // number of controller ACL buffers
    volatile unsigned m_nPacketsInFlight;

    // connection state
    boolean m_bConnected;
    u16 m_nConnectionHandle;
    unsigned m_nATTMTU;                     // negotiated
    boolean m_bNotificationsEnabled;

    // L2CAP reassembly
    u8 m_L2CAPBuffer[BLE_MAX_L2CAP_PDU];
    unsigned m_nL2CAPReceived;
    unsigned m_nL2CAPExpected;

    // outgoing text stream (notifications)
    u8 m_TxStream[BLE_TX_STREAM_SIZE];
    unsigned m_nTxHead;
    unsigned m_nTxTail;

    // command line accumulator
    char m_CmdBuffer[BLE_CMD_BUFFER_SIZE];
    unsigned m_nCmdLength;

    // firmware blob
    u8 *m_pFirmware;
    unsigned m_nFirmwareSize;

    // BT_REG_ON on boards where it is a SoC GPIO
    class CGPIOPin *m_pBTPowerPin;

    // 32.768 kHz LPO clock to the BT controller (GPCLK2 on GPIO 43)
    class CGPIOPin *m_pLPOClockPin;
    class CGPIOClock *m_pLPOClock;
};

#endif
