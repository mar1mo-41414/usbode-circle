//
// blehci.h
//
// HCI / LE definitions for the USBODE BLE service.
//
// Partially derived from Circle's former Bluetooth support:
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2015-2016  R. Stange <rsta2@o2online.de>
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
#ifndef _bleservice_blehci_h
#define _bleservice_blehci_h

#include <circle/macros.h>
#include <circle/types.h>

// Sizes
#define BT_MAX_HCI_EVENT_SIZE   257
#define BT_MAX_HCI_COMMAND_SIZE 258
#define BT_MAX_ACL_SIZE         (255 + 4)
#define BT_BD_ADDR_SIZE         6

#define BT_STATUS_SUCCESS       0

// HCI packet types (UART transport)
#define HCI_PACKET_COMMAND      0x01
#define HCI_PACKET_ACL_DATA     0x02
#define HCI_PACKET_SYNCH_DATA   0x03
#define HCI_PACKET_EVENT        0x04

// Command header
struct TBTHCICommandHeader
{
    u16     OpCode;
    u8      ParameterTotalLength;
}
PACKED;

#define PARM_TOTAL_LEN(cmd)     (sizeof (cmd) - sizeof (TBTHCICommandHeader))

// Opcode groups
#define OGF_HCI_CONTROL_BASEBAND        (3 << 10)
    #define OP_CODE_RESET                   (OGF_HCI_CONTROL_BASEBAND | 0x003)
    #define OP_CODE_SET_EVENT_MASK          (OGF_HCI_CONTROL_BASEBAND | 0x001)
    #define OP_CODE_WRITE_LE_HOST_SUPPORT   (OGF_HCI_CONTROL_BASEBAND | 0x06D)
#define OGF_INFORMATIONAL_COMMANDS      (4 << 10)
    #define OP_CODE_READ_BD_ADDR            (OGF_INFORMATIONAL_COMMANDS | 0x009)
#define OGF_LE_CONTROL                  (8 << 10)
    #define OP_CODE_LE_SET_EVENT_MASK       (OGF_LE_CONTROL | 0x001)
    #define OP_CODE_LE_READ_BUFFER_SIZE     (OGF_LE_CONTROL | 0x002)
    #define OP_CODE_LE_SET_ADV_PARAMS       (OGF_LE_CONTROL | 0x006)
    #define OP_CODE_LE_SET_ADV_DATA         (OGF_LE_CONTROL | 0x008)
    #define OP_CODE_LE_SET_SCAN_RSP_DATA    (OGF_LE_CONTROL | 0x009)
    #define OP_CODE_LE_SET_ADV_ENABLE       (OGF_LE_CONTROL | 0x00A)
#define OGF_VENDOR_COMMANDS             (0x3F << 10)
    #define OP_CODE_DOWNLOAD_MINIDRIVER     (OGF_VENDOR_COMMANDS | 0x02E)
    #define OP_CODE_WRITE_RAM               (OGF_VENDOR_COMMANDS | 0x04C)
    #define OP_CODE_LAUNCH_RAM              (OGF_VENDOR_COMMANDS | 0x04E)

struct TBTHCIBcmVendorCommand
{
    TBTHCICommandHeader     Header;
    u8      Data[255];
}
PACKED;

struct TBTHCISetEventMaskCommand
{
    TBTHCICommandHeader     Header;
    u8      EventMask[8];
}
PACKED;

struct TBTHCIWriteLEHostSupportCommand
{
    TBTHCICommandHeader     Header;
    u8      LESupportedHost;
    u8      SimultaneousLEHost;
}
PACKED;

struct TBTHCILESetAdvParamsCommand
{
    TBTHCICommandHeader     Header;
    u16     AdvIntervalMin;         // * 0.625ms
    u16     AdvIntervalMax;
    u8      AdvType;
#define ADV_TYPE_ADV_IND                0x00
    u8      OwnAddressType;
#define ADDR_TYPE_PUBLIC                0x00
    u8      PeerAddressType;
    u8      PeerAddress[BT_BD_ADDR_SIZE];
    u8      AdvChannelMap;                  // 0x07 = all channels
    u8      AdvFilterPolicy;                // 0x00 = allow all
}
PACKED;

struct TBTHCILESetAdvDataCommand
{
    TBTHCICommandHeader     Header;
    u8      AdvDataLength;
    u8      AdvData[31];
}
PACKED;

struct TBTHCILESetAdvEnableCommand
{
    TBTHCICommandHeader     Header;
    u8      AdvEnable;
}
PACKED;

// Events
struct TBTHCIEventHeader
{
    u8      EventCode;
#define EVENT_CODE_DISCONNECTION_COMPLETE       0x05
#define EVENT_CODE_COMMAND_COMPLETE             0x0E
#define EVENT_CODE_COMMAND_STATUS               0x0F
#define EVENT_CODE_NUM_COMPLETED_PACKETS        0x13
#define EVENT_CODE_LE_META                      0x3E
    u8      ParameterTotalLength;
}
PACKED;

struct TBTHCIEventCommandComplete
{
    TBTHCIEventHeader       Header;
    u8      NumHCICommandPackets;
    u16     CommandOpCode;
    u8      Status;                         // first byte of ReturnParameter[]
    u8      ReturnParameter[0];
}
PACKED;

struct TBTHCIEventCommandStatus
{
    TBTHCIEventHeader       Header;
    u8      Status;
    u8      NumHCICommandPackets;
    u16     CommandOpCode;
}
PACKED;

struct TBTHCIEventDisconnectionComplete
{
    TBTHCIEventHeader       Header;
    u8      Status;
    u16     ConnectionHandle;
    u8      Reason;
}
PACKED;

struct TBTHCIEventNumCompletedPackets
{
    TBTHCIEventHeader       Header;
    u8      NumHandles;
    // u16 handle[NumHandles]; u16 completed[NumHandles];
    u8      Data[0];
}
PACKED;

struct TBTHCIEventLEMeta
{
    TBTHCIEventHeader       Header;
    u8      SubEventCode;
#define LE_SUB_EVENT_CONNECTION_COMPLETE        0x01
#define LE_SUB_EVENT_ENHANCED_CONN_COMPLETE     0x0A
    u8      Data[0];
}
PACKED;

struct TBTHCIEventLEConnectionComplete
{
    TBTHCIEventLEMeta       Meta;
    u8      Status;
    u16     ConnectionHandle;
    u8      Role;
    u8      PeerAddressType;
    u8      PeerAddress[BT_BD_ADDR_SIZE];
    u16     ConnInterval;
    u16     ConnLatency;
    u16     SupervisionTimeout;
    u8      MasterClockAccuracy;
}
PACKED;

// ACL header (follows the 0x02 packet type byte on UART)
struct TBTHCIACLHeader
{
    u16     Handle;                         // bits 0-11 handle, 12-13 PB flag, 14-15 BC flag
    u16     DataLength;
}
PACKED;

// L2CAP
struct TBTL2CAPHeader
{
    u16     Length;
    u16     CID;
#define L2CAP_CID_ATT                   0x0004
#define L2CAP_CID_LE_SIGNALING          0x0005
#define L2CAP_CID_SMP                   0x0006
}
PACKED;

// ATT opcodes
#define ATT_OP_ERROR_RSP                0x01
#define ATT_OP_MTU_REQ                  0x02
#define ATT_OP_MTU_RSP                  0x03
#define ATT_OP_FIND_INFO_REQ            0x04
#define ATT_OP_FIND_INFO_RSP            0x05
#define ATT_OP_FIND_BY_TYPE_VALUE_REQ   0x06
#define ATT_OP_FIND_BY_TYPE_VALUE_RSP   0x07
#define ATT_OP_READ_BY_TYPE_REQ         0x08
#define ATT_OP_READ_BY_TYPE_RSP         0x09
#define ATT_OP_READ_REQ                 0x0A
#define ATT_OP_READ_RSP                 0x0B
#define ATT_OP_READ_BLOB_REQ            0x0C
#define ATT_OP_READ_BY_GROUP_TYPE_REQ   0x10
#define ATT_OP_READ_BY_GROUP_TYPE_RSP   0x11
#define ATT_OP_WRITE_REQ                0x12
#define ATT_OP_WRITE_RSP                0x13
#define ATT_OP_WRITE_CMD                0x52
#define ATT_OP_HANDLE_VALUE_NOTIFY      0x1B

// ATT error codes
#define ATT_ERR_INVALID_HANDLE          0x01
#define ATT_ERR_READ_NOT_PERMITTED      0x02
#define ATT_ERR_WRITE_NOT_PERMITTED     0x03
#define ATT_ERR_REQUEST_NOT_SUPPORTED   0x06
#define ATT_ERR_ATTRIBUTE_NOT_FOUND     0x0A

// GATT UUIDs (16 bit)
#define UUID_PRIMARY_SERVICE            0x2800
#define UUID_CHARACTERISTIC             0x2803
#define UUID_CCCD                       0x2902
#define UUID_GAP_SERVICE                0x1800
#define UUID_DEVICE_NAME                0x2A00
#define UUID_APPEARANCE                 0x2A01

// SMP
#define SMP_PAIRING_REQUEST             0x01
#define SMP_PAIRING_FAILED              0x05
#define SMP_REASON_PAIRING_NOT_SUPPORTED 0x05

#endif
