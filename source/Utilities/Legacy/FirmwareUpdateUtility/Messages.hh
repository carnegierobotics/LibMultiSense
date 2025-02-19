/**
 * @file FirmwareUpdateUtility/Messages.hh
 *
 * Copyright 2024-2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Significant history (date, user, job code, action):
 *   2024-27-08, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

 #ifndef __FUU_Messages_H__
 #define __FUU_Messages_H__

#ifdef WIN32
#define INET_ADDRSTRLEN 16
#include <WinSock2.h>
#else 
#include <netinet/in.h>
#endif


#include <stdint.h>
#include <cstring>
#include <string>
#include <iostream>

#if defined (CRL_HAVE_CONSTEXPR)
#define CRL_CONSTEXPR constexpr
#else
#define CRL_CONSTEXPR const
#endif

namespace Messages    {

static CRL_CONSTEXPR int SERVER_PORT = 4321;
static CRL_CONSTEXPR int BLOCK_SIZE = 1024;

typedef uint32_t MessageType;
static CRL_CONSTEXPR MessageType Message_BlockAck   = 1;
static CRL_CONSTEXPR MessageType Message_Setup      = 2;
static CRL_CONSTEXPR MessageType Message_FileBlock  = 3;
static CRL_CONSTEXPR MessageType Message_Status     = 4;
static CRL_CONSTEXPR MessageType Message_Reboot     = 5;

typedef uint32_t StatusMessage;
static CRL_CONSTEXPR StatusMessage Status_NotStarted          = (0x1u<<0);
static CRL_CONSTEXPR StatusMessage Status_FileNotReceived     = (0x1u<<1);
static CRL_CONSTEXPR StatusMessage Status_FileReceived        = (0x1u<<2);
static CRL_CONSTEXPR StatusMessage Status_FileCheckComplete   = (0x1u<<3);
static CRL_CONSTEXPR StatusMessage Status_FileCheckFailed     = (0x1u<<4);
static CRL_CONSTEXPR StatusMessage Status_BackupComplete      = (0x1u<<5);
static CRL_CONSTEXPR StatusMessage Status_BackupFailed        = (0x1u<<6);
static CRL_CONSTEXPR StatusMessage Status_BootImageComplete   = (0x1u<<7);
static CRL_CONSTEXPR StatusMessage Status_BootImageFailed     = (0x1u<<8);
static CRL_CONSTEXPR StatusMessage Status_OSImageComplete     = (0x1u<<9);
static CRL_CONSTEXPR StatusMessage Status_OSImageFailed       = (0x1u<<10);
static CRL_CONSTEXPR StatusMessage Status_FSComplete          = (0x1u<<11);
static CRL_CONSTEXPR StatusMessage Status_FSFailed            = (0x1u<<12);
static CRL_CONSTEXPR StatusMessage Status_UpdateComplete      = (0x1u<<13);
static CRL_CONSTEXPR StatusMessage Status_UpdateFailed        = (0x1u<<14);
static CRL_CONSTEXPR StatusMessage Status_ClientConnected     = (0x1u<<31);
static CRL_CONSTEXPR StatusMessage Status_Error =    (Status_FileNotReceived |
                                                      Status_FileCheckFailed |
                                                      Status_BackupFailed |
                                                      Status_BootImageFailed |
                                                      Status_OSImageFailed |
                                                      Status_FSFailed |
                                                      Status_UpdateFailed);
inline std::string StatusString(StatusMessage s)
{
    switch (s)
    {
        case Status_NotStarted:         return "Not Started";
        case Status_FileNotReceived:    return "File Not Received";
        case Status_FileReceived:       return "File Received";
        case Status_FileCheckComplete:  return "File Integrity Check Complete";
        case Status_FileCheckFailed:    return "File Integrity Check Failed";
        case Status_BackupComplete:     return "Backup Complete";
        case Status_BackupFailed:       return "Backup Failed";
        case Status_BootImageComplete:  return "Boot Image Write Complete";
        case Status_BootImageFailed:    return "Boot Image Write Failed";
        case Status_OSImageComplete:    return "OS Image Write Complete";
        case Status_OSImageFailed:      return "OS Image Write Failed";
        case Status_FSComplete:         return "FS Write Complete";
        case Status_FSFailed:           return "FS Write Failed";
        case Status_UpdateComplete:     return "Update Complete";
        case Status_UpdateFailed:       return "Update Failed";
        case Status_ClientConnected:    return "Client Connected";
        default: return "Unknown Status";
    }
}

inline void PrintStatus(StatusMessage s)
{
    std::cout << "-------------------------------------------------\n";
    for (size_t i = 0; i < 32U; i++)
    {
        if ((1 << i) & s)
        {
            std::cout << StatusString(1<<i) << std::endl;
        }
    }
    std::cout << "-------------------------------------------------\n\n";
}

typedef int32_t UpdateStatusMessage;
static CRL_CONSTEXPR UpdateStatusMessage UpdateStatus_Error = -1;
static CRL_CONSTEXPR UpdateStatusMessage UpdateStatus_Ok    = 0;
static CRL_CONSTEXPR UpdateStatusMessage UpdateStatus_Done  = 1;

typedef uint32_t UpdateStatusId;
static CRL_CONSTEXPR UpdateStatusId UpdateStatusId_Message = 0;
static CRL_CONSTEXPR UpdateStatusId UpdateStatusId_ACK     = 1;

#if defined (_MSC_VER) || defined(__MINGW64__)
#define PACK 
#pragma pack(push,1)
#else
#define PACK __attribute__((packed,aligned(4)))
#endif

struct PACK MessageSetup {

    MessageType Id;
    struct sockaddr_in ClientAddress;
    struct sockaddr_in ServerAddress;

    MessageSetup(struct sockaddr_in & cl, struct sockaddr_in & sr)
    {
        Id = Message_Setup;
        memcpy(&ClientAddress, &cl, sizeof(struct sockaddr_in));
        memcpy(&ServerAddress, &sr, sizeof(struct sockaddr_in));
    }
};

struct PACK MessageBlockAck {
    MessageType Id;
    uint32_t Sequence;
    uint32_t Offset;
    MessageBlockAck()
    : Id(0), Sequence(0), Offset(0)
    {
    }
};

struct PACK MessageFileBlock {
    MessageType Id;
    uint32_t Length;
    uint32_t Offset;
    uint32_t Checksum;
    uint32_t ChunkLen;
    uint32_t Sequence;
    uint8_t Buffer[BLOCK_SIZE];
    MessageFileBlock(uint32_t _Length, uint32_t _Offset, uint32_t _CheckSum,
                     uint32_t _ChunkLen, uint32_t _Sequence, uint8_t *_Buffer)
    : Id(Message_FileBlock), Length(_Length), Offset(_Offset),
      Checksum(_CheckSum), ChunkLen(_ChunkLen), Sequence(_Sequence)
    {
        memcpy(Buffer, _Buffer, ChunkLen);
    }
};

struct PACK MessageUpdateStatus {
    uint32_t State;
    float Progress;
    uint32_t Id;
    uint32_t Status;
    uint32_t Reserved[4];

    MessageUpdateStatus()
    :State(0), Progress(0.0f), Id(0), Status(0), Reserved()
    {
    }
};

#if defined (_MSC_VER) || defined(__MINGW64__)
#pragma pack(pop)
#endif
} //namespace Messages

#endif /* end of include guard: __FUU_Messages_H__ */
