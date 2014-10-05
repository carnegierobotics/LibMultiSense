/**
 * @file LibMultiSense/SysFlashOpMessage.h
 *
 * This message contains information on the number and layout of non-volatile
 * memory devices in the system.
 *
 * Copyright 2013
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
 *   2012-08-02, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_SysFlashOpMessage
#define LibMultiSense_SysFlashOpMessage

#include <typeinfo>

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysFlashOp {
public:
    static CONSTEXPR IdType      ID      = ID_CMD_SYS_FLASH_OP;
    static CONSTEXPR VersionType VERSION = 1; 

    //
    // Maximum payload length per operation

    static CONSTEXPR uint32_t MAX_LENGTH = 1024;

    //
    // Parameters representing the desired flash operation

    static CONSTEXPR uint32_t OP_STATUS  = 0; // just check status
    static CONSTEXPR uint32_t OP_ERASE   = 1; // erase entire region
    static CONSTEXPR uint32_t OP_PROGRAM = 2; // program/verify chunk within region
    static CONSTEXPR uint32_t OP_VERIFY  = 3; // just verify chunk within region

    uint32_t operation;

    //
    // Parameters representing the desired flash region

    static CONSTEXPR uint32_t RGN_BITSTREAM   = 0; // FPGA configuration bitstream
    static CONSTEXPR uint32_t RGN_FIRMWARE    = 1; // Microblaze firmware

    uint32_t region;

    //
    // Remaining fields are only used for OP_PROGRAM and OP_VERIFY:

    uint32_t start_address;  // start address of chunk to program or verify
    uint32_t length;         // size of chunk to program or verify (power-of-2)

    uint8_t data[MAX_LENGTH];

    //
    // Constructors

    SysFlashOp(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysFlashOp(uint32_t op=OP_STATUS, 
               uint32_t r=RGN_BITSTREAM,
               uint32_t s=0,
               uint32_t l=0) : operation(op), 
                               region(r),
                               start_address(s),
                               length(l) {};
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & operation;
        message & region;

        switch(operation) {
            case OP_PROGRAM:
            case OP_VERIFY:

                message & start_address;
                message & length;

                if(length > MAX_LENGTH)
                    CRL_EXCEPTION("length (%u) exceeds MAX_LENGTH (%u)", 
                                  length, MAX_LENGTH);

                if (typeid(Archive) == typeid(utility::BufferStreamWriter))
                    message.write(data, length);
                else
                    message.read(data, length);

                break;
            case OP_STATUS:
            case OP_ERASE:
                // start/length/data not required
                break;
            default:
                CRL_EXCEPTION("unknown operation (%d)", (int)operation);
        }

        switch(region) {
            case RGN_BITSTREAM:
            case RGN_FIRMWARE:
                break;
            default:
                CRL_EXCEPTION("unknown region (%d)", (int)region);
        }
    }
};

}}}}; // namespaces

#endif
