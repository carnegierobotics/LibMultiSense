/**
 * @file StreamControlMessage.hh
 *
 * Copyright 2013-2025
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
 *   2013-06-13, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_StreamControlMessage
#define LibMultiSense_StreamControlMessage

#include "utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class StreamControl {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_CMD_STREAM_CONTROL;
    static CRL_CONSTEXPR VersionType VERSION = 2;

    //
    // Set modify mask bit high to have the device
    // accept the corresponding bit in the control mask.

    SourceType modifyMask;
    SourceType controlMask;

    //
    // Convenience functions

    void enable(SourceType mask) {
        modifyMask = controlMask = mask;
    };
    void disable(SourceType mask) {
        modifyMask = mask; controlMask = 0;
    };

    //
    // Constructor

    StreamControl(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    StreamControl()
        : modifyMask(0), controlMask(0) {};

    //
    // Serialization routine.

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {

        uint32_t modifyMaskLow = 0;
        uint32_t modifyMaskHigh = 0;
        uint32_t controlMaskLow = 0;
        uint32_t controlMaskHigh = 0;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            modifyMaskLow   = (uint32_t)(modifyMask>>0);
            modifyMaskHigh  = (uint32_t)((modifyMask&0xFFFFFFFF00000000ull)>>32);
            controlMaskLow  = (uint32_t)(controlMask>>0);
            controlMaskHigh = (uint32_t)((controlMask&0xFFFFFFFF00000000ull)>>32);

            message & modifyMaskLow;
            message & controlMaskLow;

            if (version >= 2)
            {
                message & modifyMaskHigh;
                message & controlMaskHigh;
            }

        } else {


            message & modifyMaskLow;
            message & controlMaskLow;

            if (version >= 2)
            {
                message & modifyMaskHigh;
                message & controlMaskHigh;
            }

            modifyMask  = ((uint64_t)modifyMaskHigh) << 32 | modifyMaskLow;
            controlMask = ((uint64_t)controlMaskHigh) << 32 | controlMaskLow;

        }

    }
};

}}}} // namespaces

#endif
