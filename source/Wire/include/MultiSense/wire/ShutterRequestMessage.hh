/**
 * @file ShutterRequestMessage.hh
 *
 * This message requests a one-shot thermal calibration or delay action
 *
 * Copyright 2026
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
 **/

#ifndef LibMultiSense_ShutterRequestMessage
#define LibMultiSense_ShutterRequestMessage

#include <atomic>
#include <chrono>
#include "MultiSense/wire/Protocol.hh"
#include "MultiSense/utility/BufferStream.hh"

namespace crl { namespace multisense { namespace details { namespace wire {

// One-shot thermal shutter actions, independent of camera configuration.
class ShutterRequest {
public:
    static CRL_CONSTEXPR IdType ID = ID_CMD_SHUTTER_REQUEST;
    static CRL_CONSTEXPR VersionType VERSION = 1;
    enum Action : uint8_t { CALIBRATE = 0, DELAY = 1, RESUME = 2 };
    static CRL_CONSTEXPR uint32_t MAX_DELAY_MS = 60000;

    uint64_t requestId = 0;
    uint8_t action = CALIBRATE;
    uint32_t delayMs = 0;

    ShutterRequest() = default;
    ShutterRequest(utility::BufferStreamReader &r, VersionType v) { serialize(r, v); }

    // Each API call gets a new id; retransmissions retain the original id.
    static uint64_t nextRequestId()
    {
        static std::atomic<uint64_t> next{
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count())};
        return ++next;
    }

    bool valid() const
    {
        return requestId != 0 &&
            ((action == DELAY && delayMs > 0 && delayMs <= MAX_DELAY_MS) ||
             ((action == CALIBRATE || action == RESUME) && delayMs == 0));
    }

    template<class Archive>
    void serialize(Archive &message, const VersionType version)
    {
        (void) version;
        message & requestId;
        message & action;
        message & delayMs;
    }
};

}}}}
#endif
