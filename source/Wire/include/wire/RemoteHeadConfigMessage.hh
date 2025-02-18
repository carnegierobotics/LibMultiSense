/**
 * @file RemoteHeadConfig.hh
 *
 * This message contains the current remote head vpb configuration.
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
 *   2023-03-03, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef LibMultiSense_RemoteHeadConfigMessage
#define LibMultiSense_RemoteHeadConfigMessage

#include "utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class RemoteHeadChannel {
public:
    static CRL_CONSTEXPR VersionType VERSION = 1;

    ::crl::multisense::RemoteHeadChannel channel;

    //
    // Serialization routine
    //
    template<class Archive>
    void serialize(Archive&          message,
                   const VersionType version)
    {
        (void) version;

        message & channel;
    }
};

class RemoteHeadSyncGroup {
public:
    static CRL_CONSTEXPR VersionType VERSION = 1;

    wire::RemoteHeadChannel controller;
    std::vector<wire::RemoteHeadChannel> responders;

    //
    // Serialization routine
    //
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & controller;
        message & responders;
    }
};

class RemoteHeadConfig {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_CMD_REMOTE_HEAD_CONFIG;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    //
    // Parameters representing the current camera configuration

    std::vector<wire::RemoteHeadSyncGroup> syncGroups;

    //
    // Constructors

    RemoteHeadConfig(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    RemoteHeadConfig() : syncGroups() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & syncGroups;
    }
};

}}}} // namespaces

#endif
