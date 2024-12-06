/**
 * @file LibMultiSense/SysNetworkMessage.hh
 *
 * Copyright 2013-2022
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
 *   2013-05-22, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysNetworkMessage
#define LibMultiSense_SysNetworkMessage

#include "../utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysNetwork {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_CMD_SYS_SET_NETWORK;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    //
    // Configurable interfaces

    static CRL_CONSTEXPR uint8_t Interface_Unknown   = 0;
    static CRL_CONSTEXPR uint8_t Interface_Primary   = 1;  // external GigE
    static CRL_CONSTEXPR uint8_t Interface_Secondary = 2;  // internal 100Mb

    //
    // IPV4 parameters

    uint8_t     interface;
    std::string address;
    std::string gateway;
    std::string netmask;

    //
    // Constructors

    SysNetwork(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysNetwork(const std::string& a,
               const std::string& g,
               const std::string& n) :
        interface(Interface_Primary),
        address(a),
        gateway(g),
        netmask(n) {};
    SysNetwork() :
        interface(Interface_Unknown),
        address(),
        gateway(),
        netmask() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;
        message & interface;
        message & address;
        message & gateway;
        message & netmask;
    }
};

}}}} // namespaces

#endif
