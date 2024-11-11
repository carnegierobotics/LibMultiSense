/**
 * @file LibMultiSense/SecondaryAppMetaMessage.hh
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
 *   2024-10-31, patrick.smith@carnegierobotics.com, IRAD, created file.
 **/

#ifndef LibMultiSense_SecondaryAppMetaData
#define LibMultiSense_SecondaryAppMetaData

#include <typeinfo>
#include <cmath>

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ SecondaryAppMetaHeader {
public:

static CRL_CONSTEXPR IdType      ID      = ID_DATA_FEATURE_DETECTOR_META;
static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    int64_t frameId;
    size_t  dataLength;
    void *  dataP;

    SecondaryAppMetaHeader():
#ifdef SENSORDPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        dataLength(0),
        dataP(nullptr)
      {};
};

#ifndef SENSORPOD_FIRMWARE

class SecondaryAppMetadata : public SecondaryAppMetaHeader {
public:


    //
    // Constructors

    SecondaryAppMetadata(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SecondaryAppMetadata() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(dataP, dataLength);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + dataLength);
        }

    }
};

#endif // !SENSORPOD_FIRMWARE

}}}} // namespaces

#endif
