/**
 * @file LibMultiSense/FeatureDetectorMessage.hh
 *
 * This message contains first class feature data.
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
 *   2024-25-01, patrick.smith@carnegierobotics.com, IRAD, created file.
 *   2024-22-11, patrick.smith@carnegierobotics.com, IRAD, Moved file.
 **/

#ifndef LibMultiSense_FeatureDetectorMessage
#define LibMultiSense_FeatureDetectorMessage

#include <cmath>

#include <utility/Portability.hh>
#include <utility/BufferStream.hh>
#include <wire/Protocol.hh>

using namespace crl::multisense::details;

#pragma pack(push, 1)

class Feature {
public:
    uint16_t x;
    uint16_t y;
    uint8_t angle;
    uint8_t resp;
    uint8_t octave;
    uint8_t descriptor;
};

class Descriptor {
public:
    uint32_t d[8];
};

#pragma pack(pop)

class WIRE_HEADER_ATTRIBS_ FeatureDetectorHeader {
public:
    static CRL_CONSTEXPR   wire::VersionType VERSION    = 1;
    wire::VersionType      version;
    uint64_t               source;
    int64_t                frameId;
    uint16_t               numFeatures;
    uint16_t               numDescriptors;

    FeatureDetectorHeader() :
        version(VERSION),
        source(0),
        frameId(0),
        numFeatures(0),
        numDescriptors(0)
    {};

};

#ifndef SENSORPOD_FIRMWARE

class FeatureDetector : public FeatureDetectorHeader {
public:

    void * dataP;

    //
    // Constructors

    FeatureDetector(utility::BufferStreamReader&r, wire::VersionType v) {serialize(r,v);};
    FeatureDetector() {};


    template<class Archive>
        void serialize(Archive&          message,
                       const wire::VersionType _version)
    {
        (void) _version;

        message & version;
        message & source;
        message & frameId;
        message & numFeatures;
        message & numDescriptors;

        const uint32_t featureDataSize = static_cast<uint32_t> (std::ceil( numFeatures*sizeof(Feature) + numDescriptors*sizeof(Descriptor)));

        dataP = message.peek();
        message.seek(message.tell() + featureDataSize);
    }

};

#endif // !SENSORPOD_FIRMWARE


#endif
