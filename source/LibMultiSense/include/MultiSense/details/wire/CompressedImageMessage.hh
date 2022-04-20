/**
 * @file LibMultiSense/CompressedImageMessage.hh
 *
 * Copyright 2021-2022
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * https://www.carnegierobotics.com
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
 *   2021-08-24, malvarado@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_CompressedImageMessage
#define LibMultiSense_CompressedImageMessage

#include <typeinfo>
#include <cmath>

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ CompressedImageHeader {
public:

static CRL_CONSTEXPR IdType      ID      = ID_DATA_COMPRESSED_IMAGE;
static CRL_CONSTEXPR VersionType VERSION = 0;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    uint32_t source;
    uint32_t bitsPerPixel;
    uint32_t codec;
    int64_t  frameId;
    uint16_t width;
    uint16_t height;
    uint32_t exposure;
    float gain;
    uint32_t compressedDataBufferSize;

    CompressedImageHeader()
        :
#ifdef SENSORDPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        source(0),
        bitsPerPixel(0),
        codec(0),
        frameId(0),
        width(0),
        height(0),
        exposure(0),
        gain(0.0),
        compressedDataBufferSize(0)
        {};
};

#ifndef SENSORPOD_FIRMWARE

class CompressedImage : public CompressedImageHeader {
public:

    void *dataP;

    //
    // Constructors

    CompressedImage(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CompressedImage() : dataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;
        message & source;
        message & bitsPerPixel;
        message & codec;
        message & frameId;
        message & width;
        message & height;
        message & exposure;
        message & gain;
        message & compressedDataBufferSize;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(dataP, compressedDataBufferSize);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + compressedDataBufferSize);
        }

    }
};

#endif // !SENSORPOD_FIRMWARE

}}}} // namespaces

#endif
