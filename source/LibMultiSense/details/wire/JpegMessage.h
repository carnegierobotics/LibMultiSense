/**
 * @file LibMultiSense/JpegMessage.h
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
 *   2013-06-12, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_JpegMessage
#define LibMultiSense_JpegMessage

#include <typeinfo>
#include <cmath>

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ JpegImageHeader {
public:

static CRL_CONSTEXPR IdType      ID      = ID_DATA_JPEG_IMAGE;
static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    uint32_t source;
    int64_t  frameId;
    uint16_t width;
    uint16_t height;
    uint32_t length;
    uint32_t quality;

    JpegImageHeader() : 
#ifdef SENSORDPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        source(0),
        frameId(0),
        width(0),
        height(0),
        length(0),
        quality(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class JpegImage : public JpegImageHeader {
public:

    void *dataP;

    //
    // Constructors

    JpegImage(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    JpegImage() : dataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & source;
        message & frameId;
        message & width;
        message & height;
        message & length;
        message & quality;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {
          
            message.write(dataP, length);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + length);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
