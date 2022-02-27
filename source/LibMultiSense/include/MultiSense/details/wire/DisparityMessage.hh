/**
 * @file LibMultiSense/DisparityMessage.hh
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_DisparityMessage
#define LibMultiSense_DisparityMessage

#include <typeinfo>
#include <cmath>

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ DisparityHeader {
public:

    static CRL_CONSTEXPR IdType      ID      = ID_DATA_DISPARITY;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    static CRL_CONSTEXPR uint8_t  WIRE_BITS_PER_PIXEL = MULTISENSE_WIRE_BITS_PER_PIXEL;
    static CRL_CONSTEXPR uint8_t  WIRE_BYTE_ALIGNMENT = 3;
    static CRL_CONSTEXPR uint8_t  API_BITS_PER_PIXEL  = MULTISENSE_API_BITS_PER_PIXEL; // after custom assemble()
    static CRL_CONSTEXPR uint32_t META_LENGTH         = 16; // packed, includes type/version

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    int64_t  frameId;
    uint16_t width;
    uint16_t height;

    DisparityHeader()
        :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        width(0),
        height(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class Disparity : public DisparityHeader {
public:

    void *dataP;

    //
    // Constructors

    Disparity(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    Disparity() : dataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;
        message & frameId;
        message & width;
        message & height;

        const uint32_t imageSize = static_cast<uint32_t> (std::ceil(((double) API_BITS_PER_PIXEL / 8.0) * width * height));

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(dataP, imageSize);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + imageSize);
        }
    }

    //
    // UDP assembler

    static void assembler(utility::BufferStreamWriter& stream,
                          const uint8_t               *dataP,
                          uint32_t                     offset,
                          uint32_t                     length)
    {
        //
        // Special case, 1st packet contains header only. Firmware
        // does not have to worry about the header length being
        // byte-aligned on a WIRE_BITS_PER_PIXEL boundary

        if (0 == offset) {
            stream.seek(0);
            stream.write(dataP, META_LENGTH);
            return;
        }

        //
        // The data section of each incoming packet is byte-aligned
        // on a WIRE_BITS_PER_PIXEL boundary

        const uint8_t *sP           = dataP;
        const uint32_t sourceOffset = offset - META_LENGTH;
        const uint32_t count        = (8 * length) / WIRE_BITS_PER_PIXEL;
        const uint32_t destOffset   = META_LENGTH + (((8 * sourceOffset) / WIRE_BITS_PER_PIXEL) *
                                                     (API_BITS_PER_PIXEL / 8));
        //
        // Seek to the proper location

        stream.seek(destOffset);

#if MULTISENSE_WIRE_BITS_PER_PIXEL == 12 && MULTISENSE_API_BITS_PER_PIXEL == 16
        //
        // This conversion is for (WIRE == 12bits), (API == 16bits, 1/16th pixel, unsigned integer)

            uint16_t *dP = reinterpret_cast<uint16_t*>(stream.peek());

            for(uint32_t i=0; i<count; i+=2, sP+=3) {
                dP[i]   = ((sP[0]     ) | ((sP[1] & 0x0F) << 8));
                dP[i+1] = ((sP[1] >> 4) |  (sP[2] << 4)        );
            }

#elif MULTISENSE_WIRE_BITS_PER_PIXEL == 12 && MULTISENSE_API_BITS_PER_PIXEL == 32
        //
        // This conversion is for (WIRE == 12bits), (API == 32bits, floating point)

            float *dP = reinterpret_cast<float*>(stream.peek());

            for(uint32_t i=0; i<count; i+=2, sP+=3) {

                dP[i]   = static_cast<float>((sP[0]     ) | ((sP[1] & 0x0F) << 8)) / 16.0f;
                dP[i+1] = static_cast<float>((sP[1] >> 4) |  (sP[2] << 4)        ) / 16.0f;
            }

#else
#error MULTISENSE_WIRE_BITS_PER_PIXEL and MULTISENSE_API_BITS_PER_PIXEL not supported
#endif

    }
};

#endif // !SENSORPOD_FIRMWARE

}}}} // namespaces

#endif
