/**
 * @file LibMultiSense/ImageMetaMessage.h
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

#ifndef LibMultiSense_ImageMetaMessage
#define LibMultiSense_ImageMetaMessage

#include <typeinfo>

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ ImageMetaHeader {
public:

    static CONSTEXPR IdType      ID      = ID_DATA_IMAGE_META;
    static CONSTEXPR VersionType VERSION = 1;

    static CONSTEXPR uint32_t HISTOGRAM_CHANNELS = 4; // g0, r, b, g1
    static CONSTEXPR uint32_t HISTOGRAM_BINS     = 256;
    static CONSTEXPR uint32_t HISTOGRAM_LENGTH   = (HISTOGRAM_CHANNELS * HISTOGRAM_BINS *
                                                sizeof(uint32_t));

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    int64_t            frameId;
    float              framesPerSecond;
    float              gain;
    uint32_t           exposureTime;
    uint32_t           timeSeconds;
    uint32_t           timeMicroSeconds;
    int32_t            angle; // microradians

    ImageMetaHeader() 
        :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        timeSeconds(0),
        timeMicroSeconds(0),    
        angle(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class ImageMeta : public ImageMetaHeader {
public:

    uint32_t histogramP[HISTOGRAM_BINS * HISTOGRAM_CHANNELS];

    //
    // Constructors

    ImageMeta(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ImageMeta() {};
  
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & frameId;
        message & framesPerSecond;
        message & gain;
        message & exposureTime;
        message & timeSeconds;
        message & timeMicroSeconds;
        message & angle;
        
        if (typeid(Archive) == typeid(utility::BufferStreamWriter))
            message.write(histogramP, HISTOGRAM_LENGTH);
        else
            message.read(histogramP, HISTOGRAM_LENGTH);
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
