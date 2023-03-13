/**
 * @file LibMultiSense/DpuClassificationMessage.hh
 *
 * This message contains DPU classification data.
 *
 * Copyright 2022
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
 *   2022-07-20, bblakeslee@carnegierobotics.com, 2033.1, created file.
 **/

#ifndef LibMultiSense_DpuBboxResultHeader
#define LibMultiSense_DpuBboxResultHeader

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details{
namespace wire{

class WIRE_HEADER_ATTRIBS_ DpuBboxResultHeader {
public:
    static CRL_CONSTEXPR IdType ID           = ID_DATA_DPU_BBOX_RESULT_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif  // SENSORPOD_FIRMWARE

    int64_t frameId;
    int64_t timestamp;
    uint8_t success;
    uint32_t num_detections;

    DpuBboxResultHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif  // SENSORPOD_FIRMWARE
        frameId(0),
        timestamp(0),
        success(0),
        num_detections(0)
    {};
};

class BboxDetection {
public:
    static CRL_CONSTEXPR  VersionType VERSION = 1;
    uint16_t label;
    float score;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;

#ifndef SENSORPOD_FIRMWARE
        template<class Archive>
            void serialize(Archive& message,
                           const VersionType version)
        {
            (void) version;

            message & label;
            message & score;
            message & x;
            message & y;
            message & width;
            message & height;
        }
#endif  // SENSORPOD_FIRMWARE
};

class DpuBboxResult : public DpuBboxResultHeader {
public:
    std::vector<BboxDetection> detections;
#ifndef SENSORPOD_FIRMWARE
    DpuBboxResult(utility::BufferStreamReader &r, VersionType v) {serialize(r, v);};
    DpuBboxResult() : detections(std::vector<BboxDetection>{}) {};

    template<class Archive>
        void serialize(Archive& message, const VersionType version)
    {
        (void) version;

        message & frameId;
        message & timestamp;
        message & success;
        message & num_detections;

        const uint32_t wireBytes = static_cast<uint32_t>(num_detections *sizeof(BboxDetection));

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {
            message.write(reinterpret_cast<void *>(detections.data()), wireBytes);
        } else {
            void * rawDetectionsData = message.peek();
            message.seek(message.tell() + wireBytes);

            detections.resize(num_detections);
            memcpy(detections.data(), rawDetectionsData, wireBytes);
        }
    }
#endif  // SENSORPOD_FIRMWARE
};

}}}}    // Namespace termination

#endif // LibMultiSense_DpuBboxResultHeader
