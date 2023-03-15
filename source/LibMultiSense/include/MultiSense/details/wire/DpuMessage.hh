/**
 * @file LibMultiSense/DpuMessage.hh
 *
 * This message contains DPU result data.
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
 *   2023-03-13, bblakeslee@carnegierobotics.com, IRAD.8013.4, created file.
 **/

#ifndef LibMultiSense_DpuResultHeader
#define LibMultiSense_DpuResultHeader

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ DpuResultHeader {
public:
    static CRL_CONSTEXPR IdType ID           = ID_DATA_DPU_RESULT_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif  // SENSORPOD_FIRMWARE

    int64_t frameId;
    int64_t timestamp;
    uint8_t success;

    uint32_t resultType;

    uint16_t classRank;
    uint16_t confidenceRank;
    uint16_t bboxRank;
    uint16_t maskRank;

    // TODO: Hard coded memory offsets.  Make dynamic later.
    uint16_t classDims[1];
    uint16_t confidenceDims[1];
    uint16_t bboxDims[2];
    uint16_t maskDims[3];

    DpuResultHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif  // SENSORPOD_FIRMWARE
        frameId(0),
        timestamp(0),
        success(0),
        resultType(0),
        classRank(0),
        confidenceRank(0),
        bboxRank(0),
        maskRank(0)
    {};
};

#ifndef SENSORPOD_FIRMWARE

class DpuResult : public DpuResultHeader {
public:
    DpuResult(utility::BufferStreamReader &r, VersionType v) {serialize(r, v);};

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        // Serialize metadata
        message & frameId;
        message & timestamp;
        message & success;
        message & resultType;
        
        // Serialize tensor ranks
        message & classRank;
        message & confidenceRank;
        message & bboxRank;
        message & maskRank;

        // Serialize tensor dimensions
        for (int i = 0; i < classRank; i++) {
            message & classDims[i];
        }
        for (int i = 0; i < confidenceRank; i++) {
            message & confidenceDims[i];
        }
        for (int i = 0; i < bboxRank; i++) {
            message & bboxDims[i];
        }
        for (int i = 0; i < maskRank; i++) {
            message & maskDims[i];
        }
    }
};
#endif  // !SENSORPOD_FIRMWARE

}}}}    // Namespace termination

#endif
