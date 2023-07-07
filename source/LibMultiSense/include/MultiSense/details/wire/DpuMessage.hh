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

    // Frame metadata
    int64_t frameId;
    int64_t timestamp;
    bool success;

    // Tensor metadata
    // TODO: Remove magic numbers
    uint32_t resultType;
    // uint8_t numDetections;
    // uint8_t sequenceId;
    uint16_t maskRank;
    uint16_t maskDims[3];
    uint32_t maskBlobLen;
    
    // Tensor data
    // uint8_t classId[1];
    // float confidenceScore[1];
    // uint16_t bboxArray[4];
    uint8_t maskArray[160 * 256];

    DpuResultHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif  // SENSORPOD_FIRMWARE
        frameId(0),
        timestamp(0),
        success(0),
        resultType(0),
        // numDetections(0),
        // sequenceId(0),
        maskRank(0),
        maskBlobLen(0)
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
        // message & numDetections;
        // message & sequenceId;
        
        // Serialize mask metadata 
        message & maskRank;
        for (int i = 0; i < 3; i++) {
            message & maskDims[i];
        }
        message & maskBlobLen;

        // Serialze data
        // message & classId[0];
        // message & confidenceScore[0];
        
        // Serialize bboxes
        /*
        for (int i = 0; i < 4; i++) {
            message & bboxArray[i];
        }
        */

        // Serialize masks
        for (uint32_t i = 0; i < maskBlobLen; i++) {
            message & maskArray[i];
        }
    }
};
#endif  // !SENSORPOD_FIRMWARE

}}}}    // Namespace termination

#endif
