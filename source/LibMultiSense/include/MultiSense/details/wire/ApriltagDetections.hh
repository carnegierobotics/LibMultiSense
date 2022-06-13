/**
 * @file LibMultiSense/ApriltagDetections.hh
 *
 * This message contains apriltag detection data.
 *
 * Copyright 2021-2022
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
 *   2021-04-15, drobinson@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_ApriltagDetections
#define LibMultiSense_ApriltagDetections

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ ApriltagDetectionsHeader {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_APRILTAG_DETECTIONS_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    //
    // Frame ID, timestamp and success flag of images that the algorithm was processed on

    int64_t     frameId;
    int64_t     timestamp;
    uint8_t     success;

    //
    // Constructors

    ApriltagDetectionsHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        timestamp(0),
        success(0)
    {};
};

class ApriltagDetection {
public:
    static CRL_CONSTEXPR VersionType VERSION = 1;

    uint64_t family;
    uint32_t id;
    uint8_t hamming;
    float decision_margin;
    double image_H_tag[3][3];
    double center[2];
    double corners[4][2];

#ifndef SENSORPOD_FIRMWARE
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & family;
        message & id;
        message & hamming;
        message & decision_margin;

        SER_ARRAY_2(image_H_tag, 3, 3);
        SER_ARRAY_1(center, 2);
        SER_ARRAY_2(corners, 4, 2);
    }
#endif // !SENSORPOD_FIRMWARE
};

class ApriltagDetections : ApriltagDetectionsHeader {
public:

    //
    // Apriltag detections

    std::vector<ApriltagDetection> detections;

#ifndef SENSORPOD_FIRMWARE

    //
    // Constructors

    ApriltagDetections(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};

    ApriltagDetections() : detections(std::vector<ApriltagDetection>{}) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & frameId;
        message & timestamp;
        message & success;
        message & detections;
    }
#endif // !SENSORPOD_FIRMWARE
};

}}}} // namespaces

#endif
