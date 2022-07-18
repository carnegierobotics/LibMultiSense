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

    // The frame ID of the image that the apriltags were detected on
    int64_t     frameId;

    // The frame timestamp (nanoseconds) of the image that the apriltags were detected on
    int64_t     timestamp;

    // The image source that the apriltags were detected on
    char imageSource[32];

    // Success flag to indicate whether for the apriltag algorithm ran successfully
    uint8_t     success;

    // The number of apriltags that were detected
    uint32_t    numDetections;

    //
    // Constructors

    ApriltagDetectionsHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        timestamp(0),
        success(0),
        numDetections(0)
    {
        strncpy(imageSource, "", sizeof(imageSource));
    };
};

class ApriltagDetection {
public:
    static CRL_CONSTEXPR VersionType VERSION = 1;

    // The family of the tag
    char family[32];

    // The ID of the tag
    uint32_t id;

    // The hamming distance between the detection and the real code
    uint8_t hamming;

    // The quality/confidence of the binary decoding process
    // average difference between intensity of data bit vs decision thresh.
    // Higher is better. Only useful for small tags
    float decisionMargin;

    // The 3x3 homography matrix describing the projection from an
    // "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
    // -1)) to pixels in the image
    double tagToImageHomography[3][3];

    // The 2D position of the origin of the tag in the image
    double center[2];

    // The 4 tag corner pixel locations in the order:
    // point 0: [-squareLength / 2, squareLength / 2]
    // point 1: [ squareLength / 2, squareLength / 2]
    // point 2: [ squareLength / 2, -squareLength / 2]
    // point 3: [-squareLength / 2, -squareLength / 2]
    double corners[4][2];

#ifndef SENSORPOD_FIRMWARE
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        SER_ARRAY_1(family, 32);
        message & id;
        message & hamming;
        message & decisionMargin;
        SER_ARRAY_2(tagToImageHomography, 3, 3);
        SER_ARRAY_1(center, 2);
        SER_ARRAY_2(corners, 4, 2);
    }
#endif // !SENSORPOD_FIRMWARE
};

class ApriltagDetections : public ApriltagDetectionsHeader {
public:

    //
    // Apriltag detections

    std::vector<ApriltagDetection> detections;

#ifndef SENSORPOD_FIRMWARE

    //
    // Constructors

    ApriltagDetections(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};

    ApriltagDetections() : detections(std::vector<ApriltagDetection>()) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & frameId;
        message & timestamp;
        SER_ARRAY_1(imageSource, 32);
        message & success;
        message & numDetections;

        const uint32_t wireBytes = static_cast<uint32_t>(numDetections * sizeof(ApriltagDetection));

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(reinterpret_cast<void*>(detections.data()), wireBytes);

        } else {

            void *rawDetectionsData = message.peek();
            message.seek(message.tell() + wireBytes);

            detections.resize(numDetections);
            memcpy(detections.data(), rawDetectionsData, wireBytes);
        }

    }
#endif // !SENSORPOD_FIRMWARE
};

}}}} // namespaces

#endif
