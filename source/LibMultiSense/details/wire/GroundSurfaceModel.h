/**
 * @file LibMultiSense/GroundSurfaceModel.h
 *
 * This message contains ground surface spline data.
 *
 * Copyright 2021
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

#ifndef LibMultiSense_GroundSurfaceModelHeader
#define LibMultiSense_GroundSurfaceModelHeader

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

struct Boundary {
    float maxX;
    float minX;
    float maxY;
    float minY;
    float maxAzimuth;
    float minAzimuth;

    //
    // Constructors
    Boundary() :
        maxX(0.0f),
        minX(0.0f),
        maxY(0.0f),
        minY(0.0f),
        maxAzimuth(0.0f),
        minAzimuth(0.0f)
    {};
};

class WIRE_HEADER_ATTRIBS_ GroundSurfaceModelHeader {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_GROUND_SURFACE_SPLINE_DATA_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    //
    // Frame ID and timestamp of images that the algorithm was processed on

    int64_t     frameId;
    int64_t     timestamp;

    //
    // Control points dynamic array (treated as image) members

    uint32_t    controlPointsBitsPerPixel;
    uint32_t    controlPointsWidth;
    uint32_t    controlPointsHeight;

    //
    // Spline origin and size details

    std::array<float, 2> xyCellOrigin;
    std::array<float, 2> xyCellSize;
    std::array<float, 2> xyLimit;
    std::array<float, 2> minMaxAzimuthAngle;

    //
    // Extrinsic calibration that was used during the computation of the spline

    std::array<float, 6> extrinsics;

    //
    // Ground Base Model (ax^2 + by^2 + cxy + dx + ey + f)

    std::array<float, 6> quadraticParams;

    //
    // Constructors
    GroundSurfaceModelHeader() :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        controlPointsBitsPerPixel(0),
        controlPointsWidth(0),
        controlPointsHeight(0),
        xyCellOrigin({0.0f, 0.0f}),
        xyCellSize({0.0f, 0.0f}),
        xyLimit({0.0f, 0.0f}),
        minMaxAzimuthAngle({0.0f, 0.0f})
    {};
};

#ifndef SENSORPOD_FIRMWARE

class GroundSurfaceModel : public GroundSurfaceModelHeader {
public:

    void *controlPointsDataP;

    //
    // Constructors

    GroundSurfaceModel(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    GroundSurfaceModel() : controlPointsDataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & frameId;
        message & timestamp;

        message & controlPointsBitsPerPixel;
        message & controlPointsWidth;
        message & controlPointsHeight;

        message & xyCellOrigin;
        message & xyCellSize;
        message & xyLimit;
        message & minMaxAzimuthAngle;

        message & extrinsics;

        message & quadraticParams;

        const auto imageSize = static_cast<uint32_t> (std::ceil(((double) controlPointsBitsPerPixel / 8.0) * controlPointsWidth * controlPointsHeight));

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(controlPointsDataP, imageSize);

        } else {

            controlPointsDataP = message.peek();
            message.seek(message.tell() + imageSize);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}} // namespaces

#endif
