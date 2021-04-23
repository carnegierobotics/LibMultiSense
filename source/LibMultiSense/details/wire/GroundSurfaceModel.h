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

class WIRE_HEADER_ATTRIBS_ GroundSurfaceModelHeader {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_GROUND_SURFACE_SPLINE_DATA_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    //
    // Frame ID that the algorithm was processed on

    int64_t     frameId;

    //
    // Control points dynamic array (traded as image) members

    uint32_t    controlPointsBitsPerPixel;
    uint32_t    controlPointsWidth;
    uint32_t    controlPointsHeight;

    //
    // Spline details

    float xyCellOrigin_x;
    float xyCellOrigin_y;
    float xyCellSize_x;
    float xyCellSize_y;

    //
    // Extrinsics that were used during the computation of the spline

    float extrinsics_x_m;
    float extrinsics_y_m;
    float extrinsics_z_m;
    float extrinsics_rx_rad;
    float extrinsics_ry_rad;
    float extrinsics_rz_rad;

    //
    // Boundaries

    float boundary_max_x;
    float boundary_min_x;
    float boundary_max_y;
    float boundary_min_y;
    float boundary_max_azimuth_angle;
    float boundary_min_azimuth_angle;

    //
    // Ground Base Model (ax^2 + by^2 + cxy + dx + ey + f)

    std::array<float, 6> quadratic_params;

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
        xyCellOrigin_x(0.0f),
        xyCellOrigin_y(0.0f),
        xyCellSize_x(0.0f),
        xyCellSize_y(0.0f),
        extrinsics_x_m(0.0f),
        extrinsics_y_m(0.0f),
        extrinsics_z_m(0.0f),
        extrinsics_rx_rad(0.0f),
        extrinsics_ry_rad(0.0f),
        extrinsics_rz_rad(0.0f),
        boundary_max_x(0.0f),
        boundary_min_x(0.0f),
        boundary_max_y(0.0f),
        boundary_min_y(0.0f),
        boundary_max_azimuth_angle(0.0f),
        boundary_min_azimuth_angle(0.0f)
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
        message & controlPointsBitsPerPixel;
        message & controlPointsWidth;
        message & controlPointsHeight;

        message & xyCellOrigin_x;
        message & xyCellOrigin_y;
        message & xyCellSize_x;
        message & xyCellSize_y;

        message & extrinsics_x_m;
        message & extrinsics_y_m;
        message & extrinsics_z_m;
        message & extrinsics_rx_rad;
        message & extrinsics_ry_rad;
        message & extrinsics_rz_rad;

        message & boundary_max_x;
        message & boundary_min_x;
        message & boundary_max_y;
        message & boundary_min_y;
        message & boundary_max_azimuth_angle;
        message & boundary_min_azimuth_angle;

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
