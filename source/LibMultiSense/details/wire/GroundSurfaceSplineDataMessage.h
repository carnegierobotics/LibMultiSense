/**
 * @file LibMultiSense/GroundSurfaceSplineDataMessage.h
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

#ifndef LibMultiSense_GroundSurfaceSplineDataMessage
#define LibMultiSense_GroundSurfaceSplineDataMessage

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class GroundSurfaceSplineDataMessage {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_GROUND_SURFACE_SPLINE_DATA_MESSAGE;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    IdType      id;
    VersionType version;

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

    GroundSurfaceSplineDataMessage(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    GroundSurfaceSplineDataMessage() :
        id(ID),
        version(VERSION),
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

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

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

        message & quadratic_params;
    }
};

}}}} // namespaces

#endif
