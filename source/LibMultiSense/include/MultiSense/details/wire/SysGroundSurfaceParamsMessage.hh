/**
 * @file LibMultiSense/SysGroundSurfaceParamsMessage.hh
 *
 * This message contains general device information
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
 *   2022-02-08, drobinson@carnegierobotics.com, IRAD, created file.
 **/

#ifndef LibMultiSense_SysGroundSurfaceParamsMessage
#define LibMultiSense_SysGroundSurfaceParamsMessage

#include <algorithm>
#include <string>
#include "Protocol.hh"
#include "../utility/BufferStream.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysGroundSurfaceParams {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_SYS_GROUND_SURFACE_PARAM;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    int ground_surface_number_of_levels_x;
    int ground_surface_number_of_levels_z;
    int ground_surface_base_model;
    float ground_surface_pointcloud_grid_size;
    int ground_surface_min_points_per_grid;
    int ground_surface_pointcloud_decimation;
    float ground_surface_pointcloud_max_range_m;
    float ground_surface_pointcloud_min_range_m;
    float ground_surface_pointcloud_max_width_m;
    float ground_surface_pointcloud_min_width_m;
    float ground_surface_pointcloud_max_height_m;
    float ground_surface_pointcloud_min_height_m;
    float ground_surface_obstacle_height_thresh_m;
    float ground_surface_obstacle_percentage_thresh;
    int ground_surface_max_fitting_iterations;
    float ground_surface_adjacent_cell_search_size_m;

    //
    // Constructors
    //
    SysGroundSurfaceParams(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysGroundSurfaceParams()
    {
        ground_surface_number_of_levels_x = 4;
        ground_surface_number_of_levels_z = 4;
        ground_surface_base_model = 1;
        ground_surface_pointcloud_grid_size = 0.5;
        ground_surface_min_points_per_grid = 10;
        ground_surface_pointcloud_decimation = 1;
        ground_surface_pointcloud_max_range_m = 30.0;
        ground_surface_pointcloud_min_range_m = 0.5;
        ground_surface_pointcloud_max_width_m = 25.0;
        ground_surface_pointcloud_min_width_m = -25.0;
        ground_surface_pointcloud_max_height_m = 10.0;
        ground_surface_pointcloud_min_height_m = -10.0;
        ground_surface_obstacle_height_thresh_m = 2.0;
        ground_surface_obstacle_percentage_thresh = 0.5;
        ground_surface_max_fitting_iterations = 10;
        ground_surface_adjacent_cell_search_size_m = 1.5;
    };

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & ground_surface_number_of_levels_x;
        message & ground_surface_number_of_levels_z;
        message & ground_surface_base_model;
        message & ground_surface_pointcloud_grid_size;
        message & ground_surface_min_points_per_grid;
        message & ground_surface_pointcloud_decimation;
        message & ground_surface_pointcloud_max_range_m;
        message & ground_surface_pointcloud_min_range_m;
        message & ground_surface_pointcloud_max_width_m;
        message & ground_surface_pointcloud_min_width_m;
        message & ground_surface_pointcloud_max_height_m;
        message & ground_surface_pointcloud_min_height_m;
        message & ground_surface_obstacle_height_thresh_m;
        message & ground_surface_obstacle_percentage_thresh;
        message & ground_surface_max_fitting_iterations;
        message & ground_surface_adjacent_cell_search_size_m;
    }

};

}}}} // namespaces

#endif
