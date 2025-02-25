/**
 * @file calibration.hh
 *
 * Copyright 2013-2025
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
 *   2025-01-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <utility/Exception.hh>
#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>

#include <wire/SysCameraCalibrationMessage.hh>

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense {
namespace legacy {

///
/// @brief Check if the CameraCalData object is valid
///
bool is_valid(const crl::multisense::details::wire::CameraCalData &cal);

///
/// @brief Convert a wire calibration to our API calibration object
///
CameraCalibration convert(const crl::multisense::details::wire::CameraCalData &cal);

///
/// @brief Convert our API calibration object to a wire message
///
crl::multisense::details::wire::CameraCalData convert(const CameraCalibration &cal);

///
/// @brief Convert a wire calibration to our API calibration object
///
StereoCalibration convert(const crl::multisense::details::wire::SysCameraCalibration &cal);

///
/// @brief Convert our API calibration object to a wire calibration
///
crl::multisense::details::wire::SysCameraCalibration convert(const StereoCalibration &cal);

///
/// @brief Get the correct calibration corresponding to the input source
///
CameraCalibration select_calibration(const StereoCalibration &input, const DataSource &source);

///
/// @brief Scale a calibration used to update a full-res calibration based on the current operating resolution
///
CameraCalibration scale_calibration(const CameraCalibration &input, double x_scale, double y_scale);

///
/// @brief Scale a calibration used to update a full-res calibration based on the current operating resolution
///
StereoCalibration scale_calibration(const StereoCalibration &input, double x_scale, double y_scale);


}
}
