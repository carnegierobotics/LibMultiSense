/**
 * @file info.hh
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

#include <wire/ImuInfoMessage.hh>
#include <wire/SysDeviceInfoMessage.hh>
#include <wire/SysDeviceModesMessage.hh>
#include <wire/SysNetworkMessage.hh>
#include <wire/VersionResponseMessage.hh>

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense {
namespace legacy {

///
/// @brief Convert a wire DeviceInfo message to our API's DeviceInfo
///
MultiSenseInfo::DeviceInfo convert(const crl::multisense::details::wire::SysDeviceInfo &info);

///
/// @brief Convert our API's DeviceInfo to a wire DeviceInfo message
///
crl::multisense::details::wire::SysDeviceInfo convert(const MultiSenseInfo::DeviceInfo &info, const std::string &key);

///
/// @brief Convert a wire VersionResponse to a API SensorVersion
///
MultiSenseInfo::SensorVersion convert(const crl::multisense::details::wire::VersionResponse &response);

///
/// @brief Convert a wire SysDeviceModes to a API SupportedOperatingMode
///
std::vector<MultiSenseInfo::SupportedOperatingMode> convert(const crl::multisense::details::wire::SysDeviceModes &modes);

///
/// @brief Convert details for a specific imu operating mode to a API ImuInfo::Source
///
MultiSenseInfo::ImuInfo::Source convert(const crl::multisense::details::wire::imu::Details &details);

///
/// @brief Convert a wire ImuInfo to a API ImuInfo
///
MultiSenseInfo::ImuInfo convert(const crl::multisense::details::wire::ImuInfo &modes);

///
/// @brief Convert a wire message into a API NetworkInfo
///
MultiSenseInfo::NetworkInfo convert(const crl::multisense::details::wire::SysNetwork &wire);

///
/// @brief Convert a API NetworkInfo into a wire message
///
crl::multisense::details::wire::SysNetwork convert(const MultiSenseInfo::NetworkInfo &info);

}
}
