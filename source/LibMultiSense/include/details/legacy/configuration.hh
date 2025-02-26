/**
 * @file configuration.hh
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
 *   2025-01-18, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <utility/Exception.hh>
#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>

#include <wire/AuxCamConfigMessage.hh>
#include <wire/AuxCamControlMessage.hh>
#include <wire/AuxCamGetConfigMessage.hh>
#include <wire/CamConfigMessage.hh>
#include <wire/CamControlMessage.hh>
#include <wire/CamGetConfigMessage.hh>
#include <wire/CamSetResolutionMessage.hh>
#include <wire/ImuConfigMessage.hh>
#include <wire/LedStatusMessage.hh>
#include <wire/LedSetMessage.hh>
#include <wire/SysPacketDelayMessage.hh>
#include <wire/SysSetPtpMessage.hh>

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense {
namespace legacy {

///
/// @brief Convert a wire AuxCameraConfiuration to our AuxConfig API
///
MultiSenseConfig::AuxConfig convert(const crl::multisense::details::wire::AuxCamConfig &aux_config);

///
/// @brief Convert wire objects to our MultiSenseConfig
///
MultiSenseConfig convert(const crl::multisense::details::wire::CamConfig &config,
                         const std::optional<crl::multisense::details::wire::AuxCamConfig> &aux_config,
                         const std::optional<crl::multisense::details::wire::ImuConfig> &imu_config,
                         const std::optional<crl::multisense::details::wire::LedStatus> &led_config,
                         const crl::multisense::details::wire::SysPacketDelay &packet_delay,
                         bool ptp_enabled,
                         const MultiSenseInfo::DeviceInfo &info,
                         const std::optional<MultiSenseInfo::ImuInfo> &imu_info);

///
/// @brief Generic conversions between the MultiSenseConfig config object an MultiSense wire types
///
template <typename T>
T convert(const MultiSenseConfig &config);

///
/// @brief Convert a AuxConfig object to a wire AuxControl message
///
crl::multisense::details::wire::AuxCamControl convert(const MultiSenseConfig::AuxConfig &config);

///
/// @brief Convert a TimeConfig object to a wire message
///
crl::multisense::details::wire::SysSetPtp convert(const MultiSenseConfig::TimeConfig &config);

///
/// @brief Convert a ImuConfig object to a wire ImuConfig message
///
MultiSenseConfig::ImuConfig convert(const crl::multisense::details::wire::ImuConfig &imu,
                                    const MultiSenseInfo::ImuInfo &imu_info);

///
/// @brief Convert a ImuConfig object to a wire message
///
crl::multisense::details::wire::ImuConfig convert(const MultiSenseConfig::ImuConfig &imu,
                                                  const MultiSenseInfo::ImuInfo &imu_info,
                                                  uint32_t max_samples_per_message);

///
/// @brief Convert a wire lighting config to a API lighting config
///
MultiSenseConfig::LightingConfig convert(const crl::multisense::details::wire::LedStatus &led,
                                         const MultiSenseInfo::DeviceInfo::LightingType &type);

///
/// @brief Convert a API lighting config to a wire lighting config
///
crl::multisense::details::wire::LedSet convert (const MultiSenseConfig::LightingConfig &led);

///
/// @brief Create a API transmission config object from wire types
///
MultiSenseConfig::NetworkTransmissionConfig
    convert(const crl::multisense::details::wire::SysPacketDelay &packet);

///
/// @brief Generic conversions between the MultiSenseConfig::NetworkTransmissionConfig config
///        object an MultiSense wire types
///
template <typename T>
T convert(const MultiSenseConfig::NetworkTransmissionConfig &config);

}
}
