/**
 * @file channel.cc
 *
 * Copyright 2013-2026
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
 *   2026-04-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <sstream>

#include "details/amb/channel.hh"
#include "details/amb/info.hh"
#include "details/amb/http.hh"
#include "details/amb/utilities.hh"

#include <MultiSense/MultiSenseTypes.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "multisense-webrtc-client.h"

#include <utility/Exception.hh>

namespace multisense {
namespace amb {

AmbChannel::AmbChannel(const Config &config):
    m_config(config)
{
    if (config.connect_on_initialization && connect(config) != Status::OK)
    {
        CRL_EXCEPTION("Connection to MultiSense failed\n");
    }
}

AmbChannel::~AmbChannel() = default;

Status AmbChannel::start_streams(const std::vector<DataSource> &sources)
{
    (void) sources;
    return Status::UNSUPPORTED;
}

Status AmbChannel::stop_streams(const std::vector<DataSource> &sources)
{
    (void) sources;
    return Status::UNSUPPORTED;
}

void AmbChannel::add_image_frame_callback(std::function<void(const ImageFrame&)> callback)
{
    (void) callback;
    return;
}

void AmbChannel::add_imu_frame_callback(std::function<void(const ImuFrame&)> callback)
{
    (void) callback;
    CRL_DEBUG("IMU callbacks are unsupported for the Ambarella camera");
    return;
}

Status AmbChannel::connect(const Config &config)
{
    m_http_client = std::make_unique<httplib::Client>("https://" + config.ip_address);

    if (!m_http_client)
    {
        return Status::FAILED;
    }

    //
    // TODO (malvarado): Figure out the right way to resolve certs
    //
    m_http_client->enable_server_certificate_verification(false);

    //
    // Update our cached calibration
    //
    if (auto calibration = query_calibration(); calibration)
    {
        m_calibration = std::move(calibration.value());
    }
    else
    {
        CRL_DEBUG("Unable to query the camera's calibration");
        return Status::FAILED;
    }

    //
    // Update our cached info
    //
    if (auto info = query_info(); info)
    {
        m_info = std::move(info.value());
    }
    else
    {
        CRL_DEBUG("Unable to query the camera's device info");
        return Status::FAILED;
    }

    return Status::OK;
};

void AmbChannel::disconnect()
{
    return;
};

std::optional<ImageFrame> AmbChannel::get_next_image_frame()
{
    return std::nullopt;
}

std::optional<ImuFrame> AmbChannel::get_next_imu_frame()
{
    return std::nullopt;
}

MultiSenseConfig AmbChannel::get_config()
{
    return m_multisense_config;
}

Status AmbChannel::set_config(const MultiSenseConfig &config)
{
    (void) config;
    return Status::UNSUPPORTED;
}

StereoCalibration AmbChannel::get_calibration()
{
    return m_calibration;
}

Status AmbChannel::set_calibration(const StereoCalibration &calibration)
{
    (void) calibration;
    return Status::UNSUPPORTED;
}

MultiSenseInfo AmbChannel::get_info()
{
    return m_info;
}

Status AmbChannel::set_device_info(const MultiSenseInfo::DeviceInfo &device_info, const std::string &key)
{
    (void) device_info;
    (void) key;
    return Status::UNSUPPORTED;
}

std::optional<MultiSenseStatus> AmbChannel::get_system_status()
{
    return std::nullopt;
}

Status AmbChannel::set_network_config(const MultiSenseInfo::NetworkInfo &config,
                              const std::optional<std::string> &broadcast_interface)
{
    (void) config;
    (void) broadcast_interface;
    return Status::UNSUPPORTED;
}

std::optional<StereoCalibration> AmbChannel::query_calibration()
{
    const auto config_json = http_get(m_http_client, "/conf/conf.json");

    if (config_json)
    {
        auto intrinsics_yaml = std::stringstream(base64_decode(config_json.value()["calibration"]["intrinsics"].get<std::string>()));
        auto extrinsics_yaml = std::stringstream(base64_decode(config_json.value()["calibration"]["extrinsics"].get<std::string>()));

        const auto intrinsics = parse_yaml(intrinsics_yaml);
        const auto extrinsics = parse_yaml(extrinsics_yaml);

        StereoCalibration output;
        std::memcpy(&output.left.K, intrinsics.at("M1").data(), 9 * sizeof(float));
        output.left.D = intrinsics.at("D1");
        std::memcpy(&output.left.R, extrinsics.at("R1").data(), 9 * sizeof(float));
        std::memcpy(&output.left.P, extrinsics.at("P1").data(), 12 * sizeof(float));

        std::memcpy(&output.right.K, intrinsics.at("M2").data(), 9 * sizeof(float));
        output.right.D = intrinsics.at("D2");
        std::memcpy(&output.right.R, extrinsics.at("R2").data(), 9 * sizeof(float));
        std::memcpy(&output.right.P, extrinsics.at("P2").data(), 12 * sizeof(float));

        return output;
    }

    return std::nullopt;
}

std::optional<MultiSenseInfo> AmbChannel::query_info()
{
    const auto info_json = http_get(m_http_client, "/conf/info.json");
    const auto config_json = http_get(m_http_client, "/conf/conf.json");

    if (info_json && config_json)
    {
        const auto device_info_json = info_json.value()["device"];

        MultiSenseInfo::DeviceInfo device_info;
        // TODO (malvarado): Handle the rest of this conversion for more complex types
        device_info.build_date = device_info_json["builddate"].get<std::string>();
        device_info.serial_number = device_info_json["serialnum"].get<std::string>();
        device_info.imager_name = device_info_json["imagername"].get<std::string>();
        device_info.imager_width = device_info_json["imagerwidth"].get<uint32_t>();
        device_info.imager_height = device_info_json["imagerheight"].get<uint32_t>();
        device_info.lens_name = device_info_json["lensname"].get<std::string>();
        //device_info.nominal_stereo_baseline = device_info_json["nominalbaseline"];
        //device_info.nominal_focal_length = device_info_json["nominalfocallength"];
        //device_info.nominal_relative_aperture = device_info_json["nominalrelativeaperture"];
        device_info.number_of_lights = device_info_json["lightingnumber"].get<uint32_t>();

        MultiSenseInfo::NetworkInfo network_info;
        network_info.ip_address = config_json.value()["network"]["ip-address"].get<std::string>();
        network_info.netmask = cidr_to_netmask(config_json.value()["network"]["netmask"].get<uint32_t>());

        MultiSenseInfo::SensorVersion version_info;
        std::vector<MultiSenseInfo::SupportedOperatingMode> operating_modes;

        MultiSenseInfo info{std::move(device_info),
                            std::move(version_info),
                            std::move(operating_modes),
                            std::nullopt,
                            std::move(network_info)};
        return info;
    }


    return std::nullopt;
}

}
}
