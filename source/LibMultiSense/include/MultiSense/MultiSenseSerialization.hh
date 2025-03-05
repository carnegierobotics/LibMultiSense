/**
 * @file MultiSenseSerialization.hh
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
 *   2024-02-10, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <nlohmann/json.hpp>

#include "MultiSenseChannel.hh"
#include "MultiSenseTypes.hh"

namespace nlohmann
{
    template <>
    struct adl_serializer<std::chrono::microseconds>
    {
        static void to_json(json& j, const std::chrono::microseconds &d)
        {
            j = d.count();
        }

        static void from_json(const json& j, std::chrono::microseconds &d)
        {
            d = std::chrono::microseconds(j.get<std::chrono::microseconds::rep>());
        }
    };

    template <>
    struct adl_serializer<std::chrono::milliseconds>
    {
        static void to_json(json& j, const std::chrono::milliseconds &d)
        {
            j = d.count();
        }

        static void from_json(const json& j, std::chrono::milliseconds &d)
        {
            d = std::chrono::milliseconds(j.get<std::chrono::milliseconds::rep>());
        }
    };

    template <>
    struct adl_serializer<std::chrono::nanoseconds>
    {
        static void to_json(json& j, const std::chrono::nanoseconds &d)
        {
            j = d.count();
        }

        static void from_json(const json& j, std::chrono::nanoseconds &d)
        {
            d = std::chrono::nanoseconds(j.get<std::chrono::nanoseconds::rep>());
        }
    };

    ///
    /// @brief Handle generic optional
    ///
    template <typename T>
    struct adl_serializer<std::optional<T>>
    {
        static void to_json(json& j, const std::optional<T> &opt)
        {
            if (opt.has_value())
            {
                j = *opt;
            }
            else
            {
                j = nullptr;
            }
        }

        static void from_json(const json& j, std::optional<T> &opt)
        {
            if (j.is_null())
            {
                opt = std::nullopt;
            }
            else
            {
                T v{};
                j.get_to(v);
                opt = std::move(v);
            }
        }
    };

}

namespace multisense
{

NLOHMANN_JSON_SERIALIZE_ENUM(Status, {
    {Status::UNKNOWN, "UNKNOWN"},
    {Status::OK, "OK"},
    {Status::TIMEOUT, "TIMEOUT"},
    {Status::INTERNAL_ERROR, "INTERNAL_ERROR"},
    {Status::FAILED, "FAILED"},
    {Status::UNSUPPORTED, "UNSUPPORTED"},
    {Status::EXCEPTION, "EXCEPTION"},
    {Status::UNINITIALIZED, "UNINITIALIZED"},
    {Status::INCOMPLETE_APPLICATION, "INCOMPLETE_APPLICATION"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(DataSource, {
    {DataSource::UNKNOWN, "UNKNOWN"},
    {DataSource::ALL, "ALL"},
    {DataSource::LEFT_MONO_RAW, "LEFT_MONO_RAW"},
    {DataSource::RIGHT_MONO_RAW, "RIGHT_MONO_RAW"},
    {DataSource::LEFT_MONO_COMPRESSED, "LEFT_MONO_COMPRESSED"},
    {DataSource::RIGHT_MONO_COMPRESSED, "RIGHT_MONO_COMPRESSED"},
    {DataSource::LEFT_RECTIFIED_RAW, "LEFT_RECTIFIED_RAW"},
    {DataSource::RIGHT_RECTIFIED_RAW, "RIGHT_RECTIFIED_RAW"},
    {DataSource::LEFT_RECTIFIED_COMPRESSED, "LEFT_RECTIFIED_COMPRESSED"},
    {DataSource::RIGHT_RECTIFIED_COMPRESSED, "RIGHT_RECTIFIED_COMPRESSED"},
    {DataSource::LEFT_DISPARITY_RAW, "LEFT_DISPARITY_RAW"},
    {DataSource::LEFT_DISPARITY_COMPRESSED, "LEFT_DISPARITY_COMPRESSED"},
    {DataSource::AUX_COMPRESSED, "AUX_COMPRESSED"},
    {DataSource::AUX_RECTIFIED_COMPRESSED, "AUX_RECTIFIED_COMPRESSED"},
    {DataSource::AUX_LUMA_RAW, "AUX_LUMA_RAW"},
    {DataSource::AUX_LUMA_RECTIFIED_RAW, "AUX_LUMA_RECTIFIED_RAW"},
    {DataSource::AUX_CHROMA_RAW, "AUX_CHROMA_RAW"},
    {DataSource::AUX_CHROMA_RECTIFIED_RAW, "AUX_CHROMA_RECTIFIED_RAW"},
    {DataSource::AUX_RAW, "AUX_RAW"},
    {DataSource::AUX_RECTIFIED_RAW, "AUX_RECTIFIED_RAW"},
    {DataSource::COST_RAW, "COST_RAW"},
    {DataSource::IMU, "IMU"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(CameraCalibration::DistortionType, {
    {CameraCalibration::DistortionType::NONE, "NONE"},
    {CameraCalibration::DistortionType::PLUMBBOB, "PLUMBBOB"},
    {CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL, "RATIONAL_POLYNOMIAL"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseConfig::MaxDisparities, {
    {MultiSenseConfig::MaxDisparities::D64, "D64"},
    {MultiSenseConfig::MaxDisparities::D128, "D128"},
    {MultiSenseConfig::MaxDisparities::D256, "D256"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode, {
    {MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::NONE, "NONE"},
    {MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO, "SYNC_WITH_MAIN_STEREO"},
    {MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_AUX, "SYNC_WITH_AUX"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseInfo::DeviceInfo::HardwareRevision, {
    {MultiSenseInfo::DeviceInfo::HardwareRevision::UNKNOWN, "UNKNOWN"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::S7, "S7"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::S21, "S21"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::ST21, "ST21"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::S27, "S27"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::S30, "S30"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::KS21, "KS21"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::MONOCAM, "MONOCAM"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::KS21_SILVER, "KS21_SILVER"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::ST25, "ST25"},
    {MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i, "KS21i"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseInfo::DeviceInfo::ImagerType, {
    {MultiSenseInfo::DeviceInfo::ImagerType::UNKNOWN, "UNKNOWN"},
    {MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_GREY, "CMV2000_GREY"},
    {MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_COLOR, "CMV2000_COLOR"},
    {MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_GREY, "CMV4000_GREY"},
    {MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_COLOR, "CMV4000_COLOR"},
    {MultiSenseInfo::DeviceInfo::ImagerType::FLIR_TAU2, "FLIR_TAU2"},
    {MultiSenseInfo::DeviceInfo::ImagerType::AR0234_GREY, "AR0234_GREY"},
    {MultiSenseInfo::DeviceInfo::ImagerType::AR0239_COLOR, "AR0239_COLOR"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseInfo::DeviceInfo::LightingType, {
    {MultiSenseInfo::DeviceInfo::LightingType::NONE, "NONE"},
    {MultiSenseInfo::DeviceInfo::LightingType::INTERNAL, "INTERNAL"},
    {MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL, "EXTERNAL"},
    {MultiSenseInfo::DeviceInfo::LightingType::PATTERN_PROJECTOR, "PATTERN_PROJECTOR"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(MultiSenseInfo::DeviceInfo::LensType, {
    {MultiSenseInfo::DeviceInfo::LensType::UNKNOWN, "UNKNOWN"},
    {MultiSenseInfo::DeviceInfo::LensType::STANDARD, "STANDARD"},
    {MultiSenseInfo::DeviceInfo::LensType::FISHEYE, "FISHEYE"}
})

NLOHMANN_JSON_SERIALIZE_ENUM(Channel::ChannelImplementation, {
    {Channel::ChannelImplementation::LEGACY, "LEGACY"}
})

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CameraCalibration,
                                   K,
                                   R,
                                   P,
                                   distortion_type,
                                   D)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StereoCalibration,
                                   left,
                                   right,
                                   aux)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::StereoConfig,
                                   postfilter_strength)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::ManualExposureConfig,
                                   gain,
                                   exposure_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::AutoExposureRoiConfig,
                                   top_left_x_position,
                                   top_left_y_position,
                                   width,
                                   height)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::AutoExposureConfig,
                                   max_exposure_time,
                                   decay,
                                   target_intensity,
                                   target_threshold,
                                   max_gain,
                                   roi)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::ManualWhiteBalanceConfig,
                                   red,
                                   blue)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::AutoWhiteBalanceConfig,
                                   decay,
                                   threshold)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::ImageConfig,
                                   gamma,
                                   auto_exposure_enabled,
                                   manual_exposure,
                                   auto_exposure,
                                   auto_white_balance_enabled,
                                   manual_white_balance,
                                   auto_white_balance)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::AuxConfig,
                                   image_config,
                                   sharpening_enabled,
                                   sharpening_percentage,
                                   sharpening_limit)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::TimeConfig,
                                   ptp_enabled)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::NetworkTransmissionConfig,
                                   packet_delay_enabled)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuRate,
                                   sample_rate,
                                   bandwith_cutoff)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuRange,
                                   range,
                                   resolution)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::ImuConfig::OperatingMode,
                                   enabled,
                                   rate,
                                   range)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::LightingConfig::InternalConfig,
                                   intensity,
                                   flash)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::LightingConfig::ExternalConfig,
                                   intensity,
                                   flash,
                                   pulses_per_exposure,
                                   startup_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::LightingConfig,
                                   internal,
                                   external)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig::ImuConfig,
                                   samples_per_frame,
                                   accelerometer,
                                   gyroscope,
                                   magnetometer)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseConfig,
                                   width,
                                   height,
                                   disparities,
                                   frames_per_second,
                                   stereo_config,
                                   image_config,
                                   aux_config,
                                   time_config,
                                   network_config,
                                   imu_config,
                                   lighting_config)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::PtpStatus,
                                   grandmaster_present,
                                   grandmaster_id,
                                   grandmaster_offset,
                                   path_delay,
                                   steps_from_local_to_grandmaster)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::CameraStatus,
                                   cameras_ok,
                                   processing_pipeline_ok)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::TemperatureStatus,
                                   fpga_temperature,
                                   left_imager_temperature,
                                   right_imager_temperature,
                                   power_supply_temperature)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::PowerStatus,
                                   input_voltage,
                                   input_current,
                                   fpga_power)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::ClientNetworkStatus,
                                   received_messages,
                                   dropped_messages,
                                   invalid_packets)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus::TimeStatus,
                                   camera_time,
                                   client_host_time,
                                   network_delay)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseStatus,
                                   system_ok,
                                   ptp,
                                   camera,
                                   temperature,
                                   power,
                                   client_network,
                                   time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::NetworkInfo,
                                   ip_address,
                                   gateway,
                                   netmask)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::DeviceInfo::PcbInfo,
                                   name,
                                   revision)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::DeviceInfo,
                                   camera_name,
                                   build_date,
                                   serial_number,
                                   hardware_revision,
                                   pcb_info,
                                   imager_name,
                                   imager_type,
                                   imager_width,
                                   imager_height,
                                   lens_name,
                                   lens_type,
                                   nominal_stereo_baseline,
                                   nominal_focal_length,
                                   nominal_relative_aperture,
                                   lens_type,
                                   number_of_lights)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::Version,
                                   major,
                                   minor,
                                   patch)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::SensorVersion,
                                   firmware_build_date,
                                   firmware_version,
                                   hardware_version)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::SupportedOperatingMode,
                                   width,
                                   height,
                                   disparities,
                                   supported_sources)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::ImuInfo::Source,
                                   name,
                                   device,
                                   rates,
                                   ranges)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo::ImuInfo,
                                   accelerometer,
                                   gyroscope,
                                   magnetometer)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultiSenseInfo,
                                   device,
                                   version,
                                   operating_modes,
                                   imu,
                                   network)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Channel::ReceiveBufferConfig,
                                   num_small_buffers,
                                   small_buffer_size,
                                   num_large_buffers,
                                   large_buffer_size)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Channel::Config,
                                   ip_address,
                                   mtu,
                                   receive_timeout,
                                   command_port,
                                   interface,
                                   receive_buffer_configuration)

}
