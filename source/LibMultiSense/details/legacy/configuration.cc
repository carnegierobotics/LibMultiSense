/**
 * @file configuration.cc
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

#include <algorithm>

#include "details/legacy/configuration.hh"
#include "details/legacy/utilities.hh"

namespace multisense {
namespace legacy {

MultiSenseConfig convert(const crl::multisense::details::wire::CamConfig &config,
                         const std::optional<crl::multisense::details::wire::AuxCamConfig> &aux_config,
                         const std::optional<crl::multisense::details::wire::ImuConfig> &imu_config,
                         const std::optional<crl::multisense::details::wire::LedStatus> &led_config,
                         const crl::multisense::details::wire::SysPacketDelay &packet_delay,
                         bool ptp_enabled,
                         const MultiSenseInfo::DeviceInfo &info,
                         const std::optional<MultiSenseInfo::ImuInfo> &imu_info)
{
    using namespace crl::multisense::details;

    using ms_config = MultiSenseConfig;

    ms_config::StereoConfig stereo{config.stereoPostFilterStrength};

    ms_config::ManualExposureConfig manual_exposure{config.gain,
                                                           std::chrono::microseconds{config.exposure}};

    ms_config::AutoExposureRoiConfig auto_exposure_roi{config.autoExposureRoiX,
                                                              config.autoExposureRoiY,
                                                              config.autoExposureRoiWidth,
                                                              config.autoExposureRoiHeight};

    ms_config::AutoExposureConfig auto_exposure{std::chrono::microseconds{config.autoExposureMax},
                                                       config.autoExposureDecay,
                                                       config.autoExposureTargetIntensity,
                                                       config.autoExposureThresh,
                                                       config.gainMax,
                                                       std::move(auto_exposure_roi)};

    ms_config::ManualWhiteBalanceConfig manual_white_balance{config.whiteBalanceRed, config.whiteBalanceBlue};
    ms_config::AutoWhiteBalanceConfig auto_white_balance{config.autoWhiteBalanceDecay,
                                                                config.autoWhiteBalanceThresh};

    ms_config::ImageConfig image{config.gamma,
                                        (config.autoExposure != 0),
                                        std::move(manual_exposure),
                                        std::move(auto_exposure),
                                        (config.autoWhiteBalance != 0),
                                        std::move(manual_white_balance),
                                        std::move(auto_white_balance)};

    return MultiSenseConfig{config.width,
                            config.height,
                            get_disparities(config.disparities),
                            config.framesPerSecond,
                            std::move(stereo),
                            std::move(image),
                            (aux_config ? std::make_optional(convert(aux_config.value())) : std::nullopt),
                            ms_config::TimeConfig{ptp_enabled},
                            convert(packet_delay),
                            (imu_config && imu_info) ?
                                std::make_optional(convert(imu_config.value(), imu_info.value())) :
                                std::nullopt,
                            (led_config && led_config->available) ?
                                std::make_optional(convert(led_config.value(), info.lighting_type)) :
                                std::nullopt};
}

MultiSenseConfig::AuxConfig convert(const crl::multisense::details::wire::AuxCamConfig &config)
{
    using namespace crl::multisense::details;
    using ms_config = MultiSenseConfig;

    ms_config::ManualExposureConfig manual_exposure{config.gain,
                                                           std::chrono::microseconds{config.exposure}};

    ms_config::AutoExposureRoiConfig auto_exposure_roi{config.autoExposureRoiX,
                                                              config.autoExposureRoiY,
                                                              config.autoExposureRoiWidth,
                                                              config.autoExposureRoiHeight};

    ms_config::AutoExposureConfig auto_exposure{std::chrono::microseconds{config.autoExposureMax},
                                                       config.autoExposureDecay,
                                                       config.autoExposureTargetIntensity,
                                                       config.autoExposureThresh,
                                                       config.gainMax,
                                                       std::move(auto_exposure_roi)};

    ms_config::ManualWhiteBalanceConfig manual_white_balance{config.whiteBalanceRed, config.whiteBalanceBlue};
    ms_config::AutoWhiteBalanceConfig auto_white_balance{config.autoWhiteBalanceDecay,
                                                                config.autoWhiteBalanceThresh};

    ms_config::ImageConfig image{config.gamma,
                                        (config.autoExposure != 0),
                                        std::move(manual_exposure),
                                        std::move(auto_exposure),
                                        (config.autoWhiteBalance != 0),
                                        std::move(manual_white_balance),
                                        std::move(auto_white_balance)};

    return ms_config::AuxConfig{std::move(image),
                                       config.sharpeningEnable,
                                       config.sharpeningPercentage,
                                       config.sharpeningLimit};
}

template <>
crl::multisense::details::wire::CamSetResolution convert<crl::multisense::details::wire::CamSetResolution>(const MultiSenseConfig &config)
{
    using namespace crl::multisense::details;

    int32_t disparities = 256;

    switch(config.disparities)
    {
        case MultiSenseConfig::MaxDisparities::D64: {disparities = 64; break;}
        case MultiSenseConfig::MaxDisparities::D128: {disparities = 128; break;}
        case MultiSenseConfig::MaxDisparities::D256: {disparities = 256; break;}
    }

    return wire::CamSetResolution{config.width, config.height, disparities};
}

template <>
crl::multisense::details::wire::CamControl convert<crl::multisense::details::wire::CamControl>(const MultiSenseConfig &config)
{
    using namespace crl::multisense::details;

    wire::CamControl output;

    output.framesPerSecond = config.frames_per_second;

    const auto manual_exposure = config.image_config.manual_exposure ? config.image_config.manual_exposure.value() :
                                                                   MultiSenseConfig::ManualExposureConfig{};

    output.gain = manual_exposure.gain;
    output.exposure = static_cast<uint32_t>(manual_exposure.exposure_time.count());

    output.autoExposure = config.image_config.auto_exposure_enabled;

    const auto auto_exposure = config.image_config.auto_exposure ? config.image_config.auto_exposure.value() :
                                                                   MultiSenseConfig::AutoExposureConfig{};

    output.autoExposureMax = static_cast<uint32_t>(auto_exposure.max_exposure_time.count());
    output.autoExposureDecay = auto_exposure.decay;
    output.autoExposureThresh = auto_exposure.target_threshold;
    output.autoExposureTargetIntensity = auto_exposure.target_intensity;
    output.gainMax = auto_exposure.max_gain;

    output.autoExposureRoiX = auto_exposure.roi.top_left_x_position;
    output.autoExposureRoiY = auto_exposure.roi.top_left_y_position;
    output.autoExposureRoiWidth = auto_exposure.roi.width;
    output.autoExposureRoiHeight = auto_exposure.roi.height;


    const auto manual_wb = config.image_config.manual_white_balance ? config.image_config.manual_white_balance.value() :
                                                                   MultiSenseConfig::ManualWhiteBalanceConfig{};

    output.whiteBalanceRed = manual_wb.red;
    output.whiteBalanceBlue = manual_wb.blue;

    const auto auto_wb = config.image_config.auto_white_balance ? config.image_config.auto_white_balance.value() :
                                                                   MultiSenseConfig::AutoWhiteBalanceConfig{};

    output.autoWhiteBalance = config.image_config.auto_white_balance_enabled;
    output.autoWhiteBalanceDecay = auto_wb.decay;
    output.autoWhiteBalanceThresh  = auto_wb.threshold;

    output.stereoPostFilterStrength = config.stereo_config.postfilter_strength;

    output.hdrEnabled = false;
    output.gamma = config.image_config.gamma;

    return output;
}

crl::multisense::details::wire::AuxCamControl convert(const MultiSenseConfig::AuxConfig &config)
{
    using namespace crl::multisense::details;

    wire::AuxCamControl output;

    const auto manual_exposure = config.image_config.manual_exposure ? config.image_config.manual_exposure.value() :
                                                                   MultiSenseConfig::ManualExposureConfig{};

    output.gain = manual_exposure.gain;
    output.exposure = static_cast<uint32_t>(manual_exposure.exposure_time.count());

    output.autoExposure = config.image_config.auto_exposure_enabled;

    const auto auto_exposure = config.image_config.auto_exposure ? config.image_config.auto_exposure.value() :
                                                                   MultiSenseConfig::AutoExposureConfig{};

    output.autoExposureMax = static_cast<uint32_t>(auto_exposure.max_exposure_time.count());
    output.autoExposureDecay = auto_exposure.decay;
    output.autoExposureThresh = auto_exposure.target_threshold;
    output.autoExposureTargetIntensity = auto_exposure.target_intensity;
    output.gainMax = auto_exposure.max_gain;

    output.autoExposureRoiX = auto_exposure.roi.top_left_x_position;
    output.autoExposureRoiY = auto_exposure.roi.top_left_y_position;
    output.autoExposureRoiWidth = auto_exposure.roi.width;
    output.autoExposureRoiHeight = auto_exposure.roi.height;


    const auto manual_wb = config.image_config.manual_white_balance ? config.image_config.manual_white_balance.value() :
                                                                      MultiSenseConfig::ManualWhiteBalanceConfig{};

    output.whiteBalanceRed = manual_wb.red;
    output.whiteBalanceBlue = manual_wb.blue;

    const auto auto_wb = config.image_config.auto_white_balance ? config.image_config.auto_white_balance.value() :
                                                                  MultiSenseConfig::AutoWhiteBalanceConfig{};

    output.autoWhiteBalance = config.image_config.auto_white_balance_enabled;
    output.autoWhiteBalanceDecay = auto_wb.decay;
    output.autoWhiteBalanceThresh  = auto_wb.threshold;


    output.hdrEnabled = false;
    output.gamma = config.image_config.gamma;
    output.sharpeningEnable = config.sharpening_enabled;
    output.sharpeningPercentage = config.sharpening_percentage;
    output.sharpeningLimit = config.sharpening_limit;

    //
    // Currently unsupported values
    //
    output.cameraProfile = 0;

    return output;
}

crl::multisense::details::wire::SysSetPtp convert(const MultiSenseConfig::TimeConfig &config)
{
    using namespace crl::multisense::details;

    wire::SysSetPtp output;
    output.enable = config.ptp_enabled ? 1 : 0;

    return output;
}

MultiSenseConfig::ImuConfig convert(const crl::multisense::details::wire::ImuConfig &imu,
                                    const MultiSenseInfo::ImuInfo &imu_info)
{
    using namespace crl::multisense::details;
    using ImuConfig = MultiSenseConfig::ImuConfig;

    std::optional<ImuConfig::OperatingMode> accelerometer = std::nullopt;
    std::optional<ImuConfig::OperatingMode> gyroscope = std::nullopt;
    std::optional<ImuConfig::OperatingMode> magnetometer = std::nullopt;
    for (const auto &element : imu.configs)
    {
        if (imu_info.accelerometer && element.name == imu_info.accelerometer->name)
        {
            accelerometer = ImuConfig::OperatingMode{static_cast<bool>(element.flags & wire::imu::Config::FLAGS_ENABLED),
                                                     imu_info.accelerometer->rates[element.rateTableIndex],
                                                     imu_info.accelerometer->ranges[element.rangeTableIndex]};
        }
        else if (imu_info.gyroscope && element.name == imu_info.gyroscope->name)
        {
            gyroscope = ImuConfig::OperatingMode{static_cast<bool>(element.flags & wire::imu::Config::FLAGS_ENABLED),
                                                 imu_info.gyroscope->rates[element.rateTableIndex],
                                                 imu_info.gyroscope->ranges[element.rangeTableIndex]};
        }
        else if (imu_info.magnetometer && element.name == imu_info.magnetometer->name)
        {
            magnetometer = ImuConfig::OperatingMode{static_cast<bool>(element.flags & wire::imu::Config::FLAGS_ENABLED),
                                                     imu_info.magnetometer->rates[element.rateTableIndex],
                                                     imu_info.magnetometer->ranges[element.rangeTableIndex]};
        }
        else
        {
            CRL_EXCEPTION("Unknown imu name: %s\n", element.name.c_str());
        }
    }

    return ImuConfig{imu.samplesPerMessage, std::move(accelerometer), std::move(gyroscope), std::move(magnetometer)};
}

crl::multisense::details::wire::ImuConfig convert(const MultiSenseConfig::ImuConfig &imu,
                                                  const MultiSenseInfo::ImuInfo &imu_info,
                                                  uint32_t max_samples_per_message)
{
    using namespace crl::multisense::details;

    wire::ImuConfig output;

    output.samplesPerMessage = std::min(max_samples_per_message, imu.samples_per_frame);

    std::vector<wire::imu::Config> configs;

    if (imu.accelerometer && imu_info.accelerometer)
    {
        wire::imu::Config config;
        config.name = imu_info.accelerometer->name;
        config.flags = imu.accelerometer->enabled ? wire::imu::Config::FLAGS_ENABLED : 0;

        config.rateTableIndex = get_rate_index(imu_info.accelerometer->rates, imu.accelerometer->rate);
        config.rangeTableIndex = get_range_index(imu_info.accelerometer->ranges, imu.accelerometer->range);

        configs.emplace_back(std::move(config));
    }

    if (imu.gyroscope && imu_info.gyroscope)
    {
        wire::imu::Config config;
        config.name = imu_info.gyroscope->name;
        config.flags = imu.gyroscope->enabled ? wire::imu::Config::FLAGS_ENABLED : 0;

        config.rateTableIndex = get_rate_index(imu_info.gyroscope->rates, imu.gyroscope->rate);
        config.rangeTableIndex = get_range_index(imu_info.gyroscope->ranges, imu.gyroscope->range);

        configs.emplace_back(std::move(config));
    }

    if (imu.magnetometer && imu_info.magnetometer)
    {
        wire::imu::Config config;
        config.name = imu_info.magnetometer->name;
        config.flags = imu.magnetometer->enabled ? wire::imu::Config::FLAGS_ENABLED : 0;

        config.rateTableIndex = get_rate_index(imu_info.magnetometer->rates, imu.magnetometer->rate);
        config.rangeTableIndex = get_range_index(imu_info.magnetometer->ranges, imu.magnetometer->range);

        configs.emplace_back(std::move(config));
    }

    output.storeSettingsInFlash = false;
    output.configs= std::move(configs);

    return output;
}

MultiSenseConfig::LightingConfig convert(const crl::multisense::details::wire::LedStatus &led,
                                         const MultiSenseInfo::DeviceInfo::LightingType &type)
{
    using lighting = MultiSenseConfig::LightingConfig;

    const auto intensity = static_cast<float>(led.intensity[0]) * 100.0f / 255.0f;

    std::optional<lighting::InternalConfig> internal = std::nullopt;
    std::optional<lighting::ExternalConfig> external = std::nullopt;
    switch (type)
    {
        case MultiSenseInfo::DeviceInfo::LightingType::NONE:
        {
            break;
        }
        case MultiSenseInfo::DeviceInfo::LightingType::INTERNAL:
        case MultiSenseInfo::DeviceInfo::LightingType::PATTERN_PROJECTOR:
        {
            internal = lighting::InternalConfig{intensity, led.flash != 0};
            break;
        }
        case MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL:
        {
            lighting::ExternalConfig::FlashMode mode = lighting::ExternalConfig::FlashMode::NONE;

            if (led.rolling_shutter_led)
            {
                mode = lighting::ExternalConfig::FlashMode::SYNC_WITH_AUX;
            }
            else if (led.flash)
            {
                mode = lighting::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO;
            }

            external = lighting::ExternalConfig{intensity, mode, led.number_of_pulses, std::chrono::microseconds{led.led_delay_us}};
            break;
        }
        default: {CRL_EXCEPTION("Unsupported lighting type\n");}
    }

    return MultiSenseConfig::LightingConfig{std::move(internal), std::move(external)};
}

crl::multisense::details::wire::LedSet convert(const MultiSenseConfig::LightingConfig &led)
{
    using namespace crl::multisense::details;

    if (!led.internal && !led.external)
    {
        CRL_EXCEPTION("Invalid lighting config input\n");
    }

    wire::LedSet output;

    if (led.internal)
    {
        for(size_t i = 0; i< wire::MAX_LIGHTS; ++i)
        {
            output.mask |= (1<<i);
            output.intensity[i] = static_cast<uint8_t> (255.0f * (std::clamp(led.internal->intensity, 0.0f, 100.0f) / 100.0f));
        }

        output.flash = led.internal->flash ? 1 : 0;

        output.number_of_pulses = 1;
        output.invert_pulse = 0;
        output.led_delay_us = 0;
        output.rolling_shutter_led = false;
    }
    else if (led.external)
    {
        for(size_t i = 0; i< wire::MAX_LIGHTS; ++i)
        {
            output.mask |= (1<<i);
            output.intensity[i] = static_cast<uint8_t> (255.0f * (std::clamp(led.external->intensity, 0.0f, 100.0f) / 100.0f));
        }

        switch (led.external->flash)
        {
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::NONE:
            {
                output.flash = 0;
                output.rolling_shutter_led = 0;
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO:
            {
                output.flash = 1;
                output.rolling_shutter_led = 0;
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_AUX:
            {
                output.flash = 1;
                output.rolling_shutter_led = 1;
                break;
            }
            default:
            {
                CRL_EXCEPTION("Unhandled LED flash mode\n");
            }
        }

        output.number_of_pulses = led.external->pulses_per_exposure;
        output.invert_pulse = false;
        output.led_delay_us = static_cast<uint32_t>(led.external->startup_time.count());
    }

    return output;
}

MultiSenseConfig::NetworkTransmissionConfig
    convert(const crl::multisense::details::wire::SysPacketDelay &packet)
{
    return MultiSenseConfig::NetworkTransmissionConfig{ packet.enable};
}

template <>
crl::multisense::details::wire::SysPacketDelay convert(const MultiSenseConfig::NetworkTransmissionConfig &config)
{
    using namespace crl::multisense::details;

    wire::SysPacketDelay delay;

    delay.enable = config.packet_delay_enabled;

    return delay;
}

}
}
