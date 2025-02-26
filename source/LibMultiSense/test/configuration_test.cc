/**
 * @file configuration_test.cc
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
 *   2025-01-26, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/configuration.hh>

using namespace multisense::legacy;

multisense::MultiSenseConfig create_valid_config(uint32_t width, uint32_t height)
{
    using namespace multisense;
    using namespace std::chrono_literals;

    MultiSenseConfig::StereoConfig stereo_config{0.8};

    MultiSenseConfig::ManualExposureConfig manual_exposure{2.0, 500us};

    MultiSenseConfig::AutoExposureConfig auto_exposure{600us,
                                                       6,
                                                       0.6,
                                                       0.95,
                                                       3.0,
                                                       {10, 20, 400, 300}};

    MultiSenseConfig::ManualWhiteBalanceConfig manual_white_balance{2.2, 3.3};

    MultiSenseConfig::AutoWhiteBalanceConfig auto_white_balance{4, 0.7};

    MultiSenseConfig::ImageConfig image_config{1.2,
                                               false,
                                               manual_exposure,
                                               auto_exposure,
                                               true,
                                               manual_white_balance,
                                               auto_white_balance};

    MultiSenseConfig::AuxConfig aux_config{image_config, true, 10.0, 80};

    MultiSenseConfig::TimeConfig time{true};

    MultiSenseConfig::NetworkTransmissionConfig network{true};

    return MultiSenseConfig{width,
                            height,
                            MultiSenseConfig::MaxDisparities::D256,
                            11.0,
                            stereo_config,
                            image_config,
                            aux_config,
                            std::make_optional(time),
                            std::make_optional(network),
                            std::nullopt,
                            std::nullopt};
}

multisense::MultiSenseInfo::DeviceInfo create_device_info(uint32_t imager_width, uint32_t imager_height)
{
    multisense::MultiSenseInfo::DeviceInfo info;
    info.imager_width = imager_width;
    info.imager_height = imager_height;

    info.lighting_type = multisense::MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL;

    return info;
}

multisense::MultiSenseInfo::ImuInfo create_imu_info()
{
    using namespace multisense;

    std::vector<ImuRate> accel_rates{{1.0f, 2.0f}, {3.0f, 4.4f}, {5.5f, 6.7f}};
    std::vector<ImuRange> accel_ranges{{1.1f, 3.0f}, {3.1f, 4.5f}, {5.6f, 6.8f}};

    std::vector<ImuRate> gyro_rates{{1.1f, 2.1f}, {3.1f, 5.4f}, {5.6f, 7.7f}};
    std::vector<ImuRange> gyro_ranges{{2.1f, 3.1f}, {4.1f, 5.5f}, {6.6f, 7.8f}};

    std::vector<ImuRate> mag_rates{{2.1f, 3.1f}, {4.1f, 6.4f}, {6.6f, 8.7f}};
    std::vector<ImuRange> mag_ranges{{3.1f, 4.1f}, {5.1f, 5.6f}, {6.7f, 8.8f}};

    return MultiSenseInfo::ImuInfo{MultiSenseInfo::ImuInfo::Source{"accelerometer", "bmi160", accel_rates, accel_ranges},
                                   MultiSenseInfo::ImuInfo::Source{"gyroscope", "bmi160", gyro_rates, gyro_ranges},
                                   MultiSenseInfo::ImuInfo::Source{"magnetometer", "bmi160", mag_rates, mag_ranges}};
}


crl::multisense::details::wire::CamConfig create_valid_wire_config()
{
    using namespace crl::multisense::details;

    wire::CamConfig config;
    config.width = 1920;
    config.height = 1200;
    config.disparities = 256;
    config.stereoPostFilterStrength = 0.8;
    config.gain = 2.0;
    config.exposure = 500;

    config.autoExposure = 0;
    config.autoExposureMax = 600;
    config.autoExposureDecay = 6;
    config.autoExposureTargetIntensity = 0.6;
    config.autoExposureThresh = 0.95;
    config.gainMax = 3.0;
    config.autoExposureRoiX = 10;
    config.autoExposureRoiY = 20;
    config.autoExposureRoiWidth = 400;
    config.autoExposureRoiHeight = 300;

    config.autoWhiteBalance = 1;
    config.whiteBalanceRed = 2.2;
    config.whiteBalanceBlue = 3.3;
    config.autoWhiteBalanceDecay = 4;
    config.autoWhiteBalanceThresh = 0.7;

    config.gamma = 1.2;

    return config;
}

crl::multisense::details::wire::AuxCamConfig create_valid_wire_aux_config()
{
    using namespace crl::multisense::details;

    wire::AuxCamConfig config;
    config.gain = 2.0;
    config.exposure = 500;

    config.autoExposure = 0;
    config.autoExposureMax = 600;
    config.autoExposureDecay = 6;
    config.autoExposureTargetIntensity = 0.6;
    config.autoExposureThresh = 0.95;
    config.gainMax = 3.0;
    config.autoExposureRoiX = 10;
    config.autoExposureRoiY = 20;
    config.autoExposureRoiWidth = 400;
    config.autoExposureRoiHeight = 300;

    config.autoWhiteBalance = 1;
    config.whiteBalanceRed = 2.2;
    config.whiteBalanceBlue = 3.3;
    config.autoWhiteBalanceDecay = 4;
    config.autoWhiteBalanceThresh = 0.7;

    config.sharpeningEnable = true;
    config.sharpeningPercentage = 10.0;
    config.sharpeningLimit = 80;

    return config;
}

crl::multisense::details::wire::SysPacketDelay create_wire_packet_delay()
{
    using namespace crl::multisense::details;

    wire::SysPacketDelay delay;
    delay.enable = true;

    return delay;
}

crl::multisense::details::wire::ImuConfig create_valid_imu_wire_config()
{
    using namespace crl::multisense::details;

    wire::ImuConfig config;

    config.storeSettingsInFlash = false;
    config.samplesPerMessage = 300;

    config.configs.resize(3);

    config.configs[0].name = "accelerometer";
    config.configs[0].flags = 0;
    config.configs[0].rateTableIndex = 1;
    config.configs[0].rangeTableIndex = 0;

    config.configs[1].name = "gyroscope";
    config.configs[1].flags = 1;
    config.configs[1].rateTableIndex = 0;
    config.configs[1].rangeTableIndex = 1;

    config.configs[2].name = "magnetometer";
    config.configs[2].flags = 1;
    config.configs[2].rateTableIndex = 1;
    config.configs[2].rangeTableIndex = 1;

    return config;
}

crl::multisense::details::wire::LedStatus create_valid_lighting_wire_config()
{
    using namespace crl::multisense::details;

    wire::LedStatus config;

    config.available = 1 | 1<<1 | 1<<2;

    config.intensity[0] = 255;
    config.intensity[1] = 128;
    config.intensity[2] = 1;

    config.flash = 1;
    config.led_delay_us = 0;
    config.number_of_pulses = 1;
    config.invert_pulse = 0;
    config.rolling_shutter_led = 1;

    return config;
}

void check_equal(const multisense::MultiSenseConfig &config,
                 const crl::multisense::details::wire::CamSetResolution &res)
{
    using namespace multisense;

    ASSERT_EQ(config.width, res.width);
    ASSERT_EQ(config.height, res.height);

    switch(config.disparities)
    {
        case MultiSenseConfig::MaxDisparities::D64: {ASSERT_EQ(64, res.disparities); break;}
        case MultiSenseConfig::MaxDisparities::D128: {ASSERT_EQ(128, res.disparities); break;}
        case MultiSenseConfig::MaxDisparities::D256: {ASSERT_EQ(256, res.disparities); break;}
    }
}

template <typename ConfigT, typename ControlT>
void check_equal(const ConfigT &config,
                 const ControlT &control)
{
    ASSERT_TRUE(static_cast<bool>(config.manual_exposure));
    ASSERT_FLOAT_EQ(config.manual_exposure->gain, control.gain);
    ASSERT_EQ(config.manual_exposure->exposure_time.count(), control.exposure);

    ASSERT_TRUE(static_cast<bool>(config.auto_exposure));
    ASSERT_EQ(config.auto_exposure_enabled, control.autoExposure);
    ASSERT_EQ(config.auto_exposure->max_exposure_time.count(), control.autoExposureMax);
    ASSERT_EQ(config.auto_exposure->decay, control.autoExposureDecay);
    ASSERT_FLOAT_EQ(config.auto_exposure->target_threshold, control.autoExposureThresh);
    ASSERT_FLOAT_EQ(config.auto_exposure->target_intensity, control.autoExposureTargetIntensity);
    ASSERT_FLOAT_EQ(config.auto_exposure->max_gain, control.gainMax);

    ASSERT_EQ(config.auto_exposure->roi.top_left_x_position, control.autoExposureRoiX);
    ASSERT_EQ(config.auto_exposure->roi.top_left_y_position, control.autoExposureRoiY);
    ASSERT_EQ(config.auto_exposure->roi.width, control.autoExposureRoiWidth);
    ASSERT_EQ(config.auto_exposure->roi.height, control.autoExposureRoiHeight);

    ASSERT_TRUE(static_cast<bool>(config.manual_white_balance));
    ASSERT_FLOAT_EQ(config.manual_white_balance->red, control.whiteBalanceRed);
    ASSERT_FLOAT_EQ(config.manual_white_balance->blue, control.whiteBalanceBlue);

    ASSERT_TRUE(static_cast<bool>(config.auto_white_balance));
    ASSERT_EQ(config.auto_white_balance_enabled, control.autoWhiteBalance);
    ASSERT_EQ(config.auto_white_balance->decay, control.autoWhiteBalanceDecay);
    ASSERT_EQ(config.auto_white_balance->threshold, control.autoWhiteBalanceThresh);

    ASSERT_EQ(config.gamma, control.gamma);
}

void check_equal(const multisense::MultiSenseConfig &config,
                 const crl::multisense::details::wire::CamControl &control)
{
    using namespace multisense;

    ASSERT_FLOAT_EQ(config.frames_per_second, control.framesPerSecond);

    check_equal(config.image_config, control);

    ASSERT_FLOAT_EQ(config.stereo_config.postfilter_strength, control.stereoPostFilterStrength);
}

void check_equal(const multisense::MultiSenseConfig &config,
                 const crl::multisense::details::wire::AuxCamControl &control)
{
    using namespace multisense;

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    check_equal(config.aux_config->image_config, control);

    ASSERT_EQ(config.aux_config->sharpening_enabled, control.sharpeningEnable);
    ASSERT_FLOAT_EQ(config.aux_config->sharpening_percentage, control.sharpeningPercentage);
    ASSERT_FLOAT_EQ(config.aux_config->sharpening_limit, control.sharpeningLimit);
}

void check_equal(const multisense::MultiSenseConfig &config,
                 const crl::multisense::details::wire::CamConfig &wire_config)
{
    using namespace multisense;

    ASSERT_FLOAT_EQ(config.stereo_config.postfilter_strength, wire_config.stereoPostFilterStrength);

    ASSERT_EQ(config.width, wire_config.width);
    ASSERT_EQ(config.height, wire_config.height);

    switch(config.disparities)
    {
        case MultiSenseConfig::MaxDisparities::D64: {ASSERT_EQ(64, wire_config.disparities); break;}
        case MultiSenseConfig::MaxDisparities::D128: {ASSERT_EQ(128, wire_config.disparities); break;}
        case MultiSenseConfig::MaxDisparities::D256: {ASSERT_EQ(256, wire_config.disparities); break;}
    }

    ASSERT_EQ(config.frames_per_second, wire_config.framesPerSecond);

    check_equal(config.image_config, wire_config);
}

void check_equal(const multisense::MultiSenseConfig::AuxConfig &config,
                 const crl::multisense::details::wire::AuxCamConfig &wire_config)
{
    check_equal(config.image_config, wire_config);

    ASSERT_EQ(config.sharpening_enabled, wire_config.sharpeningEnable);
    ASSERT_FLOAT_EQ(config.sharpening_percentage, wire_config.sharpeningPercentage);
    ASSERT_FLOAT_EQ(config.sharpening_limit, wire_config.sharpeningLimit);
}

void check_equal(const multisense::ImuRate &rate0,
                 const multisense::ImuRate &rate1)
{
    ASSERT_FLOAT_EQ(rate0.sample_rate, rate1.sample_rate);
    ASSERT_FLOAT_EQ(rate0.bandwith_cutoff, rate1.bandwith_cutoff);
}

void check_equal(const multisense::ImuRange &range0,
                 const multisense::ImuRange &range1)
{
    ASSERT_FLOAT_EQ(range0.range, range1.range);
    ASSERT_FLOAT_EQ(range0.resolution, range1.resolution);
}

void check_equal(const multisense::MultiSenseConfig::ImuConfig &config,
                 const crl::multisense::details::wire::ImuConfig &wire_config,
                 const multisense::MultiSenseInfo::ImuInfo &imu_info)
{
    using namespace crl::multisense::details;

    ASSERT_EQ(config.samples_per_frame, wire_config.samplesPerMessage);

    ASSERT_EQ(static_cast<bool>(imu_info.accelerometer), static_cast<bool>(config.accelerometer));
    ASSERT_EQ(static_cast<bool>(imu_info.gyroscope), static_cast<bool>(config.gyroscope));
    ASSERT_EQ(static_cast<bool>(imu_info.magnetometer), static_cast<bool>(config.magnetometer));

    ASSERT_GT(wire_config.configs.size(), 2);

    for (size_t i = 0 ; i < wire_config.configs.size() ; ++i)
    {
        const auto &wire = wire_config.configs[i];

        if (imu_info.accelerometer && wire.name == imu_info.accelerometer->name)
        {
            const auto rate = imu_info.accelerometer->rates[wire.rateTableIndex];
            const auto range = imu_info.accelerometer->ranges[wire.rangeTableIndex];

            ASSERT_EQ(config.accelerometer->enabled, static_cast<bool>(wire.flags & wire::imu::Config::FLAGS_ENABLED));

            check_equal(config.accelerometer->rate, rate);
            check_equal(config.accelerometer->range, range);
        }
        else if (imu_info.gyroscope && wire.name == imu_info.gyroscope->name)
        {
            const auto rate = imu_info.gyroscope->rates[wire.rateTableIndex];
            const auto range = imu_info.gyroscope->ranges[wire.rangeTableIndex];

            ASSERT_EQ(config.gyroscope->enabled, static_cast<bool>(wire.flags & wire::imu::Config::FLAGS_ENABLED));

            check_equal(config.gyroscope->rate, rate);
            check_equal(config.gyroscope->range, range);
        }
        else if (imu_info.magnetometer && wire.name == imu_info.magnetometer->name)
        {
            const auto rate = imu_info.magnetometer->rates[wire.rateTableIndex];
            const auto range = imu_info.magnetometer->ranges[wire.rangeTableIndex];

            ASSERT_EQ(config.magnetometer->enabled, static_cast<bool>(wire.flags & wire::imu::Config::FLAGS_ENABLED));

            check_equal(config.magnetometer->rate, rate);
            check_equal(config.magnetometer->range, range);
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
}

void check_equal(const multisense::MultiSenseConfig::LightingConfig &config,
                 const crl::multisense::details::wire::LedStatus &wire)
{
    using namespace multisense;

    if (config.internal)
    {
        ASSERT_FLOAT_EQ(config.internal->intensity, wire.intensity[0] / 255.0f * 100.0f);
        ASSERT_EQ(wire.led_delay_us, 0);
        ASSERT_EQ(wire.number_of_pulses, 1);
        ASSERT_EQ(wire.invert_pulse, 0);
        ASSERT_EQ(config.internal->flash, wire.flash != 0);
    }

    if (config.external)
    {
        ASSERT_FLOAT_EQ(config.external->intensity, wire.intensity[0] / 255.0f * 100.0f);
        ASSERT_EQ(wire.led_delay_us, config.external->startup_time.count());
        ASSERT_EQ(wire.number_of_pulses, config.external->pulses_per_exposure);
        ASSERT_EQ(wire.invert_pulse, 0);

        switch (config.external->flash)
        {
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::NONE:
            {
                ASSERT_EQ(wire.flash, 0);
                ASSERT_EQ(wire.rolling_shutter_led, 0);
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO:
            {
                ASSERT_EQ(wire.flash, 1);
                ASSERT_EQ(wire.rolling_shutter_led, 0);
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_AUX:
            {
                ASSERT_EQ(wire.flash, 1);
                ASSERT_EQ(wire.rolling_shutter_led, 1);
                break;
            }
        }
    }
}

void check_equal(const multisense::MultiSenseConfig::LightingConfig &config,
                 const crl::multisense::details::wire::LedSet &wire)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    if (config.internal)
    {
        for (size_t i = 0 ; i < wire::MAX_LIGHTS ; ++i)
        {
            ASSERT_FLOAT_EQ(config.internal->intensity, wire.intensity[i] / 255.0f * 100.0f);
            ASSERT_EQ(wire.mask & 1<<i, 1<<i);
        }

        ASSERT_EQ(wire.led_delay_us, 0);
        ASSERT_EQ(wire.number_of_pulses, 1);
        ASSERT_EQ(wire.invert_pulse, 0);
        ASSERT_EQ(config.internal->flash, wire.flash != 0);
    }

    if (config.external)
    {
        for (size_t i = 0 ; i < wire::MAX_LIGHTS ; ++i)
        {
            ASSERT_FLOAT_EQ(config.external->intensity, wire.intensity[i] / 255.0f * 100.0f);
            ASSERT_EQ(wire.mask & 1<<i, 1<<i);
        }

        ASSERT_EQ(wire.led_delay_us, config.external->startup_time.count());
        ASSERT_EQ(wire.number_of_pulses, config.external->pulses_per_exposure);
        ASSERT_EQ(wire.invert_pulse, 0);

        switch (config.external->flash)
        {
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::NONE:
            {
                ASSERT_EQ(wire.flash, 0);
                ASSERT_EQ(wire.rolling_shutter_led, 0);
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO:
            {
                ASSERT_EQ(wire.flash, 1);
                ASSERT_EQ(wire.rolling_shutter_led, 0);
                break;
            }
            case MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_AUX:
            {
                ASSERT_EQ(wire.flash, 1);
                ASSERT_EQ(wire.rolling_shutter_led, 1);
                break;
            }
        }
    }
}

void check_equal(const multisense::MultiSenseConfig::NetworkTransmissionConfig &config,
                 const crl::multisense::details::wire::SysPacketDelay &packet_delay)
{
    ASSERT_EQ(config.packet_delay_enabled, packet_delay.enable);
}

TEST(equality, equal)
{
    using namespace multisense;

    const auto config = create_valid_config(1920, 1200);

    ASSERT_TRUE(config == config);
}

TEST(equality, not_equal)
{
    using namespace multisense;

    const auto config0 = create_valid_config(1920, 1200);
    const auto config1 = create_valid_config(1920, 1201);

    ASSERT_FALSE(config0 == config1);
}

TEST(convert, cam_resolution_full_res)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto config = create_valid_config(1920, 1200);

    const auto wire_resolution = convert<wire::CamSetResolution>(config);

    check_equal(config, wire_resolution);
}

TEST(convert, cam_control)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto config = create_valid_config(1920, 1200);

    const auto cam_control = convert<wire::CamControl>(config);

    check_equal(config, cam_control);
}

TEST(convert, aux_cam_control)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto config = create_valid_config(1920, 1200);

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    const auto cam_control = convert(config.aux_config.value());

    check_equal(config, cam_control);
}


TEST(convert, cam_config)
{
    const auto wire_config = create_valid_wire_config();
    const auto wire_aux_config = create_valid_wire_aux_config();
    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                wire_aux_config,
                                std::nullopt,
                                std::nullopt,
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    check_equal(config, wire_config);
    check_equal(config.aux_config.value(), wire_aux_config);
}

TEST(convert, cam_config_invalid_aux)
{
    const auto wire_config = create_valid_wire_config();
    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                std::nullopt,
                                std::nullopt,
                                std::nullopt,
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_FALSE(static_cast<bool>(config.aux_config));

    check_equal(config, wire_config);
}

TEST(convert, cam_config_invalid_imu)
{
    const auto wire_config = create_valid_wire_config();
    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                std::nullopt,
                                std::nullopt,
                                std::nullopt,
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_FALSE(static_cast<bool>(config.imu_config));

    check_equal(config, wire_config);

    ASSERT_TRUE(static_cast<bool>(config.network_config));
    check_equal(config.network_config.value(), packet_config);
}

TEST(convert, cam_config_invalid_led)
{
    const auto wire_config = create_valid_wire_config();
    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                std::nullopt,
                                std::nullopt,
                                std::nullopt,
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_FALSE(static_cast<bool>(config.lighting_config));

    check_equal(config, wire_config);

    ASSERT_TRUE(static_cast<bool>(config.network_config));
    check_equal(config.network_config.value(), packet_config);
}

TEST(convert, cam_config_valid_led_but_no_ligths)
{
    const auto wire_config = create_valid_wire_config();

    auto lighting_config = create_valid_lighting_wire_config();
    lighting_config.available = 0;

    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                std::nullopt,
                                std::nullopt,
                                std::make_optional(lighting_config),
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_FALSE(static_cast<bool>(config.lighting_config));

    check_equal(config, wire_config);

    ASSERT_TRUE(static_cast<bool>(config.network_config));
    check_equal(config.network_config.value(), packet_config);
}

TEST(convert, imu_config_round_trip)
{
    const auto wire_config = create_valid_imu_wire_config();
    const auto info = create_imu_info();

    const auto round_trip = convert(convert(convert(wire_config, info), info, 10000), info);

    check_equal(round_trip, wire_config, info);
}

TEST(convert, imu_config_limit_messages)
{
    const auto wire_config = create_valid_imu_wire_config();
    const auto info = create_imu_info();

    //
    // Limit the number of samples to one less than our initialized value
    //
    const auto round_trip = convert(convert(wire_config, info), info, wire_config.samplesPerMessage - 1);

    ASSERT_EQ(round_trip.samplesPerMessage, wire_config.samplesPerMessage - 1);
}

TEST(convert, lighting_config_round_trip)
{
    const auto wire_config = create_valid_lighting_wire_config();

    const auto config = convert(wire_config, multisense::MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL);

    check_equal(config, wire_config);
    check_equal(config, convert(config));
}

TEST(convert, network_config)
{
    using namespace crl::multisense::details;

    const auto wire_config = create_valid_wire_config();
    const auto packet_config = create_wire_packet_delay();

    const auto config = convert(wire_config,
                                std::nullopt,
                                std::nullopt,
                                std::nullopt,
                                packet_config,
                                false,
                                create_device_info(1920, 1200),
                                create_imu_info());

    ASSERT_TRUE(static_cast<bool>(config.network_config));
    check_equal(config.network_config.value(),
                convert<wire::SysPacketDelay>(config.network_config.value()));

}
