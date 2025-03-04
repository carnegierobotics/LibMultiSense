/**
 * @file info_test.cc
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
 *   2025-01-22, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/info.hh>
#include <details/legacy/utilities.hh>

using namespace multisense::legacy;

crl::multisense::details::wire::SysDeviceInfo create_wire_info(const std::string &name, const std::string &key)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info;
    info.key = key;
    info.name = name;
    info.buildDate = "2024";
    info.serialNumber = "4501";
    info.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21;
    info.numberOfPcbs = 1;

    wire::PcbInfo pcb;
    pcb.name = name;
    pcb.revision = 12345;
    info.pcbs[0] = pcb;

    info.imagerName = name;
    info.imagerType = wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR;
    info.imagerWidth = 1000;
    info.imagerHeight = 1001;
    info.lensName = name;
    info.nominalBaseline = 0.21;
    info.nominalFocalLength = 0.024;
    info.nominalRelativeAperture = 0.1;
    info.lightingType = wire::SysDeviceInfo::LIGHTING_TYPE_S21_EXTERNAL;
    info.numberOfLights = 1;

    return info;
}

crl::multisense::details::wire::VersionResponse create_version()
{
    using namespace crl::multisense::details;

    wire::VersionResponse version;
    version.firmwareBuildDate = "1234-01-23";
    version.firmwareVersion = 0x0522;
    version.hardwareVersion = 123;
    version.hardwareMagic = 567;
    version.fpgaDna = 8906;

    return version;
}

crl::multisense::details::wire::SysDeviceModes create_device_modes(int64_t sources, int32_t imager_width, int32_t imager_height)
{
    using namespace crl::multisense::details;

    wire::SysDeviceModes modes;

    wire::DeviceMode mode0;
    mode0.width = imager_width;
    mode0.height = imager_height;
    mode0.height = imager_height;
    mode0.supportedDataSources = sources;
    mode0.disparities = 64;

    wire::DeviceMode mode1;
    mode1.width = imager_width / 2;
    mode1.height = imager_height / 2;
    mode1.supportedDataSources = sources;
    mode1.disparities = 128;

    wire::DeviceMode mode2;
    mode2.width = 1234;
    mode2.height = 456;
    mode2.supportedDataSources = sources;
    mode2.disparities = 128;

    modes.modes.push_back(mode0);
    modes.modes.push_back(mode1);
    modes.modes.push_back(mode2);

    return modes;
}

crl::multisense::details::wire::ImuInfo create_imu_info()
{
    using namespace crl::multisense::details;

    wire::ImuInfo output;

    output.maxSamplesPerMessage = 100;

    wire::imu::RateType rate0;
    rate0.sampleRate = 101.0f;
    rate0.bandwidthCutoff = 201.0f;

    wire::imu::RateType rate1;
    rate1.sampleRate = 101.1f;
    rate1.bandwidthCutoff = 202.0f;

    wire::imu::RangeType range0;
    range0.range = 10.2f;
    range0.resolution = 0.2f;

    wire::imu::Details detail0;
    detail0.name = "accelerometer";
    detail0.device = "device0";
    detail0.units = "units0";

    detail0.rates.push_back(rate0);
    detail0.rates.push_back(rate1);
    detail0.ranges.push_back(range0);

    wire::imu::Details detail1;
    detail1.name = "gyroscope";
    detail1.device = "device1";
    detail1.units = "units1";

    detail1.rates.push_back(rate0);
    detail1.rates.push_back(rate1);
    detail1.ranges.push_back(range0);

    output.details.push_back(detail0);
    output.details.push_back(detail1);

    return output;
}

multisense::MultiSenseInfo::DeviceInfo create_info(const std::string &name)
{
    using namespace multisense;
    MultiSenseInfo::DeviceInfo info;
    info.camera_name = name;
    info.build_date = "2024";
    info.serial_number = "4501";
    info.hardware_revision = MultiSenseInfo::DeviceInfo::HardwareRevision::KS21;
    info.pcb_info = {MultiSenseInfo::DeviceInfo::PcbInfo{name, 12345}};
    info.imager_name = name;
    info.imager_type = MultiSenseInfo::DeviceInfo::ImagerType::AR0239_COLOR;
    info.imager_width = 1920;
    info.imager_height = 1200;
    info.lens_name = name;
    info.nominal_stereo_baseline = 0.21;
    info.nominal_focal_length = 0.024;
    info.nominal_relative_aperture = 0.1;
    info.lighting_type = MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL;
    info.number_of_lights = 1;

    return info;
}

multisense::MultiSenseInfo::NetworkInfo create_network_info(const std::string &ip_address)
{
    return multisense::MultiSenseInfo::NetworkInfo{ip_address, "test", "bar"};
}

void check_equal(const crl::multisense::details::wire::SysDeviceInfo &wire,
                 const multisense::MultiSenseInfo::DeviceInfo &info,
                 const std::string &key)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    ASSERT_EQ(wire.key, key);
    ASSERT_EQ(wire.name, info.camera_name);
    ASSERT_EQ(wire.buildDate, info.build_date);
    ASSERT_EQ(wire.serialNumber, info.serial_number);

    switch (wire.hardwareRevision)
    {
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::S7); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::S21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::ST21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::S27); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::S30); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::KS21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::MONOCAM); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21_SILVER:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::KS21_SILVER); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST25:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::ST25); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21i:
            ASSERT_EQ(info.hardware_revision, MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i); break;
        default: {CRL_EXCEPTION("Unsupported hardware revision");}
    }

    ASSERT_EQ(wire.numberOfPcbs, info.pcb_info.size());

    for (uint32_t i = 0; i < wire.numberOfPcbs ; ++i)
    {
        ASSERT_EQ(wire.pcbs[i].name, info.pcb_info[i].name);
        ASSERT_EQ(wire.pcbs[i].revision, info.pcb_info[i].revision);
    }

    ASSERT_EQ(wire.imagerName, info.imager_name);
    switch (wire.imagerType)
    {
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_GREY:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_COLOR:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_COLOR); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_GREY:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_COLOR:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_COLOR); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_FLIR_TAU2:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::FLIR_TAU2); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0234_GREY:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::AR0234_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR:
            ASSERT_EQ(info.imager_type, MultiSenseInfo::DeviceInfo::ImagerType::AR0239_COLOR); break;
        default: {CRL_EXCEPTION("Unsupported imager type");}
    }
    ASSERT_EQ(wire.imagerWidth, info.imager_width);
    ASSERT_EQ(wire.imagerHeight, info.imager_height);

    ASSERT_EQ(wire.lensName, info.lens_name);
    ASSERT_EQ(wire.nominalBaseline, info.nominal_stereo_baseline);
    ASSERT_EQ(wire.nominalFocalLength, info.nominal_focal_length);
    ASSERT_EQ(wire.nominalRelativeAperture, info.nominal_relative_aperture);

    switch (wire.lightingType)
    {
        case wire::SysDeviceInfo::LIGHTING_TYPE_NONE:
             ASSERT_EQ(info.lighting_type, MultiSenseInfo::DeviceInfo::LightingType::NONE); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_SL_INTERNAL:
             ASSERT_EQ(info.lighting_type, MultiSenseInfo::DeviceInfo::LightingType::INTERNAL); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_S21_EXTERNAL:
            ASSERT_EQ(info.lighting_type, MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_S21_PATTERN_PROJECTOR:
            ASSERT_EQ(info.lighting_type, MultiSenseInfo::DeviceInfo::LightingType::PATTERN_PROJECTOR); break;
        default: {CRL_EXCEPTION("Unsupported lighting type");}
    }
    ASSERT_EQ(wire.numberOfLights, info.number_of_lights);
}

void check_equal(const crl::multisense::details::wire::imu::Details &wire,
                 const multisense::MultiSenseInfo::ImuInfo::Source &info)
{
    ASSERT_EQ(wire.name, info.name);
    ASSERT_EQ(wire.device, info.device);

    ASSERT_EQ(wire.rates.size(), info.rates.size());
    for (size_t i = 0 ; i < info.rates.size() ; ++i)
    {
        ASSERT_FLOAT_EQ(wire.rates[i].sampleRate, info.rates[i].sample_rate);
        ASSERT_FLOAT_EQ(wire.rates[i].bandwidthCutoff, info.rates[i].bandwith_cutoff);
    }

    ASSERT_EQ(wire.ranges.size(), info.ranges.size());
    for (size_t i = 0 ; i < info.ranges.size() ; ++i)
    {
        ASSERT_FLOAT_EQ(wire.ranges[i].range, info.ranges[i].range);
        ASSERT_FLOAT_EQ(wire.ranges[i].resolution, info.ranges[i].resolution);
    }
}

void check_equal(const multisense::MultiSenseInfo::NetworkInfo &info0,
                 const multisense::MultiSenseInfo::NetworkInfo &info1)
{
    ASSERT_EQ(info0.ip_address, info1.ip_address);
    ASSERT_EQ(info0.gateway, info1.gateway);
    ASSERT_EQ(info0.netmask, info1.netmask);
}

TEST(convert, wire_to_info)
{
    const auto info = create_wire_info("test", "key");
    check_equal(info, convert(info), "key");
}

TEST(convert, info_to_wire)
{
    const auto info = create_info("test");
    check_equal(convert(info, "key"), info, "key");
}

TEST(convert, version)
{
    const auto version_wire = create_version();
    const auto version = convert(version_wire);

    const auto converted_version = get_version(version_wire.firmwareVersion);

    ASSERT_EQ(version.firmware_build_date, version_wire.firmwareBuildDate);

    //
    // This must be equal
    //
    ASSERT_FALSE(version.firmware_version < converted_version);
    ASSERT_FALSE(converted_version < version.firmware_version);

    ASSERT_EQ(version.hardware_version, version_wire.hardwareVersion);
}

TEST(convert, device_modes)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const wire::IdType sources = wire::SOURCE_LUMA_LEFT | wire::SOURCE_LUMA_RECT_RIGHT | wire::SOURCE_DISPARITY;

    constexpr uint32_t imager_width = 1920;
    constexpr uint32_t imager_height = 1200;

    const auto wire_modes = create_device_modes(sources, imager_width, imager_height);
    const auto modes = convert(wire_modes);

    ASSERT_EQ(modes.size(), wire_modes.modes.size());

    for(size_t i = 0 ; i < modes.size() ; ++i)
    {
        ASSERT_EQ(modes[i].width, wire_modes.modes[i].width);
        ASSERT_EQ(modes[i].height, wire_modes.modes[i].height);

        ASSERT_EQ(modes[i].disparities, get_disparities(wire_modes.modes[i].disparities));

        const auto full_sources = convert_sources(modes[i].supported_sources);
        ASSERT_EQ(full_sources & 0x00000000FFFFFFFF, wire_modes.modes[i].supportedDataSources);
        ASSERT_EQ(full_sources >> 32, wire_modes.modes[i].extendedDataSources);
    }
}

TEST(convert, imu_source)
{
    using namespace crl::multisense::details;

    const auto wire_info = create_imu_info();
    const auto info = convert(wire_info);

    for (size_t i = 0 ; i < wire_info.details.size() ; ++i)
    {
        const auto &detail = wire_info.details[i];

        if (info.accelerometer && info.accelerometer->name == detail.name)
        {
            check_equal(detail, info.accelerometer.value());
        }
        else if (info.gyroscope && info.gyroscope->name == detail.name)
        {
            check_equal(detail, info.gyroscope.value());
        }
        else if (info.magnetometer && info.magnetometer->name == detail.name)
        {
            check_equal(detail, info.magnetometer.value());
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
}

TEST(convert, network_info_round_trip)
{
    const auto network_info = create_network_info("1.2.3.4");

    const auto round_trip = convert(convert(network_info));

    check_equal(network_info, round_trip);
}
