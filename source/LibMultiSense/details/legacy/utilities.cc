/**
 * @file utilities.cc
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
 *   2025-01-13, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <algorithm>

#include "MultiSense/MultiSenseUtilities.hh"
#include "details/legacy/utilities.hh"

namespace multisense{
namespace legacy{

bool is_image_source(const DataSource &source)
{
    switch (source)
    {
        case DataSource::LEFT_MONO_RAW:
        case DataSource::RIGHT_MONO_RAW:
        case DataSource::LEFT_MONO_COMPRESSED:
        case DataSource::RIGHT_MONO_COMPRESSED:
        case DataSource::LEFT_RECTIFIED_RAW:
        case DataSource::RIGHT_RECTIFIED_RAW:
        case DataSource::LEFT_RECTIFIED_COMPRESSED:
        case DataSource::RIGHT_RECTIFIED_COMPRESSED:
        case DataSource::LEFT_DISPARITY_RAW:
        case DataSource::LEFT_DISPARITY_COMPRESSED:
        case DataSource::AUX_COMPRESSED:
        case DataSource::AUX_RECTIFIED_COMPRESSED:
        case DataSource::AUX_LUMA_RAW:
        case DataSource::AUX_LUMA_RECTIFIED_RAW:
        case DataSource::AUX_CHROMA_RAW:
        case DataSource::AUX_CHROMA_RECTIFIED_RAW:
        case DataSource::COST_RAW:
            return true;
        default:
            return false;
    }
}

Status get_status(const crl::multisense::details::wire::Ack::AckStatus &status)
{
    using namespace crl::multisense::details::wire;

    switch(status)
    {
        case Ack::Status_Ok: {return Status::OK;}
        case Ack::Status_TimedOut: {return Status::TIMEOUT;}
        case Ack::Status_Error: {return Status::INTERNAL_ERROR;}
        case Ack::Status_Failed: {return Status::FAILED;}
        case Ack::Status_Unsupported: {return Status::UNSUPPORTED;}
        case Ack::Status_Unknown: {return Status::UNKNOWN;}
        case Ack::Status_Exception: {return Status::EXCEPTION;}
        default: {return Status::UNKNOWN;}
    }
}

MultiSenseInfo::Version get_version(const crl::multisense::details::wire::VersionType &version)
{
    return MultiSenseInfo::Version{static_cast<uint32_t>(version >> 8), static_cast<uint32_t>(version & 0xFF), 0};
}

MultiSenseConfig::MaxDisparities get_disparities(size_t disparity)
{
    auto disparities = MultiSenseConfig::MaxDisparities::D256;
    switch (disparity)
    {
        case 64: {disparities = MultiSenseConfig::MaxDisparities::D64; break;}
        case 128: {disparities = MultiSenseConfig::MaxDisparities::D128; break;}
        case 256: {disparities = MultiSenseConfig::MaxDisparities::D256; break;}
        default: {CRL_EXCEPTION("Unsupported disparity value %d", disparity);}
    }

    return disparities;
}

std::vector<DataSource> convert_sources(const crl::multisense::details::wire::SourceType &source)
{
    using namespace crl::multisense::details;

    std::vector<DataSource> sources;
    if (source & wire::SOURCE_LUMA_LEFT) {sources.push_back(DataSource::LEFT_MONO_RAW);}
    if (source & wire::SOURCE_LUMA_RIGHT) {sources.push_back(DataSource::RIGHT_MONO_RAW);}
    if (source & wire::SOURCE_COMPRESSED_LEFT) {sources.push_back(DataSource::LEFT_MONO_COMPRESSED);}
    if (source & wire::SOURCE_COMPRESSED_RIGHT) {sources.push_back(DataSource::RIGHT_MONO_COMPRESSED);}
    if (source & wire::SOURCE_LUMA_RECT_LEFT) {sources.push_back(DataSource::LEFT_RECTIFIED_RAW);}
    if (source & wire::SOURCE_LUMA_RECT_RIGHT) {sources.push_back(DataSource::RIGHT_RECTIFIED_RAW);}
    if (source & wire::SOURCE_COMPRESSED_RECTIFIED_LEFT) {sources.push_back(DataSource::LEFT_RECTIFIED_COMPRESSED);}
    if (source & wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT) {sources.push_back(DataSource::RIGHT_RECTIFIED_COMPRESSED);}
    if (source & wire::SOURCE_DISPARITY) {sources.push_back(DataSource::LEFT_DISPARITY_RAW);}
    if (source & wire::SOURCE_COMPRESSED_AUX) {sources.push_back(DataSource::AUX_COMPRESSED);}
    if (source & wire::SOURCE_COMPRESSED_RECTIFIED_AUX) {sources.push_back(DataSource::AUX_RECTIFIED_COMPRESSED);}
    if (source & wire::SOURCE_LUMA_AUX) {sources.push_back(DataSource::AUX_LUMA_RAW);}
    if (source & wire::SOURCE_LUMA_RECT_AUX) {sources.push_back(DataSource::AUX_LUMA_RECTIFIED_RAW);}
    if (source & wire::SOURCE_CHROMA_AUX) {sources.push_back(DataSource::AUX_CHROMA_RAW);}
    if (source & wire::SOURCE_CHROMA_RECT_AUX) {sources.push_back(DataSource::AUX_CHROMA_RECTIFIED_RAW);}
    if (source & wire::SOURCE_DISPARITY_COST) {sources.push_back(DataSource::COST_RAW);}
    if (source & wire::SOURCE_IMU) {sources.push_back(DataSource::IMU);}

    return sources;
}


crl::multisense::details::wire::SourceType convert_sources(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;

    wire::SourceType mask = 0;
    for (const auto &source : sources)
    {
        switch(source)
        {
            case DataSource::LEFT_MONO_RAW: {mask |= wire::SOURCE_LUMA_LEFT; break;}
            case DataSource::RIGHT_MONO_RAW: {mask |= wire::SOURCE_LUMA_RIGHT; break;}
            case DataSource::LEFT_MONO_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_LEFT; break;}
            case DataSource::RIGHT_MONO_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RIGHT; break;}
            case DataSource::LEFT_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_LEFT; break;}
            case DataSource::RIGHT_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_RIGHT; break;}
            case DataSource::LEFT_RECTIFIED_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RECTIFIED_LEFT; break;}
            case DataSource::RIGHT_RECTIFIED_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT; break;}
            case DataSource::LEFT_DISPARITY_RAW: {mask |= wire::SOURCE_DISPARITY; break;}
            case DataSource::LEFT_DISPARITY_COMPRESSED: { CRL_DEBUG("Compressed disparity not supported"); break;}
            case DataSource::AUX_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_AUX; break;}
            case DataSource::AUX_RECTIFIED_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RECTIFIED_AUX; break;}
            case DataSource::AUX_LUMA_RAW: {mask |= wire::SOURCE_LUMA_AUX; break;}
            case DataSource::AUX_LUMA_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_AUX; break;}
            case DataSource::AUX_CHROMA_RAW: {mask |= wire::SOURCE_CHROMA_AUX; break;}
            case DataSource::AUX_CHROMA_RECTIFIED_RAW: {mask |= wire::SOURCE_CHROMA_RECT_AUX; break;}
            case DataSource::AUX_RAW: {mask |= wire::SOURCE_CHROMA_AUX | wire::SOURCE_LUMA_AUX; break;}
            case DataSource::AUX_RECTIFIED_RAW: {mask |= wire::SOURCE_CHROMA_RECT_AUX | wire::SOURCE_LUMA_RECT_AUX; break;}
            case DataSource::COST_RAW: {mask |= wire::SOURCE_DISPARITY_COST; break;}
            case DataSource::IMU: {mask |= wire::SOURCE_IMU; break;}
            case DataSource::ALL: {mask |= all_sources; break;}
            default: {CRL_DEBUG("Unsupported source %d", static_cast<int32_t>(source));}
        }
    }

    return mask;
}

std::vector<DataSource> expand_source(const DataSource &source)
{
    switch(source)
    {
        case DataSource::AUX_RAW:
        {
            return std::vector<DataSource>{DataSource::AUX_LUMA_RAW, DataSource::AUX_CHROMA_RAW};
        }
        case DataSource::AUX_RECTIFIED_RAW:
        {
            return std::vector<DataSource>{DataSource::AUX_LUMA_RECTIFIED_RAW, DataSource::AUX_CHROMA_RECTIFIED_RAW};
        }
        default: {return std::vector<DataSource>{source};}
    }
}

ImuSample add_wire_sample(ImuSample sample,
                          const crl::multisense::details::wire::ImuSample &wire,
                          const ImuSampleScalars &scalars)
{
    using namespace crl::multisense::details;


    switch(wire.type)
    {
        case wire::ImuSample::TYPE_ACCEL:
        {
            sample.accelerometer = ImuSample::Measurement{static_cast<float>(wire.x * scalars.accelerometer_scale),
                                                          static_cast<float>(wire.y * scalars.accelerometer_scale),
                                                          static_cast<float>(wire.z * scalars.accelerometer_scale)};
            break;
        }
        case wire::ImuSample::TYPE_GYRO:
        {
            sample.gyroscope = ImuSample::Measurement{static_cast<float>(wire.x * scalars.gyroscope_scale),
                                                      static_cast<float>(wire.y * scalars.gyroscope_scale),
                                                      static_cast<float>(wire.z * scalars.gyroscope_scale)};
            break;
        }
        case wire::ImuSample::TYPE_MAG:
        {
            sample.magnetometer = ImuSample::Measurement{static_cast<float>(wire.x * scalars.magnetometer_scale),
                                                         static_cast<float>(wire.y * scalars.magnetometer_scale),
                                                         static_cast<float>(wire.z * scalars.magnetometer_scale)};
            break;
        }
        default:
        {
            CRL_EXCEPTION("Unknown IMU sample type");
        }
    }

    return sample;
}

uint32_t get_rate_index(const std::vector<ImuRate> &rates, const ImuRate &rate)
{
    return static_cast<uint32_t>(std::distance(std::begin(rates),
                                               std::find_if(std::begin(rates), std::end(rates),
                                                            [&rate](const auto &e)
                                                            {
                                                                return (std::abs(e.sample_rate - rate.sample_rate) < 1e-6 &&
                                                                        std::abs(e.bandwith_cutoff - rate.bandwith_cutoff) < 1e-6);
                                                            })));
}

uint32_t get_range_index(const std::vector<ImuRange> &ranges, const ImuRange &range)
{
    return static_cast<uint32_t>(std::distance(std::begin(ranges),
                                 std::find_if(std::begin(ranges), std::end(ranges),
                                              [&range](const auto &e)
                                              {
                                                  return (std::abs(e.range - range.range) < 1e-6 &&
                                                          std::abs(e.resolution - range.resolution) < 1e-6);
                                              })));
}

double get_acceleration_scale(const std::string &units)
{
    std::string lower_units = units;
    std::transform(lower_units.begin(), lower_units.end(), lower_units.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c));});


    if (lower_units == "g" || lower_units == "gs")
    {
        return 1.0;
    }
    else if (lower_units == "millig" || lower_units == "milli-g")
    {
        return 1.0/1000.0;
    }
    else
    {
        CRL_DEBUG("Unknown acceleration units: %s\n", units.c_str());
    }

    return 1.0;
}

double get_gyroscope_scale(const std::string &units)
{
    std::string lower_units = units;
    std::transform(lower_units.begin(), lower_units.end(), lower_units.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c));});

    if (lower_units == "dps" || lower_units == "degrees-per-second")
    {
        return 1.0;
    }
    else if (lower_units == "rps" || units == "radians-per-second")
    {
        return 1.0/1000.0;
    }
    else
    {
        CRL_DEBUG("Unknown gyroscope units: %s\n", units.c_str());
    }

    return 1.0;
}

double get_magnetomter_scale(const std::string &units)
{
    std::string lower_units = units;
    std::transform(lower_units.begin(), lower_units.end(), lower_units.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c));});

    if (lower_units == "guass")
    {
        return 1000.0;
    }
    else if (lower_units == "milligauss" || units == "milli-gauss")
    {
        return 1.0;
    }
    else
    {
        CRL_DEBUG("Unknown magnetometer units: %s\n", units.c_str());
    }

    return 1.0;
}

ImuSampleScalars get_imu_scalars(const crl::multisense::details::wire::ImuInfo &info)
{
    ImuSampleScalars output;

    for (const auto &mode : info.details)
    {
        if (mode.name == "accelerometer")
        {
            output.accelerometer_scale = get_acceleration_scale(mode.units);
            continue;
        }
        else if (mode.name == "gyroscope")
        {
            output.gyroscope_scale = get_gyroscope_scale(mode.units);
            continue;
        }
        else if (mode.name == "magnetometer")
        {
            output.magnetometer_scale = get_magnetomter_scale(mode.units);
            continue;
        }
        else
        {
            CRL_EXCEPTION("Unknown IMU name: %s\n", mode.name.c_str());
        }
    }

    return output;
}

}

std::optional<Image> create_bgr(const ImageFrame &frame, const DataSource &output_source)
{
    const auto expanded_sources = legacy::expand_source(output_source);

    if (expanded_sources.size() != 2 || !frame.has_image(expanded_sources[0]) || !frame.has_image(expanded_sources[1]))
    {
        return std::nullopt;
    }

    return create_bgr_image(frame.get_image(expanded_sources[0]), frame.get_image(expanded_sources[1]), output_source);
}

}
