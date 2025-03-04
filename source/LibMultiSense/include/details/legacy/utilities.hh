/**
 * @file utilities.hh
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

#pragma once

#include <utility/Exception.hh>
#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>
#include <wire/AckMessage.hh>
#include <wire/ImuDataMessage.hh>
#include <wire/ImuInfoMessage.hh>

#include "details/legacy/message.hh"
#include "details/legacy/udp.hh"

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense{
namespace legacy{

///
/// @brief All the supported wire source types created for convenience
///
constexpr crl::multisense::details::wire::SourceType all_sources = {
    crl::multisense::details::wire::SOURCE_LUMA_LEFT |
    crl::multisense::details::wire::SOURCE_LUMA_RIGHT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_LEFT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RIGHT |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_LEFT |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_RIGHT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RECTIFIED_LEFT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT |
    crl::multisense::details::wire::SOURCE_DISPARITY |
    crl::multisense::details::wire::SOURCE_COMPRESSED_AUX |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RECTIFIED_AUX |
    crl::multisense::details::wire::SOURCE_LUMA_AUX |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_AUX |
    crl::multisense::details::wire::SOURCE_CHROMA_AUX |
    crl::multisense::details::wire::SOURCE_CHROMA_RECT_AUX |
    crl::multisense::details::wire::SOURCE_DISPARITY_COST |
    crl::multisense::details::wire::SOURCE_IMU
};

///
/// @brief Values to scale IMU samples from the MultiSense camera into standard units LibMultiSense
///        expects
///
struct ImuSampleScalars
{
    ///
    /// @brief Scale for the acclerometer to convert wire samples into G's
    ///
    double accelerometer_scale = 1.0;

    ///
    /// @brief Scale for the gyroscope_scale to convert wire samples into degrees/sec
    ///
    double gyroscope_scale = 1.0;

    ///
    /// @brief Scale for the magnetometer to convert wire samples into milligauss
    ///
    double magnetometer_scale = 1.0;
};

///
/// @brief Determine if a datasource is a image source
///
bool is_image_source(const DataSource &source);

///
/// Convert a wire status to a API Status
///
Status get_status(const crl::multisense::details::wire::Ack::AckStatus &status);

///
/// @brief Convert a wire version to a API Version
///
MultiSenseInfo::Version get_version(const crl::multisense::details::wire::VersionType &version);

///
/// @brief Convert a disparity integer to a fixed disparity setting
///
MultiSenseConfig::MaxDisparities get_disparities(size_t disparity);

///
/// @brief Convert wire sources to a vector of DataSources
///
std::vector<DataSource> convert_sources(const crl::multisense::details::wire::SourceType &source);

///
/// @brief Convert a vector of DataSources to a wire source
///
crl::multisense::details::wire::SourceType convert_sources(const std::vector<DataSource> &sources);

///
/// @brief Expand sources since some sources may represent multiple sources on the wire
///
std::vector<DataSource> expand_source(const DataSource &source);

///
/// @brief Add a wire sample to a ImuSample
///
ImuSample add_wire_sample(ImuSample sample,
                          const crl::multisense::details::wire::ImuSample &wire,
                          const ImuSampleScalars &scalars);

///
/// @brief Get the index of the rate in a vector of rates
///
uint32_t get_rate_index(const std::vector<ImuRate> &rates, const ImuRate &rate);

///
/// @brief Get the index of the range in a vector of ranges
///
uint32_t get_range_index(const std::vector<ImuRange> &ranges, const ImuRange &range);

///
/// @brief Get a scale for the acceleration value based on a units string
///
double get_acceleration_scale(const std::string &units);

///
/// @brief Get a scale for the gyroscope value based on a units string
///
double get_gyroscope_scale(const std::string &units);

///
/// @brief Get a scale for the magnetometer value based on a units string
///
double get_magnetomter_scale(const std::string &units);

///
/// @brief Get IMU scalars from the cameras's reported IMU info
///
ImuSampleScalars get_imu_scalars(const crl::multisense::details::wire::ImuInfo &info);

///
/// @brief Helper to wait for ack from the camera from a given query command. Once a query
///        command is sent to the MultiSense, it Ack's the command before sending the response
///
template <typename QueryMessage, class Rep, class Period>
std::optional<crl::multisense::details::wire::Ack> wait_for_ack(MessageAssembler &assembler,
                                                               const NetworkSocket &socket,
                                                               const QueryMessage &query,
                                                               uint16_t sequence_id,
                                                               uint16_t mtu,
                                                               const std::optional<std::chrono::duration<Rep, Period>>& wait_time,
                                                               size_t attempts = 1)
{
    using namespace crl::multisense::details;

    std::optional<wire::Ack> output = std::nullopt;

    auto ack_waiter = assembler.register_message(MSG_ID(QueryMessage::ID));

    for (size_t i = 0 ; i < attempts ; ++i)
    {
        if(publish_data(socket, serialize(query, sequence_id, mtu)) < 0)
        {
            continue;
        }

        if (auto ack = ack_waiter->wait<wire::Ack>(wait_time); ack)
        {
            output = std::move(ack);
            break;
        }
    }

    assembler.remove_registration(MSG_ID(QueryMessage::ID));

    return output;
}


///
/// @brief Helper to wait for data from the camera from a given query command. Once a query
///        command is sent to the MultiSense, it Ack's the command before sending the response
///
template <typename OutputMessage, typename QueryMessage, class Rep, class Period>
std::optional<OutputMessage> wait_for_data(MessageAssembler &assembler,
                                           const NetworkSocket &socket,
                                           const QueryMessage &query,
                                           uint16_t sequence_id,
                                           uint16_t mtu,
                                           const std::optional<std::chrono::duration<Rep, Period>>& wait_time,
                                           size_t attempts = 1)
{
    using namespace crl::multisense::details;

    std::optional<OutputMessage> output = std::nullopt;

    auto ack_waiter = assembler.register_message(MSG_ID(QueryMessage::ID));
    auto response_waiter = assembler.register_message(MSG_ID(OutputMessage::ID));

    for (size_t i = 0 ; i < attempts ; ++i)
    {
        if(publish_data(socket, serialize(query, sequence_id, mtu)) < 0)
        {
            continue;
        }

        if (auto ack = ack_waiter->wait<wire::Ack>(wait_time); ack)
        {
            //
            // TODO (malvarado): Uncomment this once the firmware issues are resolved
            //
            //if (ack->status != wire::Ack::Status_Ok)
            //{
            //    continue;
            //}

            if (auto response = response_waiter->wait<OutputMessage>(wait_time); response)
            {
                output = std::move(response);
                break;
            }
        }
    }

    assembler.remove_registration(MSG_ID(QueryMessage::ID));
    assembler.remove_registration(MSG_ID(OutputMessage::ID));

    return output;
}

template <typename T>
struct TimedResponse
{
    ///
    /// @brief The host system time the request which triggered the response message was sent
    ///
    std::chrono::nanoseconds host_start_transmit_time{0};

    ///
    /// @brief The time between when the status request was sent and when the camera ack'd the response
    ///
    std::chrono::nanoseconds host_transmit_receive_roundtrip{0};

    ///
    /// @brief The response message
    ///
    T message{};
};

///
/// @brief Helper to wait for data from the camera from a given query command. Once a query
///        command is sent to the MultiSense, it Ack's the command before sending the response
///
template <typename OutputMessage, typename QueryMessage, class Rep, class Period>
std::optional<TimedResponse<OutputMessage>> wait_for_data_timed(MessageAssembler &assembler,
                                                                const NetworkSocket &socket,
                                                                const QueryMessage &query,
                                                                uint16_t sequence_id,
                                                                uint16_t mtu,
                                                                const std::optional<std::chrono::duration<Rep, Period>>& wait_time,
                                                                size_t attempts = 1)
{
    using namespace crl::multisense::details;

    std::optional<TimedResponse<OutputMessage>> output = std::nullopt;

    auto ack_waiter = assembler.register_message(MSG_ID(QueryMessage::ID));
    auto response_waiter = assembler.register_message(MSG_ID(OutputMessage::ID));

    for (size_t i = 0 ; i < attempts ; ++i)
    {
        const auto serialized_data = serialize(query, sequence_id, mtu);

        const auto send = std::chrono::system_clock::now().time_since_epoch();
        const auto start = std::chrono::high_resolution_clock::now();

        if(publish_data(socket, serialized_data) < 0)
        {
            continue;
        }

        if (auto ack = ack_waiter->wait<wire::Ack>(wait_time); ack)
        {
            const auto end = std::chrono::high_resolution_clock::now();

            //
            // TODO (malvarado): Uncomment this once the firmware issues are resolved
            //
            //if (ack->status != wire::Ack::Status_Ok)
            //{
            //    continue;
            //}

            if (auto response = response_waiter->wait<OutputMessage>(wait_time); response)
            {
                output = TimedResponse<OutputMessage>{send, end - start, response.value()};
                break;
            }
        }
    }

    assembler.remove_registration(MSG_ID(QueryMessage::ID));
    assembler.remove_registration(MSG_ID(OutputMessage::ID));

    return output;
}

}
}
