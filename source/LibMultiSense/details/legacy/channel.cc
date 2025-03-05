/**
 * @file channel.cc
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>

#else
#include <arpa/inet.h>
#endif

#include <cstring>
#include <iostream>
#include <inttypes.h>

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
#include <wire/DisparityMessage.hh>
#include <wire/ImageMessage.hh>
#include <wire/ImageMetaMessage.hh>
#include <wire/ImuConfigMessage.hh>
#include <wire/ImuDataMessage.hh>
#include <wire/ImuGetConfigMessage.hh>
#include <wire/ImuGetInfoMessage.hh>
#include <wire/ImuInfoMessage.hh>
#include <wire/LedGetStatusMessage.hh>
#include <wire/LedSetMessage.hh>
#include <wire/LedStatusMessage.hh>
#include <wire/PtpStatusRequestMessage.hh>
#include <wire/PtpStatusResponseMessage.hh>
#include <wire/StatusRequestMessage.hh>
#include <wire/StatusResponseMessage.hh>
#include <wire/StreamControlMessage.hh>
#include <wire/SysGetCameraCalibrationMessage.hh>
#include <wire/SysGetDeviceInfoMessage.hh>
#include <wire/SysGetDeviceModesMessage.hh>
#include <wire/SysGetTransmitDelayMessage.hh>
#include <wire/SysGetPacketDelayMessage.hh>
#include <wire/SysDeviceInfoMessage.hh>
#include <wire/SysDeviceModesMessage.hh>
#include <wire/SysMtuMessage.hh>
#include <wire/SysGetNetworkMessage.hh>
#include <wire/SysNetworkMessage.hh>
#include <wire/SysTestMtuMessage.hh>
#include <wire/SysTestMtuResponseMessage.hh>
#include <wire/VersionRequestMessage.hh>
#include <wire/VersionResponseMessage.hh>

#include "details/legacy/channel.hh"
#include "details/legacy/calibration.hh"
#include "details/legacy/configuration.hh"
#include "details/legacy/info.hh"
#include "details/legacy/message.hh"
#include "details/legacy/status.hh"
#include "details/legacy/utilities.hh"

#include "MultiSense/MultiSenseUtilities.hh"

namespace multisense {
namespace legacy {

namespace {
    ///
    /// @brief The largest MTU we will ever see the camera use. This is used to bound the data read size
    ///
    constexpr uint16_t MAX_MTU = 9000;

    ///
    /// @brief MTU's to test if the user would like to pick the largest MTU
    ///
    constexpr std::array<uint16_t, 10> MTUS_TO_TEST{9000, 8167, 7333, 6500, 5667, 4833, 4000, 3167, 2333, 1500};
}

LegacyChannel::LegacyChannel(const Config &config):
    m_config(config),
    m_buffer_pool(std::make_shared<BufferPool>(BufferPoolConfig{config.receive_buffer_configuration.num_small_buffers,
                                                                config.receive_buffer_configuration.small_buffer_size,
                                                                config.receive_buffer_configuration.num_large_buffers,
                                                                config.receive_buffer_configuration.large_buffer_size})),
    m_message_assembler(m_buffer_pool)
{
    if (connect(config) != Status::OK)
    {
        CRL_EXCEPTION("Connection to MultiSense failed\n");
    }
}

LegacyChannel::~LegacyChannel()
{
    disconnect();
}


Status LegacyChannel::start_streams(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    if (!m_connected)
    {
        return Status::UNINITIALIZED;
    }

    wire::StreamControl cmd;

    cmd.enable(convert_sources(sources));

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      cmd,
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        if (ack->status != wire::Ack::Status_Ok)
        {
            CRL_DEBUG("Start streams ack invalid: %" PRIi32 "\n", ack->status);
            return get_status(ack->status);
        }

        for (const auto &source : sources)
        {
            const auto expanded = expand_source(source);
            m_active_streams.insert(std::begin(expanded), std::end(expanded));
        }

        return Status::OK;
    }

    return Status::TIMEOUT;
}

Status LegacyChannel::stop_streams(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    if (!m_connected)
    {
        return Status::UNINITIALIZED;
    }

    wire::StreamControl cmd;

    cmd.disable(convert_sources(sources));

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      cmd,
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        if (ack->status != wire::Ack::Status_Ok)
        {
            CRL_DEBUG("Start streams ack invalid: %" PRIi32 "\n", ack->status);
            return get_status(ack->status);
        }

        for (const auto &source : sources)
        {
            m_active_streams.erase(source);
        }

        return Status::OK;
    }

    return Status::TIMEOUT;
}

void LegacyChannel::add_image_frame_callback(std::function<void(const ImageFrame&)> callback)
{
    std::lock_guard<std::mutex> lock(m_image_callback_mutex);

    m_user_image_frame_callback = callback;
}

void LegacyChannel::add_imu_frame_callback(std::function<void(const ImuFrame&)> callback)
{
    std::lock_guard<std::mutex> lock(m_imu_callback_mutex);

    m_user_imu_frame_callback = callback;
}

Status LegacyChannel::connect(const Config &config)
{
    using namespace crl::multisense::details;

    if (m_connected)
    {
        CRL_DEBUG("Channel is already connected to the MultiSense\n");
        return Status::FAILED;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    //
    // Setup networking
    //
#if WIN32
    WSADATA wsaData;
    int result = WSAStartup (MAKEWORD (0x02, 0x02), &wsaData);
    if (result != 0)
    {
        CRL_EXCEPTION("WSAStartup() failed: %d\n", result);
    }

#endif

    m_socket.sensor_address = get_sockaddr(config.ip_address, config.command_port);

    auto [sensor_socket, server_socket_port] = bind(config.interface);
    m_socket.sensor_socket = sensor_socket;
    m_socket.server_socket_port = server_socket_port;

    //
    // Attach image callbacks to handle incoming image data
    //
    m_message_assembler.register_callback(wire::ImageMeta::ID,
                                          std::bind(&LegacyChannel::image_meta_callback, this, std::placeholders::_1));

    m_message_assembler.register_callback(wire::Image::ID,
                                          std::bind(&LegacyChannel::image_callback, this, std::placeholders::_1));

    m_message_assembler.register_callback(wire::Disparity::ID,
                                          std::bind(&LegacyChannel::disparity_callback, this, std::placeholders::_1));

    m_message_assembler.register_callback(wire::ImuData::ID,
                                          std::bind(&LegacyChannel::imu_callback, this, std::placeholders::_1));

    //
    // Main thread which services incoming data and dispatches to callbacks and conditions attached to the channel
    //
    m_udp_receiver = std::make_unique<UdpReceiver>(m_socket, MAX_MTU,
                                                   [this](const std::vector<uint8_t>& data)
                                                   {
                                                       if (!this->m_message_assembler.process_packet(data))
                                                       {
                                                           CRL_DEBUG("Processing packet of %" PRIu64 " bytes failed\n", 
                                                                     static_cast<uint64_t>(data.size()));
                                                       }
                                                   });
    //
    // Set the user requested MTU
    //
    if (const auto status = set_mtu(config.mtu); status != Status::OK)
    {
        CRL_DEBUG("Unable to set MTU: %s\n", to_string(status).c_str());
        return status;
    }

    //
    // Update our cached calibration
    //
    if (auto calibration = query_calibration(); calibration)
    {
        m_calibration = std::move(calibration.value());
    }
    else
    {
        CRL_EXCEPTION("Unable to query the camera's calibration");
    }

    //
    // Update our cached system info
    //
    if (auto info = query_info(); info)
    {
        m_info = std::move(info.value());
    }
    else
    {
        CRL_EXCEPTION("Unable to query the camera's info ");
    }

    //
    // Update our cached multisense configuration
    //
    if (auto cam_config = query_configuration(m_info.device.has_aux_camera(), m_info.imu.has_value(), false); cam_config)
    {
        m_multisense_config = std::move(cam_config.value());
    }
    else
    {
        CRL_EXCEPTION("Unable to query the camera's configuraton");
    }

    m_connected = true;

    return Status::OK;
}

void LegacyChannel::disconnect()
{
    using namespace crl::multisense::details;

    if (!m_connected)
    {
        return;
    }

    //
    // Stop all our streams before disconnecting
    //
    stop_streams({DataSource::ALL});

    std::lock_guard<std::mutex> lock(m_mutex);

    m_connected = false;

    m_message_assembler.remove_callback(wire::Image::ID);

    m_socket = NetworkSocket{};

    m_udp_receiver = nullptr;

    return;
}

std::optional<ImageFrame> LegacyChannel::get_next_image_frame()
{
    return m_image_frame_notifier.wait(m_config.receive_timeout);
}

std::optional<ImuFrame> LegacyChannel::get_next_imu_frame()
{
    return m_imu_frame_notifier.wait(m_config.receive_timeout);
}

MultiSenseConfig LegacyChannel::get_configuration()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_multisense_config;
}

Status LegacyChannel::set_configuration(const MultiSenseConfig &config)
{
    using namespace crl::multisense::details;

    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<Status> responses{};

    //
    // Set the camera resolution
    //
    const auto res_ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      convert<wire::CamSetResolution>(config),
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout);

    if (!res_ack || res_ack->status != wire::Ack::Status_Ok)
    {
        responses.push_back(get_status(res_ack->status));
    }

    //
    // Set the generic camera controls
    //
    const auto cam_ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      convert<wire::CamControl>(config),
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout);

    if (!cam_ack || cam_ack->status != wire::Ack::Status_Ok)
    {
        responses.push_back(get_status(cam_ack->status));
    }

    //
    // Set aux controls if they are valid
    //
    if (config.aux_config && m_info.device.has_aux_camera())
    {
        //
        // Set the aux camera controls
        //
        const auto aux_ack = wait_for_ack(m_message_assembler,
                                          m_socket,
                                          convert(config.aux_config.value()),
                                          m_transmit_id++,
                                          m_current_mtu,
                                          m_config.receive_timeout);
        if (!aux_ack || aux_ack->status != wire::Ack::Status_Ok)
        {
            responses.push_back(get_status(aux_ack->status));
        }
    }

    //
    // Set imu controls if they are valid
    //
    if (config.imu_config && m_info.imu)
    {
        //
        // Set the imu controls
        //
        const auto imu_ack = wait_for_ack(m_message_assembler,
                                          m_socket,
                                          convert(config.imu_config.value(),
                                                  m_info.imu.value(),
                                                  m_max_batched_imu_messages),
                                          m_transmit_id++,
                                          m_current_mtu,
                                          m_config.receive_timeout);
        if (!imu_ack || imu_ack->status != wire::Ack::Status_Ok)
        {
            responses.push_back(get_status(imu_ack->status));
        }
    }

    //
    // Set lighting controls if they are valid
    //
    if (config.lighting_config)
    {
        //
        // Set the lighting controls
        //
        const auto lighting_ack = wait_for_ack(m_message_assembler,
                                               m_socket,
                                               convert(config.lighting_config.value()),
                                               m_transmit_id++,
                                               m_current_mtu,
                                               m_config.receive_timeout);
        if (!lighting_ack || lighting_ack->status != wire::Ack::Status_Ok)
        {
            responses.push_back(get_status(lighting_ack->status));
        }
    }

    //
    // Enable/disable ptp
    //
    if (config.time_config)
    {
        const auto ptp_ack = wait_for_ack(m_message_assembler,
                                          m_socket,
                                          convert(config.time_config.value()),
                                          m_transmit_id++,
                                          m_current_mtu,
                                          m_config.receive_timeout);

        if (!ptp_ack || ptp_ack->status != wire::Ack::Status_Ok)
        {
            responses.push_back(get_status(ptp_ack->status));
        }
    }

    //
    // Set our packet delay
    //
    if (config.network_config)
    {
        const auto packet_ack = wait_for_ack(m_message_assembler,
                                             m_socket,
                                             convert<wire::SysPacketDelay>(config.network_config.value()),
                                             m_transmit_id++,
                                             m_current_mtu,
                                             m_config.receive_timeout);

        if (!packet_ack || packet_ack->status != wire::Ack::Status_Ok)
        {
            responses.push_back(get_status(packet_ack->status));
        }
    }

    const auto errors = std::any_of(std::begin(responses), std::end(responses),
                                    [](const auto &e)
                                    {
                                        return (e == Status::INTERNAL_ERROR ||
                                                e == Status::FAILED ||
                                                e == Status::EXCEPTION);
                                    });

    if (errors)
    {
        return Status::INTERNAL_ERROR;
    }

    //
    // Update our internal cached image config after we successfully set everything
    //
    const auto ptp_enabled = config.time_config && config.time_config->ptp_enabled;
    if (const auto new_config = query_configuration(m_info.device.has_aux_camera(),
                                                    m_info.imu.has_value(),
                                                    ptp_enabled); new_config)
    {
        m_multisense_config = new_config.value();

        //
        // If our input does not match our queried output it means the camera does not support one of
        // our configuration settings
        //
        if (config == new_config.value())
        {
            return Status::OK;
        }

        return Status::INCOMPLETE_APPLICATION;
    }

    return Status::INTERNAL_ERROR;
}

StereoCalibration LegacyChannel::get_calibration()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_calibration;
}

Status LegacyChannel::set_calibration(const StereoCalibration &calibration)
{
    using namespace crl::multisense::details;

    const wire::SysCameraCalibration wire_calibration = convert(calibration);

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      wire_calibration,
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        //
        // If we successfully set the calibration re-query it to update our internal cached value
        //
        if (ack->status == wire::Ack::Status_Ok)
        {
            if (const auto new_cal = query_calibration(); new_cal)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_calibration = new_cal.value();
            }
        }

        return get_status(ack->status);
    }

    return Status::TIMEOUT;
}

MultiSenseInfo LegacyChannel::get_info()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_info;
}

Status LegacyChannel::set_device_info(const MultiSenseInfo::DeviceInfo &device_info, const std::string &key)
{
    using namespace crl::multisense::details;

    const wire::SysDeviceInfo wire_device_info = convert(device_info, key);

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      wire_device_info,
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        //
        // If we successfully set the device info re-query it to update our internal cached value
        //
        if (ack->status == wire::Ack::Status_Ok)
        {
            if (const auto new_device_info = query_device_info(); new_device_info)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_info.device = new_device_info.value();
            }
        }

        return get_status(ack->status);
    }

    return Status::TIMEOUT;
}

std::optional<MultiSenseStatus> LegacyChannel::get_system_status()
{
    using namespace crl::multisense::details;

    //
    // Query the main status info, and time when we send the ack, and when we receive the response
    //
    const auto status = wait_for_data_timed<wire::StatusResponse>(m_message_assembler,
                                                                  m_socket,
                                                                  wire::StatusRequest(),
                                                                  m_transmit_id++,
                                                                  m_current_mtu,
                                                                  m_config.receive_timeout);

    if (!status)
    {
        CRL_DEBUG("Unable to query status\n");
        return std::nullopt;
    }

    std::optional<wire::PtpStatusResponse> ptp_status = std::nullopt;
    if (m_multisense_config.time_config && m_multisense_config.time_config->ptp_enabled)
    {

        if (ptp_status = wait_for_data<wire::PtpStatusResponse>(m_message_assembler,
                                                                       m_socket,
                                                                       wire::PtpStatusRequest(),
                                                                       m_transmit_id++,
                                                                       m_current_mtu,
                                                                       m_config.receive_timeout); !ptp_status)
        {
            CRL_DEBUG("Unable to query ptp status\n");
            return std::nullopt;
        }
    }

    //
    // To compute our network delay (i.e. offset between client host time and the camera uptime take our
    // total roundtrip and divide by 2 to account for the transmit/receive
    //
    MultiSenseStatus::TimeStatus time_status{std::chrono::nanoseconds{status->message.uptime.getNanoSeconds()},
                                             status->host_start_transmit_time,
                                             status->host_transmit_receive_roundtrip / 2};

    const auto message_stats = m_message_assembler.get_message_statistics();
    MultiSenseStatus::ClientNetworkStatus client_stats{message_stats.received_messages,
                                                       message_stats.dropped_messages,
                                                       message_stats.invalid_packets};

    return MultiSenseStatus{system_ok(status->message),
                            (ptp_status ? std::make_optional(convert(ptp_status.value())) : std::nullopt),
                            convert<MultiSenseStatus::CameraStatus>(status->message),
                            convert<MultiSenseStatus::TemperatureStatus>(status->message),
                            convert<MultiSenseStatus::PowerStatus>(status->message),
                            std::move(client_stats),
                            std::move(time_status)};
}

Status LegacyChannel::set_network_configuration(const MultiSenseInfo::NetworkInfo &config)
{
    using namespace crl::multisense::details;

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      convert(config),
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        return get_status(ack->status);
    }

    return Status::TIMEOUT;
}

Status LegacyChannel::set_mtu(uint16_t mtu)
{
    using namespace crl::multisense::details;

    if (const auto test_mtu = wait_for_data<wire::SysTestMtuResponse>(m_message_assembler,
                                                                      m_socket,
                                                                      wire::SysTestMtu(mtu),
                                                                      m_transmit_id++,
                                                                      m_current_mtu,
                                                                      m_config.receive_timeout); !test_mtu)
    {
        CRL_DEBUG("Testing MTU of %" PRIu16 " bytes failed."
                  " Please verify you can ping the MultiSense with a MTU of %" PRIu16 " bytes at %s.\n",
                  mtu, mtu, inet_ntoa(m_socket.sensor_address->sin_addr));
        return Status::INTERNAL_ERROR;
    }

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      wire::SysMtu(mtu),
                                      m_transmit_id++,
                                      m_current_mtu,
                                      m_config.receive_timeout); ack)
    {
        if (ack->status != wire::Ack::Status_Ok)
        {
            CRL_DEBUG("Unable to set MTU to %" PRIu16 " bytes: %i\n", mtu, ack->status);
            return get_status(ack->status);
        }
    }

    m_current_mtu = mtu;
    return Status::OK;
}

Status LegacyChannel::set_mtu(const std::optional<uint16_t> &mtu)
{
    using namespace crl::multisense::details;

    if (mtu)
    {
        return set_mtu(mtu.value());
    }
    else
    {
        for (const auto &value : MTUS_TO_TEST)
        {
            if (const auto status = set_mtu(value); status == Status::OK)
            {
                CRL_DEBUG("Auto-setting MTU to %" PRIu16 " bytes \n", value);
                return status;
            }
        }
        CRL_DEBUG("Unable to find a MTU that works with the MultiSense. "
                  "Please verify you can ping the MultiSense at %s\n", inet_ntoa(m_socket.sensor_address->sin_addr));
    }

    return Status::INTERNAL_ERROR;
}

std::optional<MultiSenseConfig> LegacyChannel::query_configuration(bool has_aux_camera,
                                                                   bool has_imu,
                                                                   bool ptp_enabled)
{
    using namespace crl::multisense::details;

    const auto camera_config = wait_for_data<wire::CamConfig>(m_message_assembler,
                                                              m_socket,
                                                              wire::CamGetConfig(),
                                                              m_transmit_id++,
                                                              m_current_mtu,
                                                              m_config.receive_timeout);

    const auto aux_config = has_aux_camera ? wait_for_data<wire::AuxCamConfig>(m_message_assembler,
                                                                               m_socket,
                                                                               wire::AuxCamGetConfig(),
                                                                               m_transmit_id++,
                                                                               m_current_mtu,
                                                                               m_config.receive_timeout): std::nullopt;

    const auto imu_config = has_imu ? wait_for_data<wire::ImuConfig>(m_message_assembler,
                                                                     m_socket,
                                                                     wire::ImuGetConfig(),
                                                                     m_transmit_id++,
                                                                     m_current_mtu,
                                                                     m_config.receive_timeout): std::nullopt;


    const auto led_config = wait_for_data<wire::LedStatus>(m_message_assembler,
                                                           m_socket,
                                                           wire::LedGetStatus(),
                                                           m_transmit_id++,
                                                           m_current_mtu,
                                                           m_config.receive_timeout);

    const auto packet_delay = wait_for_data<wire::SysPacketDelay>(m_message_assembler,
                                                                  m_socket,
                                                                  wire::SysGetPacketDelay(),
                                                                  m_transmit_id++,
                                                                  m_current_mtu,
                                                                  m_config.receive_timeout);

    if (camera_config)
    {
        return convert(camera_config.value(),
                       aux_config,
                       imu_config,
                       led_config,
                       packet_delay ? packet_delay.value() : wire::SysPacketDelay{false},
                       ptp_enabled,
                       m_info.device,
                       m_info.imu);
    }

    CRL_DEBUG("Unable to query the camera's configuration\n");

    return std::nullopt;
}

std::optional<StereoCalibration> LegacyChannel::query_calibration()
{
    using namespace crl::multisense::details;

    if (const auto new_cal = wait_for_data<wire::SysCameraCalibration>(m_message_assembler,
                                                                       m_socket,
                                                                       wire::SysGetCameraCalibration(),
                                                                       m_transmit_id++,
                                                                       m_current_mtu,
                                                                       m_config.receive_timeout); new_cal)
    {
        return std::make_optional(convert(new_cal.value()));
    }

    return std::nullopt;
}

std::optional<MultiSenseInfo> LegacyChannel::query_info()
{
    using namespace crl::multisense::details;

    const auto device_info = query_device_info();

    if (!device_info)
    {
        CRL_DEBUG("Unable to query the device info\n");
        return std::nullopt;
    }

    const auto version = wait_for_data<wire::VersionResponse>(m_message_assembler,
                                                              m_socket,
                                                              wire::VersionRequest(),
                                                              m_transmit_id++,
                                                              m_current_mtu,
                                                              m_config.receive_timeout);
    if (!version)
    {
        CRL_DEBUG("Unable to query the version info\n");
        return std::nullopt;
    }

    const auto device_modes = wait_for_data<wire::SysDeviceModes>(m_message_assembler,
                                                                  m_socket,
                                                                  wire::SysGetDeviceModes(),
                                                                  m_transmit_id++,
                                                                  m_current_mtu,
                                                                  m_config.receive_timeout);

    if (!device_modes)
    {
        CRL_DEBUG("Unable to query the device modes\n");
        return std::nullopt;
    }

    const auto imu_info = wait_for_data<wire::ImuInfo>(m_message_assembler,
                                                       m_socket,
                                                       wire::ImuGetInfo(),
                                                       m_transmit_id++,
                                                       m_current_mtu,
                                                       m_config.receive_timeout);

    if (imu_info)
    {
        m_max_batched_imu_messages = imu_info->maxSamplesPerMessage;
        m_imu_scalars = get_imu_scalars(imu_info.value());
    }

    const auto network_info = wait_for_data<wire::SysNetwork>(m_message_assembler,
                                                              m_socket,
                                                              wire::SysGetNetwork(),
                                                              m_transmit_id++,
                                                              m_current_mtu,
                                                              m_config.receive_timeout);

    if (!network_info)
    {
        CRL_DEBUG("Unable to query the network info\n");
        return std::nullopt;
    }

    return MultiSenseInfo{device_info.value(),
                          convert(version.value()),
                          convert(device_modes.value()),
                          imu_info ? std::make_optional(convert(imu_info.value())) : std::nullopt,
                          convert(network_info.value())};
}

std::optional<MultiSenseInfo::DeviceInfo> LegacyChannel::query_device_info()
{
    using namespace crl::multisense::details;

    if (const auto device_info = wait_for_data<wire::SysDeviceInfo>(m_message_assembler,
                                                                    m_socket,
                                                                    wire::SysGetDeviceInfo(),
                                                                    m_transmit_id++,
                                                                    m_current_mtu,
                                                                    m_config.receive_timeout); device_info)
    {
        return std::make_optional(convert(device_info.value()));
    }

    return std::nullopt;
}

void LegacyChannel::image_meta_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;

    const auto wire_meta = deserialize<wire::ImageMeta>(*data);

    m_meta_cache[wire_meta.frameId] = wire_meta;
}

void LegacyChannel::image_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;
    using namespace std::chrono;

    const auto wire_image = deserialize<wire::Image>(*data);

    const auto meta = m_meta_cache.find(wire_image.frameId);
    if (meta == std::end(m_meta_cache))
    {
        CRL_DEBUG("Missing corresponding meta for frame_id %" PRIu64 "\n", wire_image.frameId);
        return;
    }

    const nanoseconds capture_time{seconds{meta->second.timeSeconds} +
                                  microseconds{meta->second.timeMicroSeconds}};

    const TimeT capture_time_point{capture_time};

    const TimeT ptp_capture_time_point{nanoseconds{meta->second.ptpNanoSeconds}};

    Image::PixelFormat pixel_format = Image::PixelFormat::UNKNOWN;
    switch (wire_image.bitsPerPixel)
    {
        case 8: {pixel_format = Image::PixelFormat::MONO8; break;}
        case 16: {pixel_format = Image::PixelFormat::MONO16; break;}
        default: {CRL_DEBUG("Unknown pixel format %" PRIu32 "\n", wire_image.bitsPerPixel);}
    }

    const auto source = convert_sources(static_cast<uint64_t>(wire_image.sourceExtended) << 32 | wire_image.source);
    if (source.size() != 1)
    {
        CRL_DEBUG("invalid image source\n");
        return;
    }

    //
    // Copy our calibration and device info locally to make this thread safe
    //
    StereoCalibration cal;
    MultiSenseInfo::DeviceInfo info;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cal = m_calibration;
        info = m_info.device;
    }

    const auto cal_x_scale = static_cast<double>(wire_image.width) / static_cast<double>(info.imager_width);
    const auto cal_y_scale = static_cast<double>(wire_image.height) / static_cast<double>(info.imager_height);

    Image image{data,
                static_cast<const uint8_t*>(wire_image.dataP) - data->data(),
                ((wire_image.bitsPerPixel / 8) * wire_image.width * wire_image.height),
                pixel_format,
                wire_image.width,
                wire_image.height,
                capture_time_point,
                ptp_capture_time_point,
                source.front(),
                scale_calibration(select_calibration(cal, source.front()), cal_x_scale, cal_y_scale)};

    handle_and_dispatch(std::move(image),
                        wire_image.frameId,
                        scale_calibration(cal, cal_x_scale, cal_y_scale),
                        capture_time_point,
                        ptp_capture_time_point);
}

void LegacyChannel::disparity_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;
    using namespace std::chrono;

    const auto wire_image = deserialize<wire::Disparity>(*data);

    const auto meta = m_meta_cache.find(wire_image.frameId);
    if (meta == std::end(m_meta_cache))
    {
        CRL_DEBUG("Missing corresponding meta for frame_id %" PRIu64 "\n", wire_image.frameId);
        return;
    }

    const nanoseconds capture_time{seconds{meta->second.timeSeconds} +
                                  microseconds{meta->second.timeMicroSeconds}};

    const TimeT capture_time_point{capture_time};

    const TimeT ptp_capture_time_point{nanoseconds{meta->second.ptpNanoSeconds}};

    Image::PixelFormat pixel_format = Image::PixelFormat::UNKNOWN;
    switch (wire::Disparity::API_BITS_PER_PIXEL)
    {
        case 8: {pixel_format = Image::PixelFormat::MONO8; break;}
        case 16: {pixel_format = Image::PixelFormat::MONO16; break;}
        default: {CRL_DEBUG("Unknown pixel format %" PRIu8 "\n", wire::Disparity::API_BITS_PER_PIXEL);}
    }

    const auto source = DataSource::LEFT_DISPARITY_RAW;

    const auto disparity_length =
        static_cast<size_t>(((static_cast<double>(wire::Disparity::API_BITS_PER_PIXEL) / 8.0) *
                            wire_image.width *
                            wire_image.height));

    //
    // Copy our calibration and device info locally to make this thread safe
    //
    StereoCalibration cal;
    MultiSenseInfo::DeviceInfo info;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cal = m_calibration;
        info = m_info.device;
    }

    const auto cal_x_scale = static_cast<double>(wire_image.width) / static_cast<double>(info.imager_width);
    const auto cal_y_scale = static_cast<double>(wire_image.height) / static_cast<double>(info.imager_height);

    Image image{data,
                static_cast<const uint8_t*>(wire_image.dataP) - data->data(),
                disparity_length,
                pixel_format,
                wire_image.width,
                wire_image.height,
                capture_time_point,
                ptp_capture_time_point,
                source,
                scale_calibration(select_calibration(cal, source), cal_x_scale, cal_y_scale)};

    handle_and_dispatch(std::move(image),
                        wire_image.frameId,
                        scale_calibration(cal, cal_x_scale, cal_y_scale),
                        capture_time_point,
                        ptp_capture_time_point);

}

void LegacyChannel::imu_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;

    if (!m_multisense_config.imu_config)
    {
        CRL_EXCEPTION("Invalid IMU config\n");
        return;
    }

    const auto wire_imu = deserialize<wire::ImuData>(*data);

    for (const auto &wire_sample : wire_imu.samples)
    {
        const TimeT camera_time{std::chrono::nanoseconds{wire_sample.timeNanoSeconds}};
        const TimeT ptp_time{std::chrono::nanoseconds{wire_sample.ptpNanoSeconds}};

        if (!m_current_imu_frame.samples.empty() && m_current_imu_frame.samples.back().sample_time == camera_time &&
            m_current_imu_frame.samples.back().ptp_sample_time == ptp_time)
        {
            const size_t i = m_current_imu_frame.samples.size() - 1;
            m_current_imu_frame.samples[i] = add_wire_sample(std::move(m_current_imu_frame.samples[i]),
                                                             wire_sample,
                                                             m_imu_scalars);
        }
        else
        {
            ///
            /// We have enough valid samples to notify the listener and call our user callback
            ///
            if (m_current_imu_frame.samples.size() >= m_multisense_config.imu_config->samples_per_frame)
            {
                m_imu_frame_notifier.set_and_notify(m_current_imu_frame);

                std::lock_guard<std::mutex> lock(m_imu_callback_mutex);
                if (m_user_imu_frame_callback)
                {
                    m_user_imu_frame_callback(m_current_imu_frame);
                }

                m_current_imu_frame.samples.clear();
            }

            ImuSample sample{std::nullopt, std::nullopt, std::nullopt, camera_time, ptp_time};
            sample = add_wire_sample(std::move(sample), wire_sample, m_imu_scalars);
            m_current_imu_frame.samples.push_back(std::move(sample));
        }
    }
}

void LegacyChannel::handle_and_dispatch(Image image,
                                        int64_t frame_id,
                                        const StereoCalibration &calibration,
                                        const TimeT &capture_time,
                                        const TimeT &ptp_capture_time)
{
    //
    // Create a new frame if one does not exist, or add the input image to the corresponding frame
    //
    if (m_frame_buffer.count(frame_id) == 0)
    {
        const auto source = image.source;
        ImageFrame frame{frame_id,
                         std::map<DataSource, Image>{std::make_pair(source, std::move(image))},
                         calibration,
                         capture_time,
                         ptp_capture_time};

        m_frame_buffer.emplace(frame_id, std::move(frame));
    }
    else
    {
        m_frame_buffer[frame_id].add_image(std::move(image));
    }

    //
    // Check if our frame is valid, if so dispatch to our callbacks and notify anyone who is waiting on
    // the next frame
    //
    if (const auto &frame = m_frame_buffer[frame_id];
            std::all_of(std::begin(m_active_streams),
                        std::end(m_active_streams),
                        [&frame](const auto &e){return is_image_source(e) ? frame.has_image(e) : true;}))
    {
        //
        // Notify anyone waiting on the next frame
        //
        m_image_frame_notifier.set_and_notify(frame);

        //
        // Service the callback if it's valid
        //
        {
            std::lock_guard<std::mutex> lock(m_image_callback_mutex);
            if (m_user_image_frame_callback)
            {
                m_user_image_frame_callback(frame);
            }
        }


        //
        // Remove our image frame from our frame buffer and the associated image metadata since we are
        // now done with it internally
        //
        m_frame_buffer.erase(frame_id);
        m_meta_cache.erase(frame_id);
    }

    //
    // Since frames will only monotonically increase, it's safe to also delete all the frame_ids earlier than
    // the current frame id.
    //
    m_frame_buffer.erase(std::begin(m_frame_buffer), m_frame_buffer.lower_bound(frame_id));
    m_meta_cache.erase(std::begin(m_meta_cache), m_meta_cache.lower_bound(frame_id));
}

}
}
