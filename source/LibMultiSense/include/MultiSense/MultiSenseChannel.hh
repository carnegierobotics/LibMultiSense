/**
 * @file MultiSenseChannel.hh
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

#pragma once

#include <functional>
#include <optional>

#include "MultiSenseTypes.hh"

namespace multisense {

class MULTISENSE_API Channel {
public:
    ///
    /// @brief Identifiers for the different Channel implementations. Used for
    ///        selecting a implementation at runtime
    ///
    enum class ChannelImplementation
    {
        ///
        /// @brief Use the Legacy MultiSense wire protocol implemented as part of LibMultiSense
        ///
        LEGACY
    };

    ///
    /// @brief Certain implementations may use a fixed set of internal buffers to manage
    ///        incoming camera data. For those implementations specify configurations for
    ///        both small and large buffers
    ///
    struct ReceiveBufferConfig
    {
        ///
        /// @brief The number of small buffers to preallocate for receiving small MultiSense messages
        ///
        size_t num_small_buffers = 100;
        ///
        /// @brief The size of each small buffer in bytes
        ///
        size_t small_buffer_size = 1500;
        ///
        /// @brief The number of large buffers to preallocate for receiving MultiSense sensor data
        ///
        size_t num_large_buffers = 32;
        ///
        /// @brief The size of each small buffer in bytes
        ///
        size_t large_buffer_size = 1920*1200*3;
    };

    struct Config
    {
        ///
        /// @brief the IP address of the MultiSense camera to connect to
        ///
        std::string ip_address = "10.66.171.21";

        ///
        /// @brief The MTU to use for sending and receiving data from the camera. Setting the MTU to nullopt
        ///        will trigger a automatic search for the largest available MTU. For more information
        ///        on MTU see: https://docs.carnegierobotics.com/network/network.html#mtu
        ///
        std::optional<int16_t> mtu = static_cast<uint16_t>(1500);

        ///
        /// @brief Timeout to use when waiting for MultiSense command responses. Setting the timeout to nullopt
        ///        will result in the camera waiting forever for a command response
        ///
        std::optional<std::chrono::milliseconds> receive_timeout = std::chrono::milliseconds(500);

        ///
        /// @brief The UDP port on the MultiSense which accepts user commands. All production firmware builds use
        ///        port 9001
        ///
        uint16_t command_port{9001};

        ///
        /// @brief An optional name of network interface to bind to. (i.e. eth0)
        ///
        std::optional<std::string> interface = std::nullopt;

        ///
        /// @brief Config for the number and size of internal buffers used to receive data without
        ///        recurring memory allocations. May only be valid for certain implementations
        ///
        ReceiveBufferConfig receive_buffer_configuration{};

        ///
        /// @brief Connect to the camera when the Channel is initialized
        ///
        bool connect_on_initialization = true;
    };

    ///
    /// @brief Factory create function which allows for switching between different channel
    ///        implementations
    ///
    static std::unique_ptr<Channel> create(const Config &config,
                                           const ChannelImplementation &impl = ChannelImplementation::LEGACY);

    Channel() = default;

    ///
    /// Non-copyable
    ///
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    ///
    /// Movable
    ///
    Channel(Channel&&) noexcept = default;
    Channel& operator=(Channel&&) noexcept = default;

    virtual ~Channel() = default;

    ///
    /// @brief Start a collection of data sources streaming from the camera.
    ///
    virtual Status start_streams(const std::vector<DataSource> &sources) = 0;

    ///
    /// @brief Stop specific data sources from streaming from the camera. An empty
    ///        collection of sources will stop all sources
    ///
    virtual Status stop_streams(const std::vector<DataSource> &sources) = 0;

    ///
    /// @brief Setup user callback that will be invoked whenever a new image frame is received.
    ///
    virtual void add_image_frame_callback(std::function<void(const ImageFrame&)> callback) = 0;

    ///
    /// @brief Setup user callback that will be invoked whenever a new imu frame is received.
    ///
    virtual void add_imu_frame_callback(std::function<void(const ImuFrame&)> callback) = 0;

    ///
    /// @brief Initialize the connection to the camera
    ///
    virtual Status connect(const Config &config) = 0;

    ///
    /// @brief Disconnect from the camera
    ///
    virtual void disconnect() = 0;

    ///
    /// @brief A blocking call that waits for one image frame from the camera.
    ///
    /// If you’ve set a receive timeout (via Config), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImageFrame, or std::nullopt if timed out (and you used a timeout).
    ///
    virtual std::optional<ImageFrame> get_next_image_frame() = 0;

    ///
    /// @brief A blocking call that waits for one imu frame from the camera.
    ///
    /// If you’ve set a receive timeout (via Config), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImuFrame, or std::nullopt if timed out (and you used a timeout).
    ///
    virtual std::optional<ImuFrame> get_next_imu_frame() = 0;

    ///
    /// @brief Get the current MultiSense configuration
    ///
    virtual MultiSenseConfig get_config() = 0;

    ///
    /// @brief Get set the current MultiSense configuration
    ///
    virtual Status set_config(const MultiSenseConfig &config) = 0;

    ///
    /// @brief Get the current stereo calibration. The output calibration will correspond to the full-resolution
    ///        operating mode of the camera
    ///
    virtual StereoCalibration get_calibration() = 0;

    ///
    /// @brief Set the current stereo calibration. The calibration is expected to be or the full-resolution
    ///        operating mode of the camera
    ///
    virtual Status set_calibration(const StereoCalibration &calibration) = 0;

    ///
    /// @brief Get the static information associated with the camera
    ///
    virtual MultiSenseInfo get_info() = 0;

    ///
    /// @brief Set the camera's device info. This setting is protected via a key since invalid values in the
    ///        device info can result in internal camera failures
    ///
    virtual Status set_device_info(const MultiSenseInfo::DeviceInfo &device_info, const std::string &key) = 0;

    ///
    /// @brief Query the current system status
    ///
    virtual std::optional<MultiSenseStatus> get_system_status() = 0;

    ///
    /// @brief Update the network configuration of the MultiSense. This will require a hardware reboot of the
    ///        MultiSense after it's been successfully applied.
    ///
    /// @param broadcast_interface If specified a broadcast packet will be sent over the specified interface
    ///                            changing the IP address of all the camera connected to that interface
    ///
    virtual Status set_network_config(const MultiSenseInfo::NetworkInfo &config,
                                      const std::optional<std::string> &broadcast_interface) = 0;
};

}
