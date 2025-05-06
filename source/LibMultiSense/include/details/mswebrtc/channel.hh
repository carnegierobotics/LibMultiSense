/**
 * @file channel.hh
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

#include "MultiSense/MultiSenseChannel.hh"
#include "utility/Exception.hh"
#include "mswebrtc.h"

namespace multisense{
namespace mswebrtc{


class MswebrtcChannel : public Channel
{
public:
    explicit MswebrtcChannel(const Config &config);

    virtual ~MswebrtcChannel();

    ///
    /// @brief Start a collection of image streams. Repeated calls to this function will not implicitly
    ///        stop the previously started streams. For example if a user started a left_raw stream in one
    ///        call and a disparity stream in a second call, both streams would be active until stop_streams
    ///        is called for either
    ///
    Status start_streams(const std::vector<DataSource> &sources) final override;

    ///
    /// @brief Stop a collection of streams
    ///
    Status stop_streams(const std::vector<DataSource> &sources) final override;

    ///
    /// @brief Add a image frame callback to get serviced inline with the receipt of a new frame. Only
    ///        a single image frame callback can be added to the channel
    ///        NOTE: Perform minimal work in this callback, and ideally copy the lightweight
    ///        ImageFrame object out to another processing thread
    ///
    void add_image_frame_callback(std::function<void(const ImageFrame&)> callback) final override;

    ///
    /// @brief Add a imu frame callback to get serviced inline with the receipt of a new frame. Only
    ///        a single imu frame callback can be added to the channel
    ///        NOTE: Perform minimal work in this callback, and ideally copy the lightweight
    ///        ImuFrame object out to another processing thread
    ///
    void add_imu_frame_callback(std::function<void(const ImuFrame&)> callback) final override;

    ///
    /// @brief Initialize the connection to the camera
    ///
    Status connect(const Config &config) final override;

    ///
    /// @brief Disconnect from the camera
    ///
    void disconnect() final override;

    ///
    /// @brief A blocking call that waits for one image frame from the camera.
    ///
    /// If you’ve set a receive timeout (via Config), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImageFrame, or std::nullopt if timed out (and you used a timeout).
    ///
    std::optional<ImageFrame> get_next_image_frame() final override;

    ///
    /// @brief A blocking call that waits for one imu frame from the camera.
    ///
    /// If you’ve set a receive timeout (via Config), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImuFrame, or std::nullopt if timed out (and you used a timeout).
    ///
    std::optional<ImuFrame> get_next_imu_frame() final override;

    ///
    /// @brief Get the current MultiSense configuration
    ///
    MultiSenseConfig get_config() final override;

    ///
    /// @brief Get set the current MultiSense configuration
    ///
    Status set_config(const MultiSenseConfig &config) final override;

    ///
    /// @brief Get the current stereo calibration. The output calibration will correspond to the full-resolution
    ///        operating mode of the camera
    ///
    StereoCalibration get_calibration() final override;

    ///
    /// @brief Set the current stereo calibration. The calibration is expected to be or the full-resolution
    ///        operating mode of the camera
    ///
    Status set_calibration(const StereoCalibration &calibration) final override;

    ///
    /// @brief Get the device info associated with the camera
    ///
    MultiSenseInfo get_info() final override;

    ///
    /// @brief Set the camera's device info. This setting is protected via a key since invalid values in the
    ///        device info can result in internal camera failures
    ///
    Status set_device_info(const MultiSenseInfo::DeviceInfo &device_info, const std::string &key) final override;

    ///
    /// @brief Query the current MultiSense status
    ///
    std::optional<MultiSenseStatus> get_system_status() final override;

    ///
    /// @brief Update the network configuration of the MultiSense. This will require a hardware reboot of the
    ///        MultiSense after it's been successfully applied
    ///
    Status set_network_config(const MultiSenseInfo::NetworkInfo &config,
                              const std::optional<std::string> &broadcast_interface) final override;

private:
    MswebrtcImpl *impl;
};

}
}
