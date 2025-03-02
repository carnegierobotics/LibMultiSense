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

#include <set>

#include "MultiSense/MultiSenseChannel.hh"

#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>
#include <wire/ImageMetaMessage.hh>

#include "details/legacy/ip.hh"
#include "details/legacy/message.hh"
#include "details/legacy/storage.hh"
#include "details/legacy/udp.hh"
#include "details/legacy/utilities.hh"

namespace multisense{
namespace legacy{

template <typename T>
class FrameNotifier
{
public:

    FrameNotifier() = default;

    ~FrameNotifier()
    {
        m_cv.notify_all();
    }

    ///
    /// @brief Copy a frame into the local storage, and notify all waiters that the frame is valid
    ///
    void set_and_notify(const T &in_frame)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_frame = in_frame;
        m_cv.notify_all();
    }

    ///
    /// @brief Wait for the notifier to be valid. If the timeout is invalid, will wait forever
    ///
    template <class Rep, class Period>
    std::optional<T> wait(const std::optional<std::chrono::duration<Rep, Period>>& timeout)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        std::optional<T> output_frame = std::nullopt;
        if (timeout)
        {
            if (std::cv_status::no_timeout == m_cv.wait_for(lock, timeout.value()))
            {
                output_frame = std::move(m_frame);
            }
        }
        else
        {
            m_cv.wait(lock);
            output_frame = std::move(m_frame);
        }

        //
        // Reset the frame
        //
        m_frame = std::nullopt;

        return output_frame;
    }

    std::optional<T> wait()
    {
        const std::optional<std::chrono::milliseconds> timeout = std::nullopt;
        return wait(timeout);
    }

private:

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::optional<T> m_frame;
};


class LegacyChannel : public Channel
{
public:
    explicit LegacyChannel(const Config &config);

    virtual ~LegacyChannel();

    ///
    /// @brief Start a collection of image streams. Repeated calls to this function will not stop implicitly
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
    ///        a single frame callback can be added to the channel
    ///        NOTE: Perform minimal work in this callback, and ideally copy the lightweight
    ///        ImageFrame object out to another processing thread
    ///
    void add_image_frame_callback(std::function<void(const ImageFrame&)> callback) final override;

    ///
    /// @brief Add a imu frame callback to get serviced inline with the receipt of a new frame. Only
    ///        a single frame callback can be added to the channel
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
    MultiSenseConfig get_configuration() final override;

    ///
    /// @brief Get set the current MultiSense configuration
    ///
    Status set_configuration(const MultiSenseConfig &config) final override;

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
    ///        MultiSense after it's been succeffully applied
    ///
    Status set_network_configuration(const MultiSenseInfo::NetworkInfo &config) final override;

private:

    ///
    /// @brief Try and set the MTU
    ///
    Status set_mtu(uint16_t mtu);

    ///
    /// @brief Set the MTU. If the MTU is invalid, try and find the best MTU via a basic search
    ///
    Status set_mtu(const std::optional<uint16_t> &mtu);

    ///
    /// @brief Query the full configuration
    ///
    std::optional<MultiSenseConfig> query_configuration(bool has_aux_camera, bool has_imu, bool ptp_enabled);

    ///
    /// @brief Query the calibration from the camera
    ///
    std::optional<StereoCalibration> query_calibration();

    ///
    /// @brief Query the MultiSense Info
    ///
    std::optional<MultiSenseInfo> query_info();

    ///
    /// @brief Query the device_info from the camera
    ///
    std::optional<MultiSenseInfo::DeviceInfo> query_device_info();

    ///
    /// @brief Image meta callback used to internally receive images sent from the MultiSense
    ///
    void image_meta_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Image callback used to internally receive images sent from the MultiSense
    ///
    void image_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Disparity callback used to internally receive images sent from the MultiSense
    ///
    void disparity_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Disparity callback used to internally receive images sent from the MultiSense
    ///
    void imu_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Handle internal process, and potentially dispatch a image
    ///
    void handle_and_dispatch(Image image,
                            int64_t frame_id,
                            const StereoCalibration &calibration,
                            const TimeT &capture_time,
                            const TimeT &ptp_capture_time);

    ///
    /// @brief Internal mutex used to handle updates from users
    ///
    std::mutex m_mutex{};

    ///
    /// @brief Internal mutex used to handle user callbacks for image data
    ///
    std::mutex m_image_callback_mutex{};

    ///
    /// @brief Internal mutex used to handle user callbacks imu data
    ///
    std::mutex m_imu_callback_mutex{};

    ///
    /// @brief Atomic flag to determine if we are connected to an active camera
    ///
    std::atomic_bool m_connected = false;

    ///
    /// @brief The current MTU the camera is operating with
    ///
    std::atomic_uint16_t m_current_mtu = 1500;

    ///
    /// @brief Channel config
    ///
    Config m_config{};

    ///
    /// @brief Active network socket for receiving and transmitting data
    ///
    NetworkSocket m_socket{};

    ///
    /// @brief Monotonically increasing internal id used to uniquely identify requests sent to the camera
    ///
    std::atomic_uint16_t m_transmit_id = 0;

    ///
    /// @brief The current cached calibration stored here for convenience
    ///
    StereoCalibration m_calibration{};

    ///
    /// @brief The current cached device info stored here for convenience
    ///
    MultiSenseInfo m_info{};

    ///
    /// @brief The current cached MultiSense configuration stored for convenience
    ///
    MultiSenseConfig m_multisense_config{};

    ///
    /// @brief The current set of active data streams the MultiSense is transmitting.
    ///
    std::set<DataSource> m_active_streams{};

    ///
    /// @brief The currently active image frame user callback
    ///
    std::function<void(const ImageFrame&)> m_user_image_frame_callback{};

    ///
    /// @brief The currently active imu frame user callback
    ///
    std::function<void(const ImuFrame&)> m_user_imu_frame_callback{};

    ///
    /// @brief Notifier used to service the get_next_image_frame member function
    ///
    FrameNotifier<ImageFrame> m_image_frame_notifier{};

    ///
    /// @brief Notifier used to service the get_next_imu_frame member function
    ///
    FrameNotifier<ImuFrame> m_imu_frame_notifier{};

    ///
    /// @brief A cache of image metadata associated with a specific frame id
    ///
    std::map<int64_t, crl::multisense::details::wire::ImageMeta> m_meta_cache{};

    ///
    /// @brief A cache of image frames associated with a specific frame id
    ///
    std::map<int64_t, ImageFrame> m_frame_buffer{};

    ///
    /// @brief The max number of IMU messages which can be batched over the wire
    ///
    std::atomic_uint32_t m_max_batched_imu_messages = 0;

    ///
    /// @brief Scalars to convert imu samples from wire units to standard LibMultiSense units
    ///
    ImuSampleScalars m_imu_scalars{};

    ///
    /// @brief A cache of IMU samples which will be internally filled until it reaches the
    ///        sample threshold for dispatch
    ///
    ImuFrame m_current_imu_frame{};

    ///
    /// @brief A collection of buffers to avoid dynamic allocation for incoming messages
    ///
    std::shared_ptr<BufferPool> m_buffer_pool = nullptr;

    ///
    /// @brief Helper object to receive UDP traffic. Internally manages a receive thread
    ///
    std::unique_ptr<UdpReceiver> m_udp_receiver = nullptr;

    ///
    /// @brief Helper object to assemble raw UDP packets into complete MultiSense wire messages
    ///
    MessageAssembler m_message_assembler;
};

}
}
