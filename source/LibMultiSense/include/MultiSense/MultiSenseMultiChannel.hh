/**
 * @file MultiSenseMultiChannel.hh
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
 *   2025-12-02, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

#include "MultiSenseChannel.hh"

namespace multisense {

///
/// @brief Free function which determines if a collection of frames are synchronized within a given tolerance
///
/// @param frames Frames to check for synchronization
/// @param tolerance The max time difference between any frames for them to be considered valid
///
/// @return Return true if the frames are synchronized
///
MULTISENSE_API bool frames_synchronized(const std::vector<ImageFrame> &frames, const std::chrono::nanoseconds &tolerance);

///
/// @brief Helper class which provides a interface to synchronize data across multiple channels.
///
class MULTISENSE_API MultiChannelSynchronizer {
public:

   ///
   /// @brief Construct a synchronizer owning the underlying channels
   ///
   /// @param channels The channels to synchronize
   /// @param tolerance The max time difference for a set of images to be considered synchronized
   /// @param max_queue_size The max number of synchronized frames to have queued for dispatch
   ///
   explicit MultiChannelSynchronizer(std::vector<std::unique_ptr<Channel>> channels,
                                     const std::chrono::nanoseconds &tolerance,
                                     size_t max_queue_size = 0):
        m_owned_channels(std::move(channels)),
        m_tolerance(tolerance),
        m_max_queue_size(max_queue_size)
    {
        for (auto &channel : m_owned_channels)
        {
            const auto config = channel->get_config();

            if (!config.time_config || !config.time_config->ptp_enabled)
            {
                throw std::runtime_error("Creating a MultiChannelSynchronizer with PTP disabled");
            }

            m_channels.push_back(channel.get());
        }

        m_active_frames.resize(m_channels.size());
        add_user_callbacks();
    }

   ///
   /// @brief Construct a synchronizer without owning the underlying channels
   ///
   /// @param channels The channels to synchronize
   /// @param tolerance The max time difference for a set of images to be considered synchronized
   /// @param max_queue_size The max number of synchronized frames to have queued for dispatch
   ///
   explicit MultiChannelSynchronizer(std::vector<Channel*> channels,
                                     const std::chrono::nanoseconds &tolerance,
                                     size_t max_queue_size = 0):
        m_channels(std::move(channels)),
        m_tolerance(tolerance),
        m_max_queue_size(max_queue_size)
    {
        m_active_frames.resize(m_channels.size());
        add_user_callbacks();
    }

    ~MultiChannelSynchronizer() = default;

    ///
    /// @brief Access a channel by index
    ///
    Channel& channel(size_t index)
    {
        if (index >= m_channels.size())
        {
            throw std::runtime_error("Invalid multi-channel access");
        }

        return *m_channels.at(index);
    }

    ///
    /// @brief Get a collection synchronized frames from the input channels with no timeout on waiting
    ///
    ///  @return Return a collection of synchronized frames
    ///
    std::optional<std::vector<ImageFrame>> get_synchronized_frame()
    {
        return get_synchronized_frame(std::nullopt);
    }

    ///
    /// @brief Get a collection synchronized frames from the input channels and return if we have not recieved
    ///        a collection of frames before the input timeout
    ///
    ///  @param timeout The ammount of time to wait for a synchronized frame
    ///  @return Return a collection of synchronized frames
    ///
    std::optional<std::vector<ImageFrame>> get_synchronized_frame(const std::optional<std::chrono::nanoseconds> &timeout);

private:

    ///
    /// @brief Helper to add user callbacks to the input channels
    ///
    void add_user_callbacks();

    ///
    /// @brief The collection of channels raw channels to synchronize
    ///
    std::vector<Channel*> m_channels{};

    ///
    /// @brief A collection of owned channels if the user would like the synchronizer to own the channel memory
    ///
    std::vector<std::unique_ptr<Channel>> m_owned_channels{};

    ///
    /// @brief A collection of active frames which may be dispatched to the user
    ///
    std::vector<ImageFrame> m_active_frames{};

    ///
    /// @brief The max time tolerance between image frames for them to be considered equal
    ///
    std::chrono::nanoseconds m_tolerance{};

    ///
    /// @brief Maximum number of synchronized frame groups that will be queued (0 = unlimited)
    ///
    size_t m_max_queue_size = 0;

    ///
    /// @brief Mutex to notify the user a collection of synchronized frames is ready
    ///
    std::mutex m_frame_mutex;

    ///
    /// @brief Condition variable to notify the user a collection of synchronized frames is ready
    ///
    std::condition_variable m_frame_cv;

    ///
    /// @brief Queue of synchronized frames ready for the user to consume
    ///
    std::deque<std::vector<ImageFrame>> m_ready_frames{};
};

}
