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

#include <mutex>
#include <condition_variable>

#include "MultiSenseChannel.hh"

namespace multisense {

class MULTISENSE_API MultiChannelSynchronizer {
public:

   explicit MultiChannelSynchronizer(std::vector<std::unique_ptr<Channel>> channels, const std::chrono::nanoseconds &tolerance):
        m_channels(std::move(channels)),
        m_active_frames(channels.size()),
        m_tolerance(tolerance)
    {
        add_user_callbacks();
    }

    template <typename... ChannelT>
    explicit MultiChannelSynchronizer(ChannelT&&... channels, const std::chrono::nanoseconds &tolerance):
        m_tolerance(tolerance)
    {
        m_channels.reserve(sizeof...(channels));
        m_active_frames.resize(sizeof...(channels));
        (m_channels.emplace_back(std::forward<ChannelT>(channels)), ...);

        add_user_callbacks();
    }

    ~MultiChannelSynchronizer() = default;

    Channel& channel(size_t index)
    {
        if (index > m_channels.size())
        {
            throw std::runtime_error("Invalid multi-channel access");
        }

        return *m_channels.at(index);
    }

    std::optional<std::vector<ImageFrame>> get_synchronized_frame()
    {
        return get_synchronized_frame(std::nullopt);
    }

    std::optional<std::vector<ImageFrame>> get_synchronized_frame(const std::optional<std::chrono::nanoseconds> &timeout);

private:

    void add_user_callbacks();

    bool frames_valid(const std::chrono::nanoseconds &tolerance);

    ///
    /// @brief The collection of channels to synchronize
    ///
    std::vector<std::unique_ptr<Channel>> m_channels{};

    ///
    /// @brief A collection of active frames which may be dispatched to the user
    ///
    std::vector<ImageFrame> m_active_frames{};

    std::chrono::nanoseconds m_tolerance{};


    std::mutex m_frame_mutex;
    std::condition_variable m_frame_cv;
};

}
