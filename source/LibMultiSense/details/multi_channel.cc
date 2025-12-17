/**
 * @file multi_channel.cc
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

#include <algorithm>

#include "MultiSense/MultiSenseMultiChannel.hh"

namespace multisense {

bool frames_synchronized(const std::vector<ImageFrame> &frames, const std::chrono::nanoseconds &tolerance)
{
    using namespace std::chrono_literals;

    if (frames.size() <= 1)
    {
        return true;
    }

    const auto [min, max] = std::minmax_element(std::begin(frames), std::end(frames),
                                                [](const auto &lhs, const auto &rhs)
                                                {
                                                    return lhs.ptp_frame_time < rhs.ptp_frame_time;
                                                });

    return min->ptp_frame_time.time_since_epoch() != 0ns && std::chrono::abs(max->ptp_frame_time - min->ptp_frame_time) < tolerance;

}


void MultiChannelSynchronizer::add_user_callbacks()
{
    for(size_t i = 0 ; i < m_channels.size() ; ++i)
    {
        m_channels[i]->add_image_frame_callback([i, this](auto frame)
                                                {
                                                    std::lock_guard<std::mutex> lock(m_frame_mutex);
                                                    m_active_frames[i] = std::move(frame);

                                                    if (!m_ready_frames.empty())
                                                    {
                                                        m_frame_cv.notify_all();
                                                    }

                                                    if (frames_synchronized(m_active_frames, m_tolerance))
                                                    {
                                                        if (m_max_queue_size > 0 &&
                                                            m_ready_frames.size() >= m_max_queue_size)
                                                        {
                                                            m_ready_frames.pop_front();
                                                        }

                                                        m_ready_frames.emplace_back(m_active_frames);
                                                        for (auto &active_frame : m_active_frames)
                                                        {
                                                            active_frame = ImageFrame{};
                                                        }

                                                        m_frame_cv.notify_all();
                                                    }
                                                });
    }
}

std::optional<std::vector<ImageFrame>> MultiChannelSynchronizer::get_synchronized_frame(const std::optional<std::chrono::nanoseconds> &timeout)
{
    std::unique_lock<std::mutex> lock(m_frame_mutex);
    const auto frames_ready = [this]() { return !m_ready_frames.empty(); };

    if (timeout)
    {
        if (!m_frame_cv.wait_for(lock, timeout.value(), frames_ready))
        {
            return std::nullopt;
        }
    }
    else
    {
        m_frame_cv.wait(lock, frames_ready);
    }

    if (m_ready_frames.empty())
    {
        return std::nullopt;
    }

    auto output_frames = std::move(m_ready_frames.front());
    m_ready_frames.pop_front();
    return output_frames;
}

}
