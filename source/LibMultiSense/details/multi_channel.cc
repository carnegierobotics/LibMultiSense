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

#include "MultiSense/MultiSenseMultiChannel.hh"

namespace multisense {

void MultiChannelSynchronizer::add_user_callbacks()
{
    for(size_t i = 0 ; i < m_channels.size() ; ++i)
    {
        m_channels[i]->add_image_frame_callback([i, this](auto frame)
                                                {
                                                    std::lock_guard<std::mutex> lock(m_frame_mutex);
                                                    m_active_frames[i] = frame;

                                                    if (frames_valid(m_tolerance))
                                                    {
                                                        m_frame_cv.notify_all();
                                                    }
                                                });
    }
}

bool MultiChannelSynchronizer::frames_valid(const std::chrono::nanoseconds &tolerance)
{
    using namespace std::chrono_literals;

    const auto reference_time = m_active_frames.front().frame_time;

    return std::all_of(std::begin(m_active_frames), std::end(m_active_frames),
                [reference_time, tolerance](const auto &frame)
                {
                    return reference_time.time_since_epoch() != 0ns && std::chrono::abs(reference_time - frame.frame_time) < tolerance;
                });
}

std::optional<std::vector<ImageFrame>> MultiChannelSynchronizer::get_synchronized_frame(const std::optional<std::chrono::nanoseconds> &timeout)
{
    std::unique_lock<std::mutex> lock(m_frame_mutex);
    if (timeout)
    {
        if (std::cv_status::timeout == m_frame_cv.wait_for(lock, timeout.value()))
        {
            return std::nullopt;
        }
    }
    else
    {
        m_frame_cv.wait(lock);
    }

    auto output_frames = m_active_frames;
    m_active_frames.clear();
    m_active_frames.resize(m_channels.size());
    return output_frames;
}

}
