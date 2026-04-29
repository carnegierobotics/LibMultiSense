/**
 * @file utilities.hh
 *
 * Copyright 2013-2026
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
 *   2026-04-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <mutex>
#include <condition_variable>
#include <optional>

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense{

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

///
/// @brief Scale a calibration used to update a full-res calibration based on the current operating resolution
///
CameraCalibration scale_calibration(const CameraCalibration &input, double x_scale, double y_scale);

///
/// @brief Scale a calibration used to update a full-res calibration based on the current operating resolution
///
StereoCalibration scale_calibration(const StereoCalibration &input, double x_scale, double y_scale);

}
