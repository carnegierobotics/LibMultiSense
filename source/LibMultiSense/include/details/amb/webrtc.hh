/**
 * @file webrtc.hh
 *
 * Copyright 2026
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

#include <functional>
#include <string>

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense {
namespace amb {

///
/// @brief A WebRTC client to connect to the camera
///
class WebRtcClient {
public:
    ///
    /// @brief Create a new WebRTC client which attaches to a server at a specific hostname
    ///
    explicit WebRtcClient(const std::string& host);
    ~WebRtcClient();

    WebRtcClient(const WebRtcClient&) = delete;
    WebRtcClient& operator=(const WebRtcClient&) = delete;

    ///
    /// @brief Connect to the WebRTC server and start streaming available topics
    ///
    bool connect(bool left, bool right, bool disparity, bool nndata);

    ///
    /// @brief Disconnect from the WebRTC sever
    ///
    void disconnect();

    ///
    /// @brief Set the bitrate used to encode left/right rectified image
    ///
    void set_bitrate(double bitrate);

    ///
    /// @brief Attach a frame callback to the client which returns whenever a valid ImageFrame is received
    ///
    void set_frame_callback(std::function<void(ImageFrame&)> callback);

private:

    ///
    /// @brief Private callback signature which the c client WebRTC uses to return images
    ///
    static void c_frame_callback(void* left_data,
                                 int left_size,
                                 void* right_data,
                                 int right_size,
                                 void* disparity_data,
                                 int disparity_size,
                                 void* user_data);

    ///
    /// @param Pointer to the base C WebRTC implementation
    ///
    void* m_impl = nullptr;

    std::function<void(ImageFrame&)> m_frame_callback;
};

} // namespace amb
} // namespace multisense
