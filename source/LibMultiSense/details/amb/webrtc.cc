/**
 * @file webrtc_client.cc
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

#include "details/amb/webrtc.hh"

#include <cstring>
#include <iostream>

// Declare the extern "C" functions exposed by the Rust FFI library
extern "C"
{
    typedef void (*mswebrtc_frame_callback)(void* left_data, int left_size,
                                            void* right_data, int right_size,
                                            void* disparity_data, int disparity_size,
                                            void* user_data);

    void* create_mswebrtc_impl(const char* host);
    bool connect_mswebrtc_impl(void* obj, bool left, bool right, bool disparity, bool nndata);
    void disconnect_mswebrtc_impl(void* obj);
    void destroy_mswebrtc_impl(void* obj);
    void set_frame_callback_mswebrtc_impl(void* obj, mswebrtc_frame_callback cb, void* user_data);
    void set_bitrate_mswebrtc_impl(void* obj, double bitrate);
}

namespace multisense {
namespace amb {

WebRtcClient::WebRtcClient(const std::string& host)
    : m_impl(nullptr)
{
    m_impl = create_mswebrtc_impl(host.c_str());
}

WebRtcClient::~WebRtcClient()
{
    if (m_impl) {
        destroy_mswebrtc_impl(m_impl);
        m_impl = nullptr;
    }
}

bool WebRtcClient::connect(bool left, bool right, bool disparity, bool nndata)
{
    if (!m_impl)
    {
        return false;
    }

    return connect_mswebrtc_impl(m_impl, left, right, disparity, nndata);
}

void WebRtcClient::disconnect()
{
    if (m_impl)
    {
        disconnect_mswebrtc_impl(m_impl);
    }
}

void WebRtcClient::set_bitrate(double bitrate)
{
    if (m_impl)
    {
        set_bitrate_mswebrtc_impl(m_impl, bitrate);
    }
}

void WebRtcClient::set_frame_callback(std::function<void(ImageFrame&)> callback)
{
    m_frame_callback = std::move(callback);

    if (m_impl)
    {
        set_frame_callback_mswebrtc_impl(m_impl, &WebRtcClient::c_frame_callback, this);
    }
}

void WebRtcClient::c_frame_callback(void* left_data,
                                    int left_size,
                                    void* right_data,
                                    int right_size,
                                    void* disparity_data,
                                    int disparity_size,
                                    void* user_data)
{
    if (!user_data)
    {
        return;
    }

    WebRtcClient* client = static_cast<WebRtcClient*>(user_data);
    if (!client->m_frame_callback)
    {
        return;
    }

    std::cout << left_size << " " << right_size << " " << disparity_size << std::endl;

    // TODO (malvarado): do this better
    static size_t frame_id = 0;
    ImageFrame frame;
    frame.frame_id = ++frame_id;
    const auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    frame.frame_time = now;
    frame.ptp_frame_time = now;

    auto create_image = [](void* data, int size, size_t width, size_t height, DataSource source, Image::PixelFormat format) -> std::optional<Image>
    {
        if (data == nullptr || size <= 0)
        {
            return std::nullopt;
        }

        Image image;
        auto buffer = std::make_shared<std::vector<uint8_t>>(size);
        std::memcpy(buffer->data(), data, size);

        image.raw_data = buffer;
        image.image_data_offset = 0;
        image.image_data_length = size;
        image.source = source;
        image.format = format;
        // TODO (malvarado) : fix this
        image.width = width;
        image.height = height;

        return image;
    };

    if (auto left_image = create_image(left_data,
                                       left_size,
                                       1920,
                                       1200,
                                       DataSource::LEFT_RECTIFIED_RAW,
                                       Image::PixelFormat::MONO8); left_image)
    {
        frame.add_image(left_image.value());
    }

    if (auto right_image = create_image(right_data,
                                        right_size,
                                        1920,
                                        1200,
                                        DataSource::RIGHT_RECTIFIED_RAW,
                                        Image::PixelFormat::MONO8); right_image)
    {
        frame.add_image(right_image.value());
    }

    if (auto disparity_image = create_image(disparity_data,
                                            disparity_size,
                                            960,
                                            600,
                                            DataSource::LEFT_DISPARITY_RAW, Image::PixelFormat::MONO16); disparity_image)
    {
        frame.add_image(disparity_image.value());
    }

    client->m_frame_callback(frame);
}

} // namespace amb
} // namespace multisense
