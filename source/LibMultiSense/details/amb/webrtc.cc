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

#include "details/utilities.hh"
#include "details/amb/webrtc.hh"

#include <cstring>
#include <iostream>

extern "C"
{
#include "multisense-webrtc-client.h"
}

namespace multisense {
namespace amb {

WebRtcClient::WebRtcClient(const std::string& host, const StereoCalibration &calibration)
    : m_impl(nullptr),
      m_calibration(calibration)
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

void WebRtcClient::c_frame_callback(uint64_t timestamp,
                                    uint32_t frame_id,
                                    const struct ImageData* left,
                                    const struct ImageData* right,
                                    const struct ImageData* disparity,
                                    const struct ImageData* nndata,
                                    void* user_data)
{
    (void) nndata;
    if (!user_data)
    {
        return;
    }

    WebRtcClient* client = static_cast<WebRtcClient*>(user_data);
    if (!client->m_frame_callback)
    {
        return;
    }

    ImageFrame frame;

    frame.frame_id = frame_id;
    const std::chrono::nanoseconds timestamp_ns(timestamp);
    const TimeT image_time{timestamp_ns};
    frame.frame_time = image_time;
    frame.ptp_frame_time = image_time;

    auto create_image = [](const struct ImageData* data,
                           const CameraCalibration &calibration,
                           DataSource source,
                           const TimeT &image_timestamp) -> std::optional<Image>
    {
        if (data == nullptr || data->size <= 0 || data->data == nullptr)
        {
            return std::nullopt;
        }

        //
        // Increment our reference count, and convert the raw data ptr into a shared_ptr which will
        // decrement the reference count once destructed
        //
        incref_imagedata(const_cast<struct ImageData*>(data));
        auto data_p = static_cast<const uint8_t*>(data->data);
        std::shared_ptr<BufferWrapper> buffer(new BufferWrapper(data_p, static_cast<size_t>(data->size)),
                                              [data](const BufferWrapper*)
                                              {
                                                  decref_imagedata(const_cast<struct ImageData*>(data));
                                              });


        Image::PixelFormat format = Image::PixelFormat::UNKNOWN;
        if (data->channels == 1 && data->bytewidth == 1)
        {
            format = Image::PixelFormat::MONO8;
        }
        else if (data->channels == 1 && data->bytewidth == 2)
        {
            format = Image::PixelFormat::MONO16;
        }
        else if (data->channels == 3 && data->bytewidth == 1)
        {
            format = Image::PixelFormat::BGR8;
        }
        else
        {
            format = Image::PixelFormat::UNKNOWN;
        }

        return Image{std::move(buffer),
                     format,
                     data->width,
                     data->height,
                     image_timestamp,
                     image_timestamp,
                     source,
                     calibration,
                     1.0/static_cast<double>(1 << data->precision)};
    };

    if (auto left_image = create_image(left,
                                       client->m_calibration.left,
                                       DataSource::LEFT_RECTIFIED_RAW,
                                       image_time); left_image)
    {
        frame.add_image(left_image.value());
    }

    if (auto right_image = create_image(right,
                                        client->m_calibration.right,
                                        DataSource::RIGHT_RECTIFIED_RAW,
                                        image_time); right_image)
    {
        frame.add_image(right_image.value());
    }

    // TODO (malvarado): Scale this calibration
    if (auto disparity_image = create_image(disparity,
                                            scale_calibration(client->m_calibration.left, 0.5, 0.5),
                                            DataSource::LEFT_DISPARITY_RAW,
                                            image_time); disparity_image)
    {
        frame.add_image(disparity_image.value());
    }

    client->m_frame_callback(frame);
}

} // namespace amb
} // namespace multisense
