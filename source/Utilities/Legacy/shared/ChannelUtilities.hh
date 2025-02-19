/**
 * @file shared/ChannelUtilities.hh
 *
 * Copyright 2013-2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * https://www.carnegierobotics.com
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
 **/

#ifndef CHANNEL_UTILITIES_HH
#define CHANNEL_UTILITIES_HH

#include <array>
#include <memory>

#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

//
// Wrapper around a Channel pointer to make cleanup easier

class ChannelWrapper
{
    public:

        ChannelWrapper(const std::string& ipAddress) :
            channelPtr_(Channel::Create(ipAddress))
    {
    }

        ~ChannelWrapper()
        {
            if (channelPtr_) {
                Channel::Destroy(channelPtr_);
            }
        }

        Channel* ptr() noexcept
        {
            return channelPtr_;
        }

    private:

        ChannelWrapper(const ChannelWrapper&) = delete;
        ChannelWrapper operator=(const ChannelWrapper&) = delete;

        Channel* channelPtr_ = nullptr;
};

//
// Wrapper to preserve image data outside of the image callback

class ImageBufferWrapper
{
    public:
        ImageBufferWrapper(crl::multisense::Channel* driver,
                const crl::multisense::image::Header& data) :
            driver_(driver),
            callbackBuffer_(driver->reserveCallbackBuffer()),
            data_(data)
    {
    }

        ~ImageBufferWrapper()
        {
            if (driver_) {
                driver_->releaseCallbackBuffer(callbackBuffer_);
            }
        }

        const image::Header& data() const noexcept
        {
            return data_;
        }

    private:

        ImageBufferWrapper(const ImageBufferWrapper&) = delete;
        ImageBufferWrapper operator=(const ImageBufferWrapper&) = delete;

        crl::multisense::Channel* driver_ = nullptr;
        void* callbackBuffer_;
        const image::Header data_;

};

template<typename T>
constexpr std::array<uint8_t, 3> ycbcrToBgr(const crl::multisense::image::Header& luma,
                                            const crl::multisense::image::Header& chroma,
                                            const size_t u,
                                            const size_t v)
{
    const uint8_t* lumaP = reinterpret_cast<const uint8_t*>(luma.imageDataP);
    const uint8_t* chromaP = reinterpret_cast<const uint8_t*>(chroma.imageDataP);

    const size_t luma_offset = (v * luma.width) + u;
    const size_t chroma_offset = 2 * (((v / 2) * (luma.width / 2)) + (u / 2));

    const float px_y = static_cast<float>(lumaP[luma_offset]);
    const float px_cb = static_cast<float>(chromaP[chroma_offset + 0]) - 128.0f;
    const float px_cr = static_cast<float>(chromaP[chroma_offset + 1]) - 128.0f;

    float px_r = px_y + 1.13983f * px_cr;
    float px_g = px_y - 0.39465f * px_cb - 0.58060f * px_cr;
    float px_b = px_y + 2.03211f * px_cb;

    if (px_r < 0.0f)        px_r = 0.0f;
    else if (px_r > 255.0f) px_r = 255.0f;
    if (px_g < 0.0f)        px_g = 0.0f;
    else if (px_g > 255.0f) px_g = 255.0f;
    if (px_b < 0.0f)        px_b = 0.0f;
    else if (px_b > 255.0f) px_b = 255.0f;

    return { {static_cast<uint8_t>(px_r), static_cast<uint8_t>(px_g), static_cast<uint8_t>(px_b)} };
}

void ycbcrToBgr(const crl::multisense::image::Header& luma,
                const crl::multisense::image::Header& chroma,
                uint8_t* output)
{
    if (luma.bitsPerPixel != 8 || chroma.bitsPerPixel != 16)
    {
        throw std::runtime_error("Only 8-bit luma and 16-bit chroma images are supported by the \
                                  ycbcrToBgr conversion function");
    }

    const size_t rgb_stride = luma.width * 3;

    for (uint32_t y = 0; y < luma.height; ++y)
    {
        const size_t row_offset = y * rgb_stride;

        for (uint32_t x = 0; x < luma.width; ++x)
        {
            memcpy(output + row_offset + (3 * x), ycbcrToBgr<uint8_t>(luma, chroma, x, y).data(), 3);
        }
    }
}

#endif //CHANNEL_UTILITIES_HH

