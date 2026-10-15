/**
 * @file color_conversion.cc
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
 **/

#include <algorithm>
#include <cstddef>

#if defined(MULTISENSE_HAVE_AVX2) && defined(_MSC_VER)
#include <intrin.h>
#endif

#include "details/color_conversion.hh"

namespace multisense {
namespace details {

#ifdef MULTISENSE_HAVE_AVX2
void convert_ycbcr420_avx2(const uint8_t *luma, const uint8_t *chroma,
                          uint8_t *output, int width, int height);
#endif

namespace {

void convert_ycbcr420_scalar(const uint8_t *luma, const uint8_t *chroma,
                            uint8_t *output, int width, int height)
{
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t luma_offset = static_cast<size_t>(h) * width + w;
            const size_t chroma_offset = 2 * (static_cast<size_t>(h / 2) * (width / 2) + (w / 2));
            const float y = static_cast<float>(luma[luma_offset]);
            const float cb = static_cast<float>(chroma[chroma_offset]) - 128.0f;
            const float cr = static_cast<float>(chroma[chroma_offset + 1]) - 128.0f;

            output[3 * luma_offset] = static_cast<uint8_t>(std::clamp(y + 2.03211f * cb, 0.0f, 255.0f));
            output[3 * luma_offset + 1] = static_cast<uint8_t>(std::clamp(y - 0.39465f * cb - 0.58060f * cr, 0.0f, 255.0f));
            output[3 * luma_offset + 2] = static_cast<uint8_t>(std::clamp(y + 1.13983f * cr, 0.0f, 255.0f));
        }
    }
}

}

bool cpu_supports_avx2()
{
#if !defined(MULTISENSE_HAVE_AVX2)
    return false;
#elif defined(_MSC_VER)
    int registers[4];
    __cpuid(registers, 0);
    if (registers[0] < 7)
    {
        return false;
    }

    __cpuidex(registers, 1, 0);
    // XSAVE, OSXSAVE, and AVX must be present before executing XGETBV.
    constexpr int avx_state_mask = (1 << 26) | (1 << 27) | (1 << 28);
    if ((registers[2] & avx_state_mask) != avx_state_mask || (_xgetbv(0) & 0x6) != 0x6)
    {
        return false;
    }

    __cpuidex(registers, 7, 0);
    return (registers[1] & (1 << 5)) != 0;
#else
    // GCC and Clang include OS AVX-state support in this runtime check.
    // Explicit initialization also permits use from application constructors.
    __builtin_cpu_init();
    return __builtin_cpu_supports("avx2");
#endif
}

YcbcrConverter select_ycbcr_converter(bool avx2_supported)
{
#ifdef MULTISENSE_HAVE_AVX2
    if (avx2_supported)
    {
        return convert_ycbcr420_avx2;
    }
#else
    (void)avx2_supported;
#endif
    return convert_ycbcr420_scalar;
}

void convert_ycbcr420_to_bgr(const uint8_t *luma, const uint8_t *chroma,
                            uint8_t *output, int width, int height)
{
    // Initialization is thread safe and feature detection runs only once.
    static const auto convert = select_ycbcr_converter(cpu_supports_avx2());
    convert(luma, chroma, output, width, height);
}

}
}
