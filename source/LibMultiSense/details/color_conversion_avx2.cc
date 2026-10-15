/**
 * @file color_conversion_avx2.cc
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

#include <cstddef>
#include <cstdint>
#include <immintrin.h>

namespace multisense {
namespace details {
namespace {

inline uint8_t clamp_byte(float value)
{
    const int integer = static_cast<int>(value);
    return static_cast<uint8_t>(integer < 0 ? 0 : (integer > 255 ? 255 : integer));
}

// Narrow eight channels, preserving truncation and saturating to [0, 255].
inline __m128i to_words(__m256 value)
{
    const __m256i ints = _mm256_cvttps_epi32(value);
    return _mm_packs_epi32(_mm256_castsi256_si128(ints), _mm256_extracti128_si256(ints, 1));
}

inline void store_bgr8(uint8_t *dst, __m256 b, __m256 g, __m256 r)
{
    const __m128i bg = _mm_packus_epi16(to_words(b), to_words(g));
    const __m128i rr = to_words(r);
    const __m128i rb = _mm_packus_epi16(rr, rr);
    // bg holds B0..B7,G0..G7; rb holds R0..R7 twice. Write exactly 24 bytes.
    const __m128i low = _mm_or_si128(
        _mm_shuffle_epi8(bg, _mm_setr_epi8(0, 8, -1, 1, 9, -1, 2, 10, -1, 3, 11, -1, 4, 12, -1, 5)),
        _mm_shuffle_epi8(rb, _mm_setr_epi8(-1, -1, 0, -1, -1, 1, -1, -1, 2, -1, -1, 3, -1, -1, 4, -1)));
    const __m128i high = _mm_or_si128(
        _mm_shuffle_epi8(bg, _mm_setr_epi8(13, -1, 6, 14, -1, 7, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1)),
        _mm_shuffle_epi8(rb, _mm_setr_epi8(-1, 5, -1, -1, 6, -1, -1, 7, -1, -1, -1, -1, -1, -1, -1, -1)));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), low);
    _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + 16), high);
}

inline __m256 load_y8(const uint8_t *src)
{
    return _mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(src))));
}

inline __m256 unpack_chroma8(__m128i cbcr, __m128i mask)
{
    const __m256i values = _mm256_cvtepu8_epi32(_mm_shuffle_epi8(cbcr, mask));
    return _mm256_sub_ps(_mm256_cvtepi32_ps(values), _mm256_set1_ps(128.0f));
}

inline void convert8(const uint8_t *src, uint8_t *dst, __m256 b, __m256 g_cb, __m256 g_cr, __m256 r)
{
    const __m256 y = load_y8(src);
    store_bgr8(dst, _mm256_add_ps(y, b), _mm256_sub_ps(_mm256_sub_ps(y, g_cb), g_cr), _mm256_add_ps(y, r));
}
}

// Keep this ISA-specific kernel out of baseline callers, including LTO builds.
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
void convert_ycbcr420_avx2(const uint8_t *y_data, const uint8_t *uv_data,
                          uint8_t *output, int image_width, int height)
{
    if (image_width <= 0 || height <= 0)
    {
        return;
    }
    const size_t width = image_width;
    const size_t rows = height;
    // Retain the scalar implementation's floor(width / 2) chroma stride.
    const size_t uv_stride = 2 * (width / 2);
    const auto cb_mask = _mm_setr_epi8(0, 0, 2, 2, 4, 4, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1);
    const auto cr_mask = _mm_setr_epi8(1, 1, 3, 3, 5, 5, 7, 7, -1, -1, -1, -1, -1, -1, -1, -1);
    for (size_t h = 0; h < rows; h += 2)
    {
        const auto *y0 = y_data + h * width;
        const auto *y1 = h + 1 < rows ? y0 + width : y0;
        const auto *uv = uv_data + (h / 2) * uv_stride;
        auto *dst0 = output + h * width * 3;
        auto *dst1 = h + 1 < rows ? dst0 + width * 3 : dst0;
        size_t w = 0;
        for (; w + 8 <= width; w += 8)
        {
            const __m128i cbcr = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(uv + w));
            const __m256 cb = unpack_chroma8(cbcr, cb_mask);
            const __m256 cr = unpack_chroma8(cbcr, cr_mask);
            const __m256 b = _mm256_mul_ps(cb, _mm256_set1_ps(2.03211f));
            const __m256 g_cb = _mm256_mul_ps(cb, _mm256_set1_ps(0.39465f));
            const __m256 g_cr = _mm256_mul_ps(cr, _mm256_set1_ps(0.58060f));
            const __m256 r = _mm256_mul_ps(cr, _mm256_set1_ps(1.13983f));
            convert8(y0 + w, dst0 + 3 * w, b, g_cb, g_cr, r);
            convert8(y1 + w, dst1 + 3 * w, b, g_cb, g_cr, r);
        }
        for (; w < width; ++w)
        {
            const float cb = static_cast<float>(uv[2 * (w / 2)]) - 128.0f;
            const float cr = static_cast<float>(uv[2 * (w / 2) + 1]) - 128.0f;
            const auto scalar_pixel = [=](uint8_t value, uint8_t *dst)
            {
                const float y = value;
                dst[0] = clamp_byte(y + 2.03211f * cb);
                dst[1] = clamp_byte(y - 0.39465f * cb - 0.58060f * cr);
                dst[2] = clamp_byte(y + 1.13983f * cr);
            };
            scalar_pixel(y0[w], dst0 + 3 * w);
            scalar_pixel(y1[w], dst1 + 3 * w);
        }
    }
}

}
}
