/**
 * @file color_conversion_test.cc
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
#include <array>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "details/color_conversion.hh"

using namespace multisense::details;

TEST(color_conversion, runtime_selection_and_forced_scalar)
{
    const auto scalar = select_ycbcr_converter(false);
#ifdef MULTISENSE_HAVE_AVX2
    EXPECT_NE(scalar, select_ycbcr_converter(true));
#else
    EXPECT_FALSE(cpu_supports_avx2());
    EXPECT_EQ(scalar, select_ycbcr_converter(true));
#endif

    const std::array<uint8_t, 8> y{0, 16, 128, 255, 0, 16, 128, 255};
    const std::array<uint8_t, 8> uv{128, 128, 0, 255, 255, 0, 129, 129};
    const std::array<uint8_t, 24> expected{
        0, 0, 0, 16, 16, 16, 0, 104, 255, 0, 231, 255,
        255, 24, 0, 255, 40, 0, 130, 127, 129, 255, 254, 255};
    std::array<uint8_t, 24> scalar_output{};
    scalar(y.data(), uv.data(), scalar_output.data(), 8, 1);
    EXPECT_EQ(scalar_output, expected);

    std::array<uint8_t, 24> dispatched{};
    convert_ycbcr420_to_bgr(y.data(), uv.data(), dispatched.data(), 8, 1);
    EXPECT_EQ(dispatched, expected);

    std::array<uint8_t, 24> selected{};
    select_ycbcr_converter(cpu_supports_avx2())(y.data(), uv.data(), selected.data(), 8, 1);
    EXPECT_EQ(dispatched, selected);
}

TEST(color_conversion, all_y_cb_cr_combinations)
{
    if (!cpu_supports_avx2())
    {
        GTEST_SKIP() << "AVX2 is not available in this build or on this CPU/OS";
    }
    const auto scalar = select_ycbcr_converter(false);
    const auto avx2 = select_ycbcr_converter(true);
    constexpr int width = 512;
    constexpr int height = 128;
    std::vector<uint8_t> y(width * height);
    std::vector<uint8_t> uv(width * height / 2);
    std::vector<uint8_t> expected(width * height * 3);
    std::vector<uint8_t> actual(expected.size());

    // Every 2x2 block contains four consecutive Y values and one Cb/Cr pair.
    // Each image covers every Y/Cb pair for a fixed Cr, without a large allocation.
    for (int cr = 0; cr < 256; ++cr)
    {
        for (int h = 0; h < height; h += 2)
        {
            for (int w = 0; w < width; w += 2)
            {
                const int c = (h / 2) * width + w;
                uv[c] = static_cast<uint8_t>(w / 2);
                uv[c + 1] = static_cast<uint8_t>(cr);
                y[h * width + w] = static_cast<uint8_t>(2 * h);
                y[h * width + w + 1] = static_cast<uint8_t>(2 * h + 1);
                y[(h + 1) * width + w] = static_cast<uint8_t>(2 * h + 2);
                y[(h + 1) * width + w + 1] = static_cast<uint8_t>(2 * h + 3);
            }
        }
        scalar(y.data(), uv.data(), expected.data(), width, height);
        avx2(y.data(), uv.data(), actual.data(), width, height);
        ASSERT_EQ(actual, expected) << "Cr=" << cr;
    }
}

TEST(color_conversion, tails_alignment_and_output_bounds)
{
    const auto scalar = select_ycbcr_converter(false);
    std::mt19937 rng(43);
    for (int width = 1; width <= 65; ++width)
    {
        for (int height : {1, 2, 3, 4, 5, 17})
        {
            for (int offset = 0; offset < 32; ++offset)
            {
                const int uv_offset = (offset * 3 + 5) % 32;
                const int output_offset = 32 + (offset * 7 + 11) % 32;
                const size_t pixels = width * height;
                // Match the existing floor(width/2) indexing, even for odd dimensions.
                const size_t chroma_bytes = 2 * (((height - 1) / 2) * (width / 2) + (width - 1) / 2) + 2;
                std::vector<uint8_t> y(pixels + offset);
                std::vector<uint8_t> uv(chroma_bytes + uv_offset);
                for (auto &v : y) v = static_cast<uint8_t>(rng());
                for (auto &v : uv) v = static_cast<uint8_t>(rng());
                std::vector<uint8_t> expected(3 * pixels);
                std::vector<uint8_t> output(3 * pixels + output_offset + 32, 0xcd);
                scalar(y.data() + offset, uv.data() + uv_offset, expected.data(), width, height);
                convert_ycbcr420_to_bgr(y.data() + offset, uv.data() + uv_offset,
                                        output.data() + output_offset, width, height);
                ASSERT_TRUE(std::equal(expected.begin(), expected.end(), output.begin() + output_offset))
                    << width << "x" << height << ", offset=" << offset;
                ASSERT_TRUE(std::all_of(output.begin(), output.begin() + output_offset,
                                        [](uint8_t v) { return v == 0xcd; }));
                ASSERT_TRUE(std::all_of(output.end() - 32, output.end(),
                                        [](uint8_t v) { return v == 0xcd; }));
            }
        }
    }
}

TEST(color_conversion, empty_dimensions)
{
    for (const auto &dimensions : {std::pair{0, 0}, std::pair{0, 2}, std::pair{2, 0}})
    {
        select_ycbcr_converter(false)(nullptr, nullptr, nullptr, dimensions.first, dimensions.second);
        convert_ycbcr420_to_bgr(nullptr, nullptr, nullptr, dimensions.first, dimensions.second);
    }
}
