/**
 * @file multi_channel_test.cc
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

#include <gtest/gtest.h>

#include <MultiSense/MultiSenseMultiChannel.hh>

using namespace multisense;

TEST(frames_synchronized, basic_test)
{
    using namespace std::chrono_literals;
    std::vector<ImageFrame> frames{};

    EXPECT_TRUE(frames_synchronized(frames, 0ns));

    frames.emplace_back(ImageFrame{1, {}, {}, TimeT{10ms}, TimeT{10ms}, {}, {}, {}, {}});
    frames.emplace_back(ImageFrame{3, {}, {}, TimeT{30ms}, TimeT{30ms}, {}, {}, {}, {}});

    EXPECT_FALSE(frames_synchronized(frames, 0ns));
    EXPECT_TRUE(frames_synchronized(frames, 21ms));

    frames.emplace_back(ImageFrame{37, {}, {}, TimeT{38ms}, TimeT{38ms}, {}, {}, {}, {}});
    frames.emplace_back(ImageFrame{7, {}, {}, TimeT{31ms}, TimeT{31ms}, {}, {}, {}, {}});

    EXPECT_FALSE(frames_synchronized(frames, 21ms));
    EXPECT_TRUE(frames_synchronized(frames, 29ms));
}
