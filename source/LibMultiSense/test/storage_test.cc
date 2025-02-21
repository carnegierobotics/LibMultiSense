/**
 * @file storage_test.cc
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
 *   2025-01-08, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/storage.hh>

using namespace multisense::legacy;

TEST(BufferPool, null_construction)
{
    BufferPool pool(BufferPoolConfig{0, 0, 0, 0});

    ASSERT_EQ(pool.get_buffer(1), nullptr);
}

TEST(BufferPool, valid_construction)
{
    BufferPool pool(BufferPoolConfig{1, 10, 1, 30});

    const auto small_buffer = pool.get_buffer(2);
    const auto large_buffer = pool.get_buffer(20);

    ASSERT_NE(small_buffer, nullptr);
    ASSERT_NE(large_buffer, nullptr);

    //
    // At this point we should be out of buffers
    //
    ASSERT_EQ(pool.get_buffer(20), nullptr);
    ASSERT_EQ(pool.get_buffer(2), nullptr);
}

TEST(BufferPool, buffer_to_large)
{
    BufferPool pool(BufferPoolConfig{1, 10, 1, 30});

    //
    // We should have no buffer of size 40
    //
    ASSERT_EQ(pool.get_buffer(40), nullptr);
}
