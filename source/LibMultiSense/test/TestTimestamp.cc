/**
 * @file LibMultiSense/test/TestTimestamp.hh
 *
 * Copyright 2023
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
 *   2023-12-12, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_TIMESTAMP_TEST_HH
#define CRL_MULTISENSE_TIMESTAMP_TEST_HH

#include "MultiSense/details/utility/TimeStamp.hh"

#include <gtest/gtest.h>

using namespace crl::multisense::details::utility;

TEST(TimeStamp, nanosecond_construction)
{
    const TimeStamp a(1000);

    EXPECT_EQ(a.getNanoSeconds(), 1000);
    EXPECT_EQ(a.getSeconds(), 0);
    EXPECT_EQ(a.getMicroSeconds(), 1);

    const TimeStamp b(-1000);

    EXPECT_EQ(b.getNanoSeconds(), -1000);
    EXPECT_EQ(b.getSeconds(), 0);
    EXPECT_EQ(b.getMicroSeconds(), -1);

    const TimeStamp c(-1000001000);

    EXPECT_EQ(c.getNanoSeconds(), -1000001000);
    EXPECT_EQ(c.getSeconds(), -1);
    EXPECT_EQ(c.getMicroSeconds(), 1);
}

TEST(TimeStamp, addition)
{
    const TimeStamp a(123, 456);
    const TimeStamp b(567, 789);

    const TimeStamp sum = a + b;

    EXPECT_EQ(sum.getSeconds(), 123 + 567);
    EXPECT_EQ(sum.getMicroSeconds(), 456 + 789);
}

TEST(TimeStamp, addition_negative_time)
{
    const TimeStamp a(-10000001000);
    const TimeStamp b(-20000002000);

    const TimeStamp sum = a + b;

    EXPECT_EQ(sum.getNanoSeconds(), -30000003000);
    EXPECT_EQ(sum.getSeconds(), -30);
    EXPECT_EQ(sum.getMicroSeconds(), 3);
}

TEST(TimeStamp, addition_rollover)
{
    const TimeStamp a(123, 999999);
    const TimeStamp b(567, 2);

    const TimeStamp sum = a + b;

    EXPECT_EQ(sum.getSeconds(), 123 + 567 + 1);
    EXPECT_EQ(sum.getMicroSeconds(), 1);
}

TEST(TimeStamp, subtraction)
{
    const TimeStamp a(123, 1);
    const TimeStamp b(567, 2000);

    const TimeStamp sum = b - a;

    EXPECT_EQ(sum.getSeconds(), 567 - 123);
    EXPECT_EQ(sum.getMicroSeconds(), 2000 - 1);
}

TEST(TimeStamp, subtraction_negative_time)
{
    const TimeStamp a(-10000001000);
    const TimeStamp b(-20000002000);

    const TimeStamp sum = a - b;

    EXPECT_EQ(sum.getNanoSeconds(), 10000001000);
    EXPECT_EQ(sum.getSeconds(), 10);
    EXPECT_EQ(sum.getMicroSeconds(), 1);
}

TEST(TimeStamp, subtraction_rollover)
{
    const TimeStamp a(123, 2000);
    const TimeStamp b(567, 1);

    const TimeStamp sum = a - b;

    EXPECT_EQ(sum.getSeconds(), 123 - 567 + 1);
    EXPECT_EQ(sum.getMicroSeconds(), 1000000 - 2000 + 1);
}

#endif /* #ifndef CRL_MULTISENSE_TIMESTAMP_TEST_HH */
