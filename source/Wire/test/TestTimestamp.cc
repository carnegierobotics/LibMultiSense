/**
 * @file TestTimestamp.hh
 *
 * Copyright 2023-2025
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

#include "utility/TimeStamp.hh"

#include <gtest/gtest.h>

using namespace crl::multisense::details::utility;

TEST(Construction, default_)
{
    const TimeStamp a;

    EXPECT_EQ(a.getSeconds(), 0);
    EXPECT_EQ(a.getMicroSeconds(), 0);
}

TEST(Construction, second_microsecond)
{
    for (int32_t sec = -1000000; sec <= 1000000; sec += 100000)
    {
        for (int32_t usec = -1000000; usec <= 1100000; usec += 10000)
        {
            const TimeStamp time(sec, usec);

            EXPECT_EQ(time.getSeconds() * 1000000 + time.getMicroSeconds(),
                      sec * 1000000 + usec);

            if (usec >= 0 && usec < 1000000)
            {
                EXPECT_EQ(time.getSeconds(), sec);
                EXPECT_EQ(time.getMicroSeconds(), usec);
            }
        }
    }
}

TEST(Construction, timeval)
{
    for (int32_t sec = -1000000; sec <= 1000000; sec += 100000)
    {
        for (int32_t usec = -1000000; usec <= 1100000; usec += 10000)
        {
            struct timeval tv;
            tv.tv_sec = sec;
            tv.tv_usec = usec;

            const TimeStamp time(tv);

            EXPECT_EQ(time.getSeconds() * 1000000 + time.getMicroSeconds(),
                      sec * 1000000 + usec);

            if (usec >= 0 && usec < 1000000)
            {
                EXPECT_EQ(time.getSeconds(), sec);
                EXPECT_EQ(time.getMicroSeconds(), usec);
            }
        }
    }
}

TEST(Construction, nanoseconds)
{
    // numbers are large enough to check for rollover of int32_t when convering seconds to nanoseconds
    for (int64_t sec = -1000000; sec <= 1000000; sec += 100000)
    {
        for (int64_t usec = -1000000; usec <= 1100000; usec += 10000)
        {
            const int64_t nsec = sec * 1000000000 + usec * 1000;

            const TimeStamp time(nsec);

            EXPECT_EQ(static_cast<int64_t>(time.getSeconds()) * 1000000 + time.getMicroSeconds(),
                      sec * 1000000 + usec);

            if (usec >= 0 && usec < 1000000)
            {
                EXPECT_EQ(time.getSeconds(), sec);
                EXPECT_EQ(time.getMicroSeconds(), usec);
            }
        }
    }
}

// set() functions have been implicitly tested via constructors

TEST(GetNanoSeconds, getNanoSeconds)
{
    // handle values larger than a uint32_t (2,147,483,647) to check rollover
    // increment by 0.1 seconds
    //
    for (int64_t nsec = -100000000000; nsec <= 100000000000; nsec += 100000000)
    {
        const TimeStamp time(nsec);
        EXPECT_EQ(time.getNanoSeconds(), nsec);
    }
}

TEST(Operators, add)
{
    // handle nsec values larger than a uint32_t (2,147,483,647) to check rollover
    // increment by 0.1 seconds
    const int64_t max_nsec = 10000000000;
    const int64_t step_nsec = 100000000;
    for (int64_t a_nsec = -max_nsec; a_nsec <= max_nsec; a_nsec += step_nsec)
    {
        for (int64_t b_nsec = -max_nsec; b_nsec <= max_nsec; b_nsec += step_nsec)
        {
            const TimeStamp a(a_nsec);
            const TimeStamp b(b_nsec);

            EXPECT_EQ((a + b).getNanoSeconds(), a_nsec + b_nsec);
        }
    }
}

TEST(Operators, subtract)
{
    // handle nsec values larger than a uint32_t (2,147,483,647) to check rollover
    // increment by 0.1 seconds
    const int64_t max_nsec = 10000000000;
    const int64_t step_nsec = 100000000;
    for (int64_t a_nsec = -max_nsec; a_nsec <= max_nsec; a_nsec += step_nsec)
    {
        for (int64_t b_nsec = -max_nsec; b_nsec <= max_nsec; b_nsec += step_nsec)
        {
            const TimeStamp a(a_nsec);
            const TimeStamp b(b_nsec);

            EXPECT_EQ((a - b).getNanoSeconds(), a_nsec - b_nsec);
        }
    }
}
