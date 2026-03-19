/**
 * @file utilities_test.cc
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
 *   2025-01-10, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/utilities.hh>

using namespace multisense::legacy;

TEST(get_version, basic)
{
    using namespace crl::multisense::details;

    const wire::VersionType raw_version = 0x0520;

    const auto version = get_version(raw_version);

    ASSERT_EQ(version.major, 5);
    ASSERT_EQ(version.minor, 32);
}

TEST(version, less_than)
{
    using namespace crl::multisense::details;

    const wire::VersionType raw_version0 = 0x0520;
    const auto version0 = get_version(raw_version0);

    const wire::VersionType raw_version1 = 0x0521;
    const auto version1 = get_version(raw_version1);

    ASSERT_TRUE(version0 < version1);
}

TEST(get_disparities, basic)
{
    using namespace multisense;

    ASSERT_EQ(get_disparities(64), multisense::MultiSenseConfig::MaxDisparities::D64);
    ASSERT_EQ(get_disparities(128), multisense::MultiSenseConfig::MaxDisparities::D128);
    ASSERT_EQ(get_disparities(256), multisense::MultiSenseConfig::MaxDisparities::D256);
    EXPECT_THROW(get_disparities(1), std::exception);
}

TEST(convert_sources_from_wire, null)
{
    ASSERT_TRUE(convert_sources(0).empty());
}

TEST(convert_sources_from_wire, all)
{
    ASSERT_GT(convert_sources(all_sources).size(), 13);
}

TEST(convert_sources_from_api, null)
{
    ASSERT_EQ(convert_sources(std::vector<multisense::DataSource>{}), 0);
}

TEST(convert_sources_round_trip, all)
{
    ASSERT_EQ(convert_sources(convert_sources(all_sources)), all_sources);
}

TEST(get_status, status)
{
    using namespace crl::multisense::details::wire;
    using namespace multisense;

    ASSERT_EQ(Status::OK, get_status(Ack::Status_Ok));
    ASSERT_EQ(Status::TIMEOUT, get_status(Ack::Status_TimedOut));
    ASSERT_EQ(Status::INTERNAL_ERROR, get_status(Ack::Status_Error));
    ASSERT_EQ(Status::FAILED, get_status(Ack::Status_Failed));
    ASSERT_EQ(Status::UNSUPPORTED, get_status(Ack::Status_Unsupported));
    ASSERT_EQ(Status::UNKNOWN, get_status(Ack::Status_Unknown));
    ASSERT_EQ(Status::EXCEPTION, get_status(Ack::Status_Exception));
    ASSERT_EQ(Status::UNKNOWN, get_status(100));
}
