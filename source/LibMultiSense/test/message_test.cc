/**
 * @file message_test.cc
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

#include <details/legacy/message.hh>

#include <wire/SysDeviceInfoMessage.hh>

using namespace multisense::legacy;

TEST(unwrap_sequence_id, null)
{
    auto full_sequence_id = unwrap_sequence_id(0, -1, 0);

    EXPECT_EQ(full_sequence_id, 0);

    full_sequence_id = unwrap_sequence_id(0, 0, 0);

    EXPECT_EQ(full_sequence_id, 0);
}

TEST(unwrap_sequence_id, no_increment)
{
    const auto full_sequence_id = unwrap_sequence_id(1, 1, 0);

    EXPECT_EQ(full_sequence_id, 0);
}

TEST(unwrap_sequence_id, increment)
{
    const auto full_sequence_id = unwrap_sequence_id(1, 0, 0);

    EXPECT_EQ(full_sequence_id, 1);
}

TEST(unwrap_sequence_id, rollover)
{
    const auto full_sequence_id = unwrap_sequence_id(0, 65535, 0);

    EXPECT_EQ(full_sequence_id, 1);
}

TEST(unwrap_sequence_id, rollover_unique)
{
    const auto full_sequence_id = unwrap_sequence_id(0, 65535, 123456);

    EXPECT_EQ(full_sequence_id, 123457);
}

TEST(header_valid, invalid)
{
   std::vector<uint8_t> data(50, 0);

   ASSERT_FALSE(header_valid(data));
}

TEST(header_valid, valid)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info{};

    auto serialized = serialize(info, 10, 9000);

    ASSERT_TRUE(header_valid(serialized));
}

TEST(get_message_type, basic)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info{};

    auto serialized = serialize(info, 10, 9000);

    const auto type = get_message_type(serialized);

    ASSERT_EQ(type, wire::SysDeviceInfo::ID);
}

TEST(get_full_message_size, invalid)
{
    const auto full_size = get_full_message_size({});

    ASSERT_FALSE(static_cast<bool>(full_size));
}

TEST(get_full_message_size, valid)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info{};

    const auto serialized = serialize(info, 10, 9000);

    const auto full_size = get_full_message_size(serialized);

    ASSERT_TRUE(static_cast<bool>(full_size));

    ASSERT_GT(full_size.value(), 20);
    ASSERT_LT(full_size.value(), 200);
}

TEST(seralize_deseralize, roundtrip)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info{};
    info.name = "test";
    info.numberOfPcbs = 0;
    info.motorName = "bar";

    auto serialized = serialize(info, 10, 9000);

    //
    // Remove the wire::Header we added for the MultiSense
    //
    serialized.erase(std::begin(serialized), std::begin(serialized) + sizeof(wire::Header));


    const auto round_trip = deserialize<wire::SysDeviceInfo>(serialized);

    ASSERT_EQ(round_trip.name, info.name);
    ASSERT_EQ(round_trip.numberOfPcbs, info.numberOfPcbs);
    ASSERT_EQ(round_trip.motorName, info.motorName);
}

TEST(MessageAssembler, process_notify_wait_for)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{10, 9000, 2, 100000})};
    auto registration = assembler.register_message(wire::SysDeviceInfo::ID);
    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto output = registration->wait<wire::SysDeviceInfo>(std::make_optional(500ms));

    ASSERT_TRUE(static_cast<bool>(output));

    ASSERT_EQ(output->name, info.name);
}


TEST(MessageAssembler, process_notify_wait)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_wait";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{10, 9000, 2, 100000})};
    auto registration = assembler.register_message(wire::SysDeviceInfo::ID);
    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto output = registration->wait<wire::SysDeviceInfo>();

    ASSERT_TRUE(static_cast<bool>(output));

    ASSERT_EQ(output->name, info.name);
}

TEST(MessageAssembler, process_notify_wait_for_multi_registrations)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{10, 9000, 2, 100000})};
    auto registration0 = assembler.register_message(wire::SysDeviceInfo::ID);
    auto registration1 = assembler.register_message(wire::SysDeviceInfo::ID);
    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto output0 = registration0->wait<wire::SysDeviceInfo>(std::make_optional(500ms));
    const auto output1 = registration1->wait<wire::SysDeviceInfo>(std::make_optional(500ms));

    ASSERT_TRUE(static_cast<bool>(output0));
    ASSERT_TRUE(static_cast<bool>(output1));

    ASSERT_EQ(output0->name, info.name);
    ASSERT_EQ(output1->name, info.name);
}

TEST(MessageAssembler, process_notify_remove_registration)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{10, 9000, 2, 100000})};
    auto registration = assembler.register_message(wire::SysDeviceInfo::ID);
    assembler.remove_registration(wire::SysDeviceInfo::ID);
    ASSERT_TRUE(assembler.process_packet(serialized));

    //
    // Since we don't have an active registration we should not get a valid message
    //
    const auto output = registration->wait<wire::SysDeviceInfo>(std::make_optional(500ms));

    ASSERT_FALSE(static_cast<bool>(output));
}

TEST(MessageAssembler, process_callback)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    //
    // Our lambda should be called when the packet is processed
    //
    wire::SysDeviceInfo output;
    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{10, 9000, 2, 100000})};
    assembler.register_callback(wire::SysDeviceInfo::ID,
                               [&output](const auto &data)
                               {
                                   output = deserialize<wire::SysDeviceInfo>(*data);
                               });
    ASSERT_TRUE(assembler.process_packet(serialized));

    ASSERT_EQ(output.name, info.name);

    //
    // Remove our callback and make sure our output does not get updated
    //
    assembler.remove_callback(wire::SysDeviceInfo::ID);

    info.name = "test_callback_new";
    serialized = serialize(info, 11, 9000);
    ASSERT_TRUE(assembler.process_packet(serialized));

    ASSERT_NE(output.name, info.name);
}

TEST(MessageAssembler, use_all_buffers)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    //
    // Our lambda should be called when the packet is processed
    //
    std::vector<std::shared_ptr<const std::vector<uint8_t>>> outputs;
    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{2, 9000, 1, 100000})};
    assembler.register_callback(wire::SysDeviceInfo::ID,
                               [&outputs](const auto data)
                               {
                                   outputs.push_back(data);
                               });
    ASSERT_TRUE(assembler.process_packet(serialized));
    ASSERT_TRUE(assembler.process_packet(serialized));

    //
    // At this point we should be out of our small buffers
    //
    ASSERT_FALSE(assembler.process_packet(serialized));
}

TEST(MessageAssembler, invalid_message)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    //
    // Mess up our message
    //
    serialized[0] = 123;

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{2, 9000, 1, 100000})};

    //
    // Processing should return false
    //
    ASSERT_FALSE(assembler.process_packet(serialized));
    ASSERT_FALSE(assembler.process_packet(std::vector<uint8_t>{}));
}

TEST(MessageAssembler, only_large_buffers)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{1, 1, 1, 100000})};
    auto registration = assembler.register_message(wire::SysDeviceInfo::ID);

    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto output = registration->wait<wire::SysDeviceInfo>(std::make_optional(500ms));

    ASSERT_TRUE(static_cast<bool>(output));

    ASSERT_EQ(output->name, info.name);
}

TEST(MessageAssembler, multi_large_packets)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 0, 9000);

    //
    // Update our message size to indicate our message is huge
    //
    auto sequence_id = reinterpret_cast<uint16_t*>(&serialized[8]);
    auto message_length = reinterpret_cast<uint32_t*>(&serialized[10]);
    *message_length = 10000;

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{1, 1, 1, 100000})};
    auto registration = assembler.register_message(wire::SysDeviceInfo::ID);

    //
    // We should be able to process the message, but we wont get a valid message since we messed with
    // the message length. We should be internally dropping all these messages when a message with the
    // next sequence id shows up
    //
    for (size_t i = 1 ; i < 100 ; ++i)
    {
        ASSERT_TRUE(assembler.process_packet(serialized));
        {
            const auto output = registration->wait<wire::SysDeviceInfo>(std::make_optional(1us));
            ASSERT_FALSE(static_cast<bool>(output));
        }

        *sequence_id = i;
    }
}

TEST(MessageAssembler, stats_valid_message)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{2, 9000, 1, 100000})};

    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto stats = assembler.get_message_statistics();

    ASSERT_EQ(stats.received_messages, 1);
    ASSERT_EQ(stats.dropped_messages, 0);
    ASSERT_EQ(stats.invalid_packets, 0);
}

TEST(MessageAssembler, stats_invalid_message)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    //
    // Mess up our message
    //
    serialized[0] = 123;

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{2, 9000, 1, 100000})};

    //
    // Processing should return false
    //
    ASSERT_FALSE(assembler.process_packet(serialized));
    ASSERT_FALSE(assembler.process_packet(std::vector<uint8_t>{}));

    const auto stats = assembler.get_message_statistics();

    ASSERT_EQ(stats.received_messages, 0);
    ASSERT_EQ(stats.dropped_messages, 0);
    ASSERT_EQ(stats.invalid_packets, 2);
}

TEST(MessageAssembler, stats_dropped_valid_message)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::SysDeviceInfo info{};
    info.name = "test_callback";
    auto serialized = serialize(info, 10, 9000);

    MessageAssembler assembler{std::make_shared<BufferPool>(BufferPoolConfig{2, 9000, 1, 100000})};

    //
    // Send in 3 messages. One valid, one of a very large size, one valid
    //
    ASSERT_TRUE(assembler.process_packet(serialized));

    serialized = serialize(info, 11, 9000);
    wire::Header& header = *(reinterpret_cast<wire::Header*>(serialized.data()));
    header.messageLength = 100000;
    ASSERT_TRUE(assembler.process_packet(serialized));

    serialized = serialize(info, 12, 9000);
    ASSERT_TRUE(assembler.process_packet(serialized));

    const auto stats = assembler.get_message_statistics();

    ASSERT_EQ(stats.received_messages, 2);
    ASSERT_EQ(stats.dropped_messages, 1);
    ASSERT_EQ(stats.invalid_packets, 0);
}
