/**
 * @file message.hh
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

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>

#include <utility/BufferStream.hh>
#include <wire/Protocol.hh>
#include <wire/AckMessage.hh>

#include "details/legacy/storage.hh"

namespace multisense{
namespace legacy{

///
/// @brief deserialize the raw bytes of a message. Note this does not account for the wire::Header which
///        is transmitted as part of most messages
///
template <typename T>
T deserialize(const std::vector<uint8_t>& data)
{
    using namespace crl::multisense::details;

    utility::BufferStreamReader stream{data.data(), data.size()};

    wire::IdType id = 0;
    wire::VersionType version = 0;

    stream & id;
    stream & version;
    T m(stream, version);

    return m;
}

///
/// @brief Unwrap the wire ID into a full 16 bit sequence ID
///
int64_t unwrap_sequence_id(uint16_t current_wire_id, int32_t previous_wire_id, int64_t current_sequence_id);

///
/// @brief Validate the Multisense header
///
bool header_valid(const std::vector<uint8_t> &raw_data);

///
/// @brief Get the message type of the message from the buffer over the wire. Note this does account
///        for the wire::Header that is transmitted from the camera
///
crl::multisense::details::wire::IdType get_message_type(const std::vector<uint8_t>& raw_buffer);

///
/// @brief Get the size of the full message in bytes from a raw data buffer
///
std::optional<uint32_t> get_full_message_size(const std::vector<uint8_t> &raw_data);

///
/// @brief Serialize a MultiSense Wire message for transmission. This adds the wire header to the message
///        for transmission
///
template <typename T>
std::vector<uint8_t> serialize(const T& message, uint16_t sequence_id, size_t mtu)
{
    using namespace crl::multisense::details;

    const wire::IdType id = T::ID;
    const wire::VersionType version = T::VERSION;

    std::vector<uint8_t> output_buffer(mtu - wire::COMBINED_HEADER_LENGTH, static_cast<uint8_t>(0));

    utility::BufferStreamWriter stream(output_buffer.data(), output_buffer.size());

    wire::Header& header = *(reinterpret_cast<wire::Header*>(stream.data()));
    header.magic = wire::HEADER_MAGIC;
    header.version = wire::HEADER_VERSION;
    header.group = wire::HEADER_GROUP;
    header.flags = 0;
    header.sequenceIdentifier = sequence_id;

    stream.seek(sizeof(wire::Header));
    stream & id;
    stream & version;
    const_cast<T*>(&message)->serialize(stream, version);

    header.messageLength = static_cast<uint32_t>(stream.tell() - sizeof(wire::Header));
    header.byteOffset = 0;

    output_buffer.resize(stream.tell());

    return output_buffer;
}

///
/// @brief A condition object which can be used to wait on messages from the stream
///
class MessageCondition
{
public:

    MessageCondition() = default;

    ~MessageCondition()
    {
        m_cv.notify_all();
    }

    void unset()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_notified = false;
    }

    void set_and_notify(std::shared_ptr<std::vector<uint8_t>> data)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data = *data;
        m_notified = true;
        m_cv.notify_all();
    }

    ///
    /// @brief convenience function equivalent to std::condition_variable::wait_for that performs a type
    ///        conversion to the target type
    ///
    template <typename T, class Rep, class Period>
    std::optional<T> wait(const std::optional<std::chrono::duration<Rep, Period>> &timeout)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if (timeout)
        {
            if (m_cv.wait_for(lock, timeout.value(), [this]{return this->m_notified;}))
            {
                return std::make_optional(deserialize<T>(m_data));
            }
        }
        else
        {
            m_cv.wait(lock, [this]{return this->m_notified;});
            return std::make_optional(deserialize<T>(m_data));
        }

        return std::nullopt;
    }

    template <typename T>
    std::optional<T> wait()
    {
        const std::optional<std::chrono::milliseconds> timeout = std::nullopt;
        return wait<T>(timeout);
    }

private:

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::vector<uint8_t> m_data;
    bool m_notified = false;

};

struct MessageStatistics
{
    ///
    /// @brief The number of received messages
    ///
    size_t received_messages = 0;

    ///
    /// @brief The number of dropped messages
    ///
    size_t dropped_messages = 0;

    ///
    /// @brief The number of invalid packets we received
    ///
    size_t invalid_packets = 0;
};

///
/// @brief Process incoming network data, and try the data into valid MultiSense Wire messages
///
class MessageAssembler
{
public:
    MessageAssembler(std::shared_ptr<BufferPool> buffer_pool);

    ~MessageAssembler() = default;

    bool process_packet(const std::vector<uint8_t> &raw_data);

    ///
    /// @brief Register to be notified when a message of a given id arrives. Note this registration will only
    ///        receive a single message. To receive multiple messages please use the callback interface
    ///
    std::shared_ptr<MessageCondition> register_message(const crl::multisense::details::wire::IdType &message_id);

    ///
    /// @brief Remove a registration for a specific message type
    ///
    void remove_registration(const crl::multisense::details::wire::IdType &message_id);

    ///
    /// @brief Register a callback to receive valid messages of a given id. Note currently only one callback
    ///        can be registered for a message_id. Note this will pass the raw image buffer to the callback.
    ///        It's up to the user to deserialize the message into it's appropriate type
    ///
    void register_callback(const crl::multisense::details::wire::IdType& message_id,
                           std::function<void(std::shared_ptr<const std::vector<uint8_t>>)> callback);

    ///
    /// @brief Remove a callback for a specific message type
    ///
    void remove_callback(const crl::multisense::details::wire::IdType& message_id);

    MessageStatistics get_message_statistics() const
    {
        return MessageStatistics{m_dispatched_messages,
                                 (m_processing_messages ? (m_received_messages - 1 - m_dispatched_messages) :
                                                          (m_received_messages - m_dispatched_messages)),
                                 m_invalid_packets};
    }

private:

    ///
    /// @brief An internal message object used to track how many bytes were written to a data buffer
    ///
    struct InternalMessage
    {
        crl::multisense::details::wire::IdType type = 0;
        size_t bytes_written = 0;
        std::shared_ptr<std::vector<uint8_t>> data;
    };

    ///
    /// @brief Get a new buffer that can hold a message of a given size. If no buffers are available try
    ///        destroying older buffers we may no longer be using. The id's of these buffers are stored
    ///        in the input ordered_messages
    ///
    std::tuple<std::shared_ptr<std::vector<uint8_t>>, std::deque<int64_t>>
        get_buffer(uint32_t message_size, std::deque<int64_t> ordered_messages);

    ///
    /// @brief Write raw data to an message
    ///
    bool write_data(InternalMessage &message, const std::vector<uint8_t> &raw_data);

    ///
    /// @brief Dispatch to both our registrations and callbacks
    ///
    void dispatch(const crl::multisense::details::wire::IdType& message_id,
                  std::shared_ptr<std::vector<uint8_t>> data);

    ///
    /// @brief Mutex to ensure registrations MessageAssembler are thread safe
    ///
    std::mutex m_condition_mutex;

    ///
    /// @brief Mutex to ensure callback calls into the MessageAssembler are thread safe
    ///
    std::mutex m_callback_mutex;

    ///
    /// @brief Buffer pool used to allocate messages into
    ///
    std::shared_ptr<BufferPool> m_buffer_pool = nullptr;

    ///
    /// @brief Internal id used to detect internal rollover of the 16 bit wire id
    ///
    int32_t m_previous_wire_id = -1;

    ///
    /// @brief Internal cache of our last sequence id
    ///
    int64_t m_current_sequence_id = 0;

    ///
    /// @brief Tracking for the ordering of the small buffers we have allocated. Used to determine
    ///        which active message to potentially remove
    ///
    std::deque<int64_t> m_small_ordered_messages;
    ///
    /// @brief Tracking for the ordering of the large buffers we have allocated. Used to determine
    ///        which active message to potentially remove
    ///
    std::deque<int64_t> m_large_ordered_messages;

    ///
    /// @brief Active messages we are accumulating
    ///
    std::map<int64_t, InternalMessage> m_active_messages;

    ///
    /// @brief Conditions which we are tracking. These are notified when a message of a given type is
    ///        fully received
    ///
    std::map<crl::multisense::details::wire::IdType, std::shared_ptr<MessageCondition>> m_conditions;

    ///
    /// @brief Callbacks which we are tracking. These are called when a message of a given type is
    ///        fully received
    ///
    std::map<crl::multisense::details::wire::IdType,
             std::function<void(std::shared_ptr<const std::vector<uint8_t>>)>> m_callbacks;

    ///
    /// @brief A counter for the number of messages we have received
    ///
    std::atomic_uint64_t m_received_messages = 0;

    ///
    /// @brief The a flag which indicates if a message is being processed
    ///
    std::atomic_bool m_processing_messages = false;

    ///
    /// @brief A counter for the number of messages we dispatched
    ///
    std::atomic_uint64_t m_dispatched_messages = 0;

    ///
    /// @brief A counter for the number of invalid packets we received
    ///
    std::atomic_uint64_t m_invalid_packets = 0;
};

}
}
