/**
 * @file udp.hh
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "details/legacy/ip.hh"

namespace multisense{
namespace legacy{

///
/// @brief Convenience network socket object which contains the data corresponding to our connection
///
struct NetworkSocket
{
    std::unique_ptr<sockaddr_in> sensor_address = nullptr;
    socket_t sensor_socket;
    uint16_t server_socket_port = 0;
};

///
/// @brief Convenience object which receives data from a UDP socket and dispatches to a user defined callback.
///        This object internally manages a single receive thread which is used to read data off the socket, and
///        dispatch to the user callback. Note that the dispatch is serial with the UDP receive commands, so
///        blocking callbacks will cause data to be dropped
///
class UdpReceiver
{
public:

    ///
    /// @brief Construct a UDP receiver for a given socket, and call a user callback whenever there is data
    ///        available. The max_mtu will be the size of the internal buffer used to read data from the
    ///        socket
    ///
    UdpReceiver(const NetworkSocket &socket,
                size_t max_mtu,
                std::function<void(const std::vector<uint8_t>&)> receive_callback,
                size_t max_queue_bytes=16777215);

    ~UdpReceiver();

    size_t dropped_packets() const
    {
        return m_dropped_packets;
    }

private:

    ///
    /// @brief The rx thread function which is receives UDP data
    ///
    void rx_thread();

    ///
    /// @brief The dispatch thread function which is sends UDP data to clients
    ///
    void dispatch_thread();

    ///
    /// @brief The internal socket which UDP data is receive on
    ///
    socket_t m_socket;

    ///
    /// @brief The rx_thread object which is spawned on construction to receive UDP data
    ///
    std::thread m_rx_thread;

    ///
    /// @brief The dispatch_thread object which is spawned on construction to dispatch data to clients
    ///
    std::thread m_dispatch_thread;

    ///
    /// @brief Atomic flag to stop the rx_thread on destruction
    ///
    std::atomic_bool m_stop{false};

    ///
    /// @brief Signal for coordinating when the processing queue is valid
    ///
    std::condition_variable m_queue_valid;

    ///
    /// @brief Signal used to wake the rx thread when buffers are freed
    ///
    std::condition_variable m_free_buffer_available;

    ///
    /// @brief Mutext for coordinating when data is ready to process
    ///
    std::mutex m_queue_mutex;

    ///
    /// @brief The amount of data to read off the socket during each read operation
    ///
    size_t m_max_mtu = 0;

    ///
    /// @brief queue used to send data between the rx and dispatch threads. The queue stores indices
    ///        into m_packet_buffers to avoid copying the payload
    ///
    std::vector<std::vector<uint8_t>> m_packet_buffers;

    ///
    /// @brief scratch buffer used when packets have to be drained from the socket
    ///
    std::vector<uint8_t> m_drop_buffer;

    ///
    /// @brief indices of buffers which are free to write to in m_packet_buffers
    ///
    std::deque<size_t> m_free_buffers;

    ///
    /// @brief indices of buffers which are ready to process in m_packet_buffers
    ///
    std::deque<size_t> m_ready_buffers;

    ///
    /// @brief User specified callback which is called once UDP data is received
    ///
    std::function<void(const std::vector<uint8_t>&)> m_receive_callback;

    ///
    /// @brief Counter for messages dropped due to dispatch processing slowdowsn
    ///
    std::atomic_size_t m_dropped_packets = 0;

    ///
    /// @brief Types of packets we may encounter
    ///
    enum class PacketCategory
    {
        STREAMING,
        CONTROL
    };

    ///
    /// @brief Drain a single packet from the socket without enqueueing it
    ///
    bool drop_next_packet();

    ///
    /// @brief Determine the category of the next packet waiting on the socket
    ///
    PacketCategory next_packet_category();

    ///
    /// @brief Block until a buffer is available, or drop streaming data if needed
    ///
    bool wait_or_drop();
};


///
/// @brief Convenience function used to user specified data out on the host's UDP socket
///
int64_t publish_data(const NetworkSocket &socket, const std::vector<uint8_t> &data);

}
}
