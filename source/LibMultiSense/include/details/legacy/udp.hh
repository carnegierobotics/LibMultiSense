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
                std::function<void(const std::vector<uint8_t>&)> receive_callback);

    ~UdpReceiver();

private:

    ///
    /// @brief The rx thread function which is receives UDP data
    ///
    void rx_thread();

    ///
    /// @brief The internal socket which UDP data is receive on
    ///
    socket_t m_socket;

    ///
    /// @brief The rx_thread object which is spawned on construction
    ///
    std::thread m_rx_thread;

    ///
    /// @brief Atomic flag to stop the rx_thread on destruction
    ///
    std::atomic_bool m_stop{false};

    ///
    /// @brief The amount of data to read off the socket during each read operation
    ///
    size_t m_max_mtu = 0;

    ///
    /// @brief Internal buffer used to write incoming UDP data into
    ///
    std::vector<uint8_t> m_incoming_buffer;

    ///
    /// @brief User specified callback which is called once UDP data is received
    ///
    std::function<void(const std::vector<uint8_t>&)> m_receive_callback;
};


///
/// @brief Convenience function used to user specified data out on the host's UDP socket
///
int64_t publish_data(const NetworkSocket &socket, const std::vector<uint8_t> &data);

}
}
