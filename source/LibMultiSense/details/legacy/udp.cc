/**
 * @file udp.cc
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

#include <cstring>
#include <functional>
#include <iostream>

#include <utility/Exception.hh>

#include "details/legacy/udp.hh"

namespace multisense{
namespace legacy{

UdpReceiver::UdpReceiver(const NetworkSocket &socket,
                         size_t max_mtu,
                         std::function<void(const std::vector<uint8_t>&)> receive_callback,
                         size_t max_packet_queue_depth):
    m_socket(socket.sensor_socket),
    m_stop(false),
    m_max_packet_queue_depth(max_packet_queue_depth),
    m_max_mtu(max_mtu),
    m_packet_buffers(max_packet_queue_depth, std::vector<uint8_t>(max_mtu, 0)),
    m_receive_callback(receive_callback)
{
    for (size_t i = 0; i < m_packet_buffers.size(); ++i)
    {
        m_free_buffers.emplace_back(i);
    }

    m_rx_thread = std::thread(&UdpReceiver::rx_thread, this);
    m_dispatch_thread = std::thread(&UdpReceiver::dispatch_thread, this);
}

UdpReceiver::~UdpReceiver()
{
    m_stop.store(true);
    m_queue_valid.notify_all();

    if (m_rx_thread.joinable())
    {
        m_rx_thread.join();
    }

    if (m_dispatch_thread.joinable())
    {
        m_dispatch_thread.join();
    }
}

void UdpReceiver::rx_thread()
{
    fd_set read_set;
    while(!m_stop)
    {
        FD_ZERO(&read_set);
        FD_SET(m_socket, &read_set);

        struct timeval tv = {0, 200000}; // 5Hz
#ifdef WIN32
        // Windows is "special" and doesn't have the same call semantics as posix
        const int result = select(1, &read_set, NULL, NULL, &tv);
#else
        const int result = select(m_socket + 1, &read_set, NULL, NULL, &tv);
#endif
        if (result <= 0)
        {
            continue;
        }

        //
        // Read all the data currently sitting on the socket
        //
        while(true)
        {
            size_t buffer_index = 0;
            {
                std::lock_guard<std::mutex> lock(m_queue_mutex);
                if (m_free_buffers.empty())
                {
                    ++m_dropped_packets;
                    continue;
                }

                buffer_index = m_free_buffers.front();
                m_free_buffers.pop_front();
            }

            try
            {
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
            auto& buffer = m_packet_buffers[buffer_index];
            buffer.resize(m_max_mtu);
            const int bytes_read = recvfrom(m_socket,
                                            reinterpret_cast<char*>(buffer.data()),
                                            buffer.size(),
                                            0, NULL, NULL);
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (pop)
#endif
                //
                // Nothing left to read
                //
                if (bytes_read < 0)
                {
                    std::lock_guard<std::mutex> lock(m_queue_mutex);
                    m_free_buffers.emplace_back(buffer_index);
                    break;
                }

                buffer.resize(static_cast<size_t>(bytes_read));

                {
                    std::lock_guard<std::mutex> lock(m_queue_mutex);
                    m_ready_buffers.emplace_back(buffer_index);
                }

                m_queue_valid.notify_one();
            }
            catch (const std::exception& e)
            {
                CRL_DEBUG("exception while decoding packet: %s\n", e.what());
            }
            catch ( ... )
            {
                CRL_DEBUG_RAW("unknown exception while decoding packet\n");
            }
        }
    }
}

void UdpReceiver::dispatch_thread()
{
    while(true)
    {
        size_t buffer_index = 0;
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            m_queue_valid.wait(lock, [this]()
            {
                return m_stop || !m_ready_buffers.empty();
            });

            if (m_stop && m_ready_buffers.empty())
            {
                break;
            }

            buffer_index = m_ready_buffers.front();
            m_ready_buffers.pop_front();
        }

        try
        {
            auto& packet = m_packet_buffers[buffer_index];
            m_receive_callback(packet);
        }
        catch (const std::exception& e)
        {
            CRL_DEBUG("exception while decoding packet: %s\n", e.what());
        }
        catch ( ... )
        {
            CRL_DEBUG_RAW("unknown exception while decoding packet\n");
        }

        {
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            m_packet_buffers[buffer_index].resize(m_max_mtu);
            m_free_buffers.emplace_back(buffer_index);
        }
    }
}

int64_t publish_data(const NetworkSocket &socket, const std::vector<uint8_t> &data)
{
// disable MSVC warning for narrowing conversion.
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
    sockaddr_in* raw_address = socket.sensor_address.get();
    const auto ret = sendto(socket.sensor_socket, (char*) data.data(), data.size(), 0,
                               (struct sockaddr *) raw_address,
                               sizeof(*raw_address));
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (pop)
#endif

    if (static_cast<size_t>(ret) != data.size())
    {
        CRL_EXCEPTION("error sending data to sensor, %d/%d bytes written: %s",
                      ret, data.size(), strerror(errno));
    }

    return ret;
}

}
}
