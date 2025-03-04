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

#include <functional>
#include <iostream>

#include <utility/Exception.hh>

#include "details/legacy/udp.hh"

namespace multisense{
namespace legacy{

UdpReceiver::UdpReceiver(const NetworkSocket &socket,
                         size_t max_mtu,
                         std::function<void(const std::vector<uint8_t>&)> receive_callback):
    m_socket(socket.sensor_socket),
    m_stop(false),
    m_max_mtu(max_mtu),
    m_incoming_buffer(max_mtu, 0),
    m_receive_callback(receive_callback)

{
    m_rx_thread = std::thread(&UdpReceiver::rx_thread, this);
}

UdpReceiver::~UdpReceiver()
{
    if(!m_stop.exchange(true))
    {
        if (m_rx_thread.joinable())
        {
            m_rx_thread.join();
        }
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

            try
            {
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
            const int bytes_read = recvfrom(m_socket,
                                            reinterpret_cast<char*>(m_incoming_buffer.data()),
                                            m_incoming_buffer.size(),
                                            0, NULL, NULL);
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (pop)
#endif
                //
                // Nothing left to read
                //
                if (bytes_read < 0)
                {
                    break;
                }

                m_incoming_buffer.resize(bytes_read);
                m_receive_callback(m_incoming_buffer);
                m_incoming_buffer.resize(m_max_mtu);
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
