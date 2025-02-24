/**
 * @file ip.cc
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

#include <utility/Exception.hh>

#include "details/legacy/ip.hh"

namespace multisense{
namespace legacy{

std::unique_ptr<sockaddr_in> get_sockaddr(const std::string &ip_address, uint16_t command_port)
{
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = 0;

    const int addrstatus = getaddrinfo(ip_address.c_str(), NULL, &hints, &res);
    if (addrstatus != 0 || res == NULL)
    {
        CRL_EXCEPTION("unable to resolve \"%s\": %s", ip_address.c_str(), strerror(errno));
    }

    in_addr addr;
    memcpy(&addr, &(((struct sockaddr_in *)(res->ai_addr))->sin_addr), sizeof(in_addr));

    auto socaddr = std::unique_ptr<sockaddr_in>(new sockaddr_in);
    socaddr->sin_family = AF_INET;
    socaddr->sin_port = htons(command_port);
    socaddr->sin_addr = addr;

    freeaddrinfo(res);

    return socaddr;
}

std::tuple<socket_t, uint16_t> bind(const std::optional<std::string>& interface_name)
{
    //
    // Create the socket.

    auto server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#if defined(__MINGW64__)
    if (server_socket == INVALID_SOCKET)
#else
    if (server_socket < 0)
#endif
        CRL_EXCEPTION("failed to create the UDP socket: %s",
                      strerror(errno));

    #if __linux__
        //
        // Bind to spcific interface if specified
        if (interface_name && !interface_name->empty())
        {
            if (0 != setsockopt(server_socket,
                                SOL_SOCKET,
                                SO_BINDTODEVICE,
                                interface_name->c_str(),
                                interface_name->size()))
            {
                CRL_EXCEPTION("Failed to bind to device %s. Error: %s", interface_name->c_str(),
                              strerror(errno));
            }
        }
    #else
        if (interface_name && !interface_name->empty())
        {
            CRL_DEBUG("User specified binding to adapter %s, but this feature is only supported under linux."
                      "Ignoring bind to specific adapter", interface_name->c_str());
        }
    #endif

    //
    // Turn non-blocking on.
#if WIN32
    u_long ioctl_arg = 1;
    if (0 != ioctlsocket(server_socket, FIONBIO, &ioctl_arg))
        CRL_EXCEPTION("failed to make a socket non-blocking: %d",WSAGetLastError ());
#else
    const int flags = fcntl(server_socket, F_GETFL, 0);

    if (0 != fcntl(server_socket, F_SETFL, flags | O_NONBLOCK))
        CRL_EXCEPTION("failed to make a socket non-blocking: %s",
                      strerror(errno));
#endif

    //
    // Allow reusing sockets.

    int reuseSocket = 1;

    if (0 != setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, (char*) &reuseSocket,
                        sizeof(reuseSocket)))
        CRL_EXCEPTION("failed to turn on socket reuse flag: %s",
                      strerror(errno));

    //
    // We want very large buffers to store several images

#if __APPLE__
    // MacOS cannot reliably allocate a buffer larger than this
    int bufferSize = 4 * 1024 * 1024;
#else
    int bufferSize = 48 * 1024 * 1024;
#endif

    if (0 != setsockopt(server_socket, SOL_SOCKET, SO_RCVBUF, (char*) &bufferSize,
                        sizeof(bufferSize)) ||
        0 != setsockopt(server_socket, SOL_SOCKET, SO_SNDBUF, (char*) &bufferSize,
                        sizeof(bufferSize)))
    {
        CRL_EXCEPTION("failed to adjust socket buffer sizes (%d bytes): %s",
                      bufferSize, strerror(errno));
    }

    //
    // Bind the connection to the port.

    struct sockaddr_in address;

    address.sin_family      = AF_INET;
    address.sin_port        = htons(0); // system assigned
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (0 != ::bind(server_socket, (struct sockaddr*) &address, sizeof(address)))
        CRL_EXCEPTION("failed to bind the server socket to system-assigned port: %s",
                      strerror(errno));

    //
    // Retrieve the system assigned local UDP port
#if WIN32
    int len = sizeof(address);
#else
    socklen_t len = sizeof(address);
#endif
    if (0 != getsockname(server_socket, (struct sockaddr*) &address, &len))
    {
        CRL_EXCEPTION("getsockname() failed: %s", strerror(errno));
    }

    auto server_socket_port = htons(address.sin_port);

    return std::make_tuple(server_socket, server_socket_port);
}

}
}
