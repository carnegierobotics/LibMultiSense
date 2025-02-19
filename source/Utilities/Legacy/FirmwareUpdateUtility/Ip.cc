/**
 * @file FirmwareUpdateUtility/Ip.cc
 *
 * Copyright 2024-2025
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
 *   2024-27-08, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#include "Ip.hh"
#include "Messages.hh"
#include "crc.h"

#include <iostream>

int Ip::Bind()
{
#if WIN32
    int PeerLength = sizeof(struct sockaddr_in);
    int ClientLength = sizeof(struct sockaddr_in);
#else
    socklen_t PeerLength = sizeof(struct sockaddr_in);
    socklen_t ClientLength = sizeof(struct sockaddr_in);
#endif

    if (m_SockFd < 0) {
        std::cerr << "Error: Invalid Socket Descriptor\n";
        return -1;
    }

    m_ClientAddress.sin_family = AF_INET;
    m_ClientAddress.sin_addr.s_addr = 0;
    m_ClientAddress.sin_port = 0;

    m_ServerAddress.sin_family = AF_INET;
    m_ServerAddress.sin_addr.s_addr = inet_addr(m_IpAddress.c_str());
    m_ServerAddress.sin_port = htons(Messages::SERVER_PORT);

    Messages::MessageSetup MsgSetup(m_ClientAddress, m_ServerAddress);
    long int bSent = sendto(m_SockFd, (char *)&MsgSetup, sizeof(MsgSetup), 0,
      (struct sockaddr *)&m_ServerAddress, ClientLength);
    if ((sizeof(Messages::MessageSetup) != bSent) || (bSent < 0))
    {
        std::cerr << "Error Failed to Send Setup Message\n";
        if (bSent < 0)
        {
            std::cerr << SOCKET_ERRNO << std::endl;
        }
        else
        {
            std::cerr << "Byte mismatch! Expected: " << sizeof(Messages::MessageSetup);
            std::cerr << " Sent: " << bSent << std::endl;
        }
        return -1;
    }

    long int bRead = recvfrom(m_SockFd, (char *)&MsgSetup, sizeof(Messages::MessageSetup), 0,
      (struct sockaddr *)&m_ServerAddress, &PeerLength);
    if (sizeof(Messages::MessageSetup) != bRead|| (bRead < 0))
    {
        std::cerr << "Error Failed to Receive Setup Message\n";
        if (bRead < 0)
        {
            std::cerr << SOCKET_ERRNO << std::endl;
        }
        else
        {
            std::cerr << "Byte mismatch Expected: " << sizeof(Messages::MessageSetup);
            std::cerr << " Received: " << bRead << std::endl;
        }
        return -1;
    }

    memcpy(&m_ClientAddress, &MsgSetup.ClientAddress, sizeof(struct sockaddr_in));

    std::cout << "Info: Bound to client\n";
    std::cout << "Server Address: " << inet_ntoa(m_ServerAddress.sin_addr) << std::endl;
    std::cout << "Server Port:    " << m_ServerAddress.sin_port << std::endl;
    std::cout << "Client Address: " << inet_ntoa(m_ClientAddress.sin_addr) << std::endl;
    std::cout << "Client Port:    " << m_ClientAddress.sin_port << std::endl;

    return 0;

}

int Ip::Setup(std::string _IpAddress)
{
    int so_sendbuf = 1024*1024*16;
    int so_recvbuf = 1024*1024*16;
    int so_reuse = 1;
    int ret = 0;

    m_IpAddress = _IpAddress;


#ifdef WIN32
    WSADATA wsaData;
    ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (ret != NO_ERROR) {
        std::cerr << "Failed to init windows socket API!\n";
        std::cerr << SOCKET_ERRNO << std::endl;
        return -1;
    }
#endif


    m_SockFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_SockFd < 0) {
       std::cerr << "Failed to create udp socket!\n";
       std::cerr << SOCKET_ERRNO << std::endl;
       return -1;
    }

    ret = setsockopt(m_SockFd, SOL_SOCKET, SO_SNDBUF, (char *)&so_sendbuf, sizeof(so_sendbuf));
    if (ret < 0) {
        std::cerr <<  "Failed to set socket option sendbuf size: " << so_sendbuf << std::endl;
        std::cerr << SOCKET_ERRNO << std::endl;
        closesocket(m_SockFd);
        return ret;
    }

    ret = setsockopt(m_SockFd, SOL_SOCKET, SO_RCVBUF, (char*)&so_recvbuf, sizeof(so_recvbuf));
    if (ret < 0) {
        std::cerr <<  "Failed to set socket option recvbuf size: " << so_recvbuf << std::endl;
        std::cerr << SOCKET_ERRNO << std::endl;
        closesocket(m_SockFd);
        return ret;
    }

    ret = setsockopt(m_SockFd, SOL_SOCKET, SO_REUSEADDR, (char*)&so_reuse, sizeof(so_reuse));
    if (ret < 0) {
        std::cerr <<  "Failed to set socket option reuseaddr\n";
        std::cerr << SOCKET_ERRNO << std::endl;
        return ret;
    }

#if WIN32
    // Milliseconds
    DWORD tv = 1000;
#else
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 000000;
#endif // WIN32

    if (setsockopt(m_SockFd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv)) == -1){
        std::cerr << "Couldn't set socket timeout\n" << std::endl;
        std::cerr << SOCKET_STR_ERR << std::endl;
        return -1;
    }

    return 0;
}
