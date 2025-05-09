/**
 * @file FirmwareUpdateUtility/Ip.hh
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

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#define INET_ADDRSTRLEN 16
#include <WinSock2.h>
#include <Windows.h>
typedef SOCKET socket_t;

#else
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sendfile.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <ifaddrs.h>
#include <netinet/udp.h>
#include <sys/time.h>
typedef int socket_t;
#endif



 #include <stdio.h>
 #include <stdlib.h>
 #include <errno.h>
 #include <fcntl.h>
 #include <string.h>
 #include <sys/types.h>
 #include <signal.h>
 #include <string>

#include <MultiSense/details/utility/Portability.hh>

class Ip
{

public:
    Ip()
    {

    }
    int Bind();
    int Setup(std::string _IpAddress);
    ~Ip()
    {
        if (m_SockFd > 0)
        {
            closesocket(m_SockFd);
        }

        m_Connected = 0;
    }

    socket_t Sock( )
    {
        return m_SockFd;
    }

    struct sockaddr_in Server()
    {
        return m_ServerAddress;
    }

private:

    socket_t m_SockFd;
    struct sockaddr_in m_ClientAddress;
    struct sockaddr_in m_ServerAddress;
    std::string m_IpAddress;
    int m_Connected;
};
