/**
 * @file ChangeIpUtility/ChangeIpUtility.cc
 *
 * Copyright 2013
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
 *   2013-05-22, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <errno.h>
#include <string.h>

#include <LibMultiSense/details/utility/Portability.hh>
#include <LibMultiSense/MultiSenseChannel.hh>

#include <LibMultiSense/details/utility/BufferStream.hh>
#include <LibMultiSense/details/wire/Protocol.h>
#include <LibMultiSense/details/wire/SysNetworkMessage.h>

#include <Utilities/portability/getopt/getopt.h>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-A <new_address>        : NEW IPV4 address     (default=10.66.171.21)\n");
    fprintf(stderr, "\t-G <new_gateway>        : NEW IPV4 gateway     (default=10.66.171.1)\n");
    fprintf(stderr, "\t-N <new_netmask>        : NEW IPV4 address     (default=255.255.240.0)\n");
#ifndef WIN32
    fprintf(stderr, "\t-b <interface>          : send broadcast packet to specified network interface (requires root)\n");
#endif
    fprintf(stderr, "\t-y                      : disable confirmation prompt\n");
    
    exit(-1);
}

bool decodeIpv4(const std::string& addr,
                unsigned int&      p1,
                unsigned int&      p2,
                unsigned int&      p3,
                unsigned int&      p4)
{
    if (addr.empty() ||
        4 != sscanf(addr.c_str(), "%3u.%3u.%3u.%3u",
                    &p1, &p2, &p3, &p4) ||
        (p1 > 255 || p2 > 255 || p3 > 255 || p4 > 255)) {
        fprintf(stderr, "Unable to decode \"%s\" as IPV4 dotted-quad \n",
                        addr.c_str());
        fflush(stderr);
        return false;
    }
    return true;
}

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    std::string desiredAddress = "10.66.171.21";
    std::string desiredGateway = "10.66.171.1";
    std::string desiredNetmask = "255.255.240.0";
    std::string iface = "eth0";
    bool        blind=false;
    bool        prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:A:G:N:b:y")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'A': desiredAddress = std::string(optarg);    break;
        case 'G': desiredGateway = std::string(optarg);    break;
        case 'N': desiredNetmask = std::string(optarg);    break;
        case 'b': blind = true; iface = std::string(optarg); break;
        case 'y': prompt         = false;                  break;
        default: usage(*argvPP);                           break;
        }

    Status status;
    Channel *channelP = NULL;
    int sockfd = -1;

    system::DeviceInfo deviceInfo;

    uint32_t a1 = 0;
    uint32_t a2 = 0;
    uint32_t a3 = 0;
    uint32_t a4 = 0;


    if(!blind)
    {
        //
        // Initialize communications.

        channelP = Channel::Create(currentAddress);
        if (NULL == channelP) {
        fprintf(stderr, "Failed to establish communications with \"%s\"\n",
            currentAddress.c_str());
        return -1;
        }

        //
        // Query version

        VersionType version;

        status = channelP->getSensorVersion(version);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to query sensor version: %s\n",
                    Channel::statusString(status));
            goto clean_out;
        }
    }
    else
    {
#ifdef WIN32
        perror ("broadcast is not yet supported on Windows");
        goto clean_out;
#else
        int broadcast=1;

        // create UDP socket
        sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(sockfd == -1) {
            perror("socket() failed");
            goto clean_out;
        }

        // enable support for sending to broadcast address
        if (setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,reinterpret_cast<char const*>(&broadcast),sizeof(broadcast))==-1) {
            perror("setsockopt(...SO_BROADCAST) failed");
            goto clean_out;
        }
        
        // bind to a specific interface so broadcast packet goes to the right place
        // note: on most systems, this will require elevated privileges
        if (setsockopt(sockfd,SOL_SOCKET,SO_BINDTODEVICE,iface.c_str(),iface.size()+1)==-1) {
            perror("setsockopt(...SO_BINDTODEVICE) failed (are you root?)");
            goto clean_out;
        }
#endif
    }

    status = channelP->getDeviceInfo(deviceInfo);

    if (Status_Ok != status) {
        fprintf(stderr, "failed to qyery device info %s\n",
                Channel::statusString(status));
        goto clean_out;
    }


    if (!decodeIpv4(desiredAddress, a1, a2, a3, a4))
    {
        goto clean_out;
    }

    // MultiSense SL units use the 192.168.0 subnet to talk to the Hokuyo
    // laser. Setting the ip address of the MultiSense to 192.168.0.X will
    // interfere with the networking configuration
    if (deviceInfo.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL &&
        192 == a1  && 168 == a2 && 0 == a3)
    {
        fprintf(stderr, "Invalid IP address on the 192.168.0 subnet. Aborting\n");
        fflush(stderr);
        goto clean_out;
    }

    if(prompt)
    {

        fprintf(stdout, "NEW address: %s\n", desiredAddress.c_str());
        fprintf(stdout, "NEW gateway: %s\n", desiredGateway.c_str());
        fprintf(stdout, "NEW netmask: %s\n\n", desiredNetmask.c_str());

        if(blind) {
            fprintf(stdout, "** WARNING: All MultiSense devices attached to interface '%s' will have their addresses changed **\n\n", iface.c_str());
        }

        fprintf(stdout, "Really update network configuration? (y/n): ");
        fflush(stdout);

        int c = getchar();
        if ('Y' != c && 'y' != c) {
            fprintf(stdout, "Aborting\n");
            goto clean_out;
        }
    }

    if(!blind)
    {
        //
        // Try setting the new IP parameters. The device will
        // verify that the IP addresses are valid dotted-quads,
        // however, little complex verification is done.

        status = channelP->setNetworkConfig(system::NetworkConfig(desiredAddress,
                                                                  desiredGateway,
                                                                  desiredNetmask));
        if (Status_Ok != status)
            fprintf(stderr, "Failed to set the network configuration: %s\n",
                    Channel::statusString(status));
        else 
            fprintf(stdout, "Network parameters changed successfully\n");
    }
    else
    {
#ifndef WIN32
        struct sockaddr_in si;

        // construct destination address
        memset(&si,0,sizeof(si));
        si.sin_family = AF_INET;
        si.sin_port = htons(9001);
        si.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        // manually construct SysNetwork message
        using namespace crl::multisense::details;
        utility::BufferStreamWriter buffer(256);
        wire::Header & header = *(reinterpret_cast<wire::Header*>(buffer.data()));

        // hide header area
        buffer.seek(sizeof(wire::Header));

        // set ID and version
        const wire::IdType      id      = wire::SysNetwork::ID;
        const wire::VersionType version = wire::SysNetwork::VERSION;
        buffer & id;
        buffer & version;

        // construct message
        wire::SysNetwork msg(desiredAddress,desiredGateway,desiredNetmask);
        msg.serialize(buffer,version);

        // install header
        header.magic              = wire::HEADER_MAGIC;
        header.version            = wire::HEADER_VERSION;
        header.group              = wire::HEADER_GROUP;
        header.flags              = 0;
        header.sequenceIdentifier = 0;
        header.messageLength      = buffer.tell() - sizeof(wire::Header);
        header.byteOffset         = 0;

        // send packet
        if(sendto(sockfd,reinterpret_cast<char const*>(buffer.data()),buffer.tell(),0,reinterpret_cast<sockaddr*>(&si),sizeof(si)) == -1) {
            perror("sendto() failed");
            goto clean_out;
        }

        fprintf(stdout, "Successfully transmitted network parameter change command\n");
#endif
    }

clean_out:

    if(channelP)
        Channel::Destroy(channelP);
    if(sockfd != -1)
#ifdef WIN32
        closesocket(sockfd);
#else
        close(sockfd);
#endif

    return 0;
}
