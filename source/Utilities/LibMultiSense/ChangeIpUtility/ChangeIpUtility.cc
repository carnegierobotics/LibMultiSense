/**
 * @file ChangeIpUtility.cc
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
 *   2025-04-04, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#endif

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string.h>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "getopt/getopt.h"

namespace lms = multisense;

namespace
{

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " -e <extrinsics-file> -i <intrinsics-file> [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current-address> : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-A <new-address>     : NEW IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-G <new-gateway>     : NEW IPV4 gateway (default=10.66.171.1)" << std::endl;
    std::cerr << "\t-N <new-netmask>     : NEW IPV4 netmask (default=255.255.255.0)" << std::endl;
    std::cerr << "\t-b <interface>       : send broadcast packet to a specified network interface."
                                           "This resets the IP address to the new configured IP" << std::endl;
    std::cerr << "\t-y                   : Disable confirmation prompt (default=false)" << std::endl;
    exit(1);
}

}

int main(int argc, char** argv)
{
    std::string ip_address = "10.66.171.21";
    std::string new_ip_address = "10.66.171.21";
    std::string new_gateway = "10.66.171.1";
    std::string new_netmask = "255.255.255.0";
    std::optional<std::string> interface = std::nullopt;
    bool disable_confirmation = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:A:G:N:b:y")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'A': new_ip_address = std::string(optarg); break;
            case 'G': new_gateway = std::string(optarg); break;
            case 'N': new_netmask = std::string(optarg); break;
            case 'b': interface = std::string(optarg); break;
            case 'y': disable_confirmation = true; break;
            default: usage(*argv); break;
        }
    }

    lms::Channel::Config config{ip_address};
    config.connect_on_initialization = !static_cast<bool>(interface);
    const auto channel = lms::Channel::create(config);

    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    if (!disable_confirmation)
    {
        std::cout << "NEW address: " << new_ip_address << std::endl;;
        std::cout << "NEW gateway: " << new_gateway << std::endl;;
        std::cout << "NEW netmask: " << new_netmask << std::endl;;

        if(interface)
        {
            std::cout << "** WARNING: All MultiSense devices attached to interface " << interface.value() <<
                         "will have their addresses changed **" << std::endl;
        }

        std::cerr << "Really update network configuration? (y/n):" << std::endl;

        int reply = getchar();
        if ('Y' != reply && 'y' != reply)
        {
            std::cout << "Aborting" << std::endl;
            return 1;
        }
    }

    const lms::MultiSenseInfo::NetworkInfo new_info{new_ip_address, new_gateway, new_netmask};

    if (const auto status = channel->set_network_config(new_info, interface); status != lms::Status::OK)
    {
        std::cerr << "Unable to set new IP address: " << lms::to_string(status) << std::endl;
        return 1;
    }

    return 0;
}
