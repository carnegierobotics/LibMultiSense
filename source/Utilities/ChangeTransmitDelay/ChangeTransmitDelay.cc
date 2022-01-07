/**
 * @file ChangeTransmitDelay/ChangeTransmitDelay.cc
 *
 * Copyright 2017-2022
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
#include <fstream>
#include <iostream>
#include <iomanip>

#include <errno.h>
#include <string.h>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include <MultiSense/details/utility/BufferStream.hh>
#include <MultiSense/details/wire/Protocol.hh>
#include <MultiSense/details/wire/SysNetworkMessage.hh>

#include <Utilities/portability/getopt/getopt.h>

namespace {  // anonymous

void usage(const char *programNameP)
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-d transmit delay in ms\n");

    exit(-1);
}

} // anonymous

using namespace crl::multisense;

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int delay = 0;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:d:")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'd': delay         = atoi(optarg);            break;
        default: usage(*argvPP);                           break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(currentAddress);
    if (NULL == channelP) {
		std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return -1;
    }

    //
    // Query version

    Status status;
    system::VersionInfo v;

    status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
		std::cerr << "Failed to query sensor version: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

	std::cout << "API build date      :  " << v.apiBuildDate << "\n";
        std::cout << "API version         :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.apiVersion << "\n";
	std::cout << "Firmware build date :  " << v.sensorFirmwareBuildDate << "\n";
	std::cout << "Firmware version    :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.sensorFirmwareVersion << "\n";
	std::cout << "Hardware version    :  0x" << std::hex << v.sensorHardwareVersion << "\n";
	std::cout << "Hardware magic      :  0x" << std::hex << v.sensorHardwareMagic << "\n";
	std::cout << "FPGA DNA            :  0x" << std::hex << v.sensorFpgaDna << "\n";
	std::cout << std::dec;

    //
    // Change framerate

    {
        image::TransmitDelay TransmitDelay;

    status = channelP->getTransmitDelay(TransmitDelay);
        if (Status_Ok != status) {
			std::cerr << "Failed to get transmit delay: " << Channel::statusString(status) << std::endl;
            goto clean_out;
        } else {

            TransmitDelay.delay = delay;

            status = channelP->setTransmitDelay(TransmitDelay);
            if (Status_Ok != status) {
				std::cerr << "Failed to configure transmit delay: " << Channel::statusString(status) << std::endl;
                goto clean_out;
            }
        }
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
