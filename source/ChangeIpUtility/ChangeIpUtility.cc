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

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <getopt.h>

#include <LibMultiSense/MultiSenseChannel.hh>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-A <new_address>        : NEW IPV4 address     (default=10.66.171.21)\n");
    fprintf(stderr, "\t-G <new_gateway>        : NEW IPV4 gateway     (default=10.66.171.1)\n");
    fprintf(stderr, "\t-N <new_netmask>        : NEW IPV4 address     (default=255.255.240.0)\n");
    fprintf(stderr, "\t-y                      : disable confirmation prompt\n");
    
    exit(-1);
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
    bool        prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:A:G:N:y")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'A': desiredAddress = std::string(optarg);    break;
        case 'G': desiredGateway = std::string(optarg);    break;
        case 'N': desiredNetmask = std::string(optarg);    break;
        case 'y': prompt         = false;                  break;
        default: usage(*argvPP);                           break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(currentAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		currentAddress.c_str());
	return -1;
    }

    //
    // Query version

    Status status;
    VersionType version;

    status = channelP->getSensorVersion(version);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %s\n",
                Channel::statusString(status));
        goto clean_out;
    }
    
    if (prompt) {

        fprintf(stdout, "NEW address: %s\n", desiredAddress.c_str());
        fprintf(stdout, "NEW gateway: %s\n", desiredGateway.c_str());
        fprintf(stdout, "NEW netmask: %s\n\n", desiredNetmask.c_str());
        fprintf(stdout, "Really update network configuration? (y/n): ");
        fflush(stdout);

        int c = getchar();
        if ('Y' != c && 'y' != c) {
            fprintf(stdout, "Aborting\n");
            goto clean_out;
        }
    }

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

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
