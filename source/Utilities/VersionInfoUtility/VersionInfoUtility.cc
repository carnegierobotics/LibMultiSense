/**
 * @file VersionInfoUtility/VersionInfoUtility.cc
 *
 * Copyright 2013-2022
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
 *   2021-04-02, dlr@carnegierobotics.com, PR1044, Created file from DeviceInfoUtility.cc
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <MultiSense/MultiSenseChannel.hh>

#include <Utilities/portability/getopt/getopt.h>

using namespace crl::multisense;

namespace {  // anonymous

void usage(const char *programNameP)
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>    : ip address (default=10.66.171.21)\n");

    exit(-1);
}

//
// Dump version info to a file.

void printVersionInfo(const system::VersionInfo& info,
                      FILE*                      fP=stdout)
{
    fprintf(fP, "API build date: %s\n", info.apiBuildDate.c_str());
    fprintf(fP, "API version: %#06x\n", info.apiVersion);
    fprintf(fP, "\n");
    fprintf(fP, "Firmware build date: %s\n", info.sensorFirmwareBuildDate.c_str());
    fprintf(fP, "Firmware version: %#06x\n", info.sensorFirmwareVersion);
    fprintf(fP, "Hardware version: %#" PRIx64 "\n", info.sensorHardwareVersion);
    fprintf(fP, "Hardware magic: %#" PRIx64 "\n", info.sensorHardwareMagic);
    fprintf(fP, "FPGA DNA: %#" PRIx64 "\n", info.sensorFpgaDna);
    fprintf(fP, "\n");
}

} // namespace

int main(int    argc,
         char **argvPP)
{
    std::string ipAddress  = "10.66.171.21";

    //
    // Parse arguments

    int cc;

    while(-1 != (cc = getopt(argc, argvPP, "a:k:s:qy")))
        switch(cc) {
        case 'a': ipAddress = std::string(optarg);    break;
        default: usage(*argvPP);                      break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		ipAddress.c_str());
	return -1;
    }

    //
    // Query version

    Status      status;
    system::VersionInfo versionInfo;

    status = channelP->getVersionInfo(versionInfo);
    if (Status_Ok != status) {
        fprintf(stderr, "Failed to query detailed version info: %s\n",
                Channel::statusString(status));
        goto clean_out;
    } else {
        printVersionInfo(versionInfo);
        fflush(stdout);
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
