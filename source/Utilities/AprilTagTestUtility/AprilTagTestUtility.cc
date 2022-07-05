/**
 * @file AprilTagTestUtility/AprilTagTestUtility.cc
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
 *   2013-11-12, ekratzer@carnegierobotics.com, PR1044, Created file.
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
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include <Utilities/portability/getopt/getopt.h>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG         = false;

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <ip_address>    : IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>           : default=7200" << std::endl;
    std::cerr << "\t-f <log_file>      : FILE to log IMU data (stdout by default)" << std::endl;

    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED (dwCtrlType);
    std::cerr << "Shutting down on signal: CTRL-C" << std::endl;
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    std::cerr << "Shutting down on signal: " << strsignal(sig) << std::endl;
    doneG = true;
}
#endif

void apriltagCallback(const apriltag::Header& header, void* userDataP)
{
    (void) userDataP;

    std::cout << "----------------------------" << std::endl;
    std::cout << "frameId: " << header.frameId << std::endl;
    std::cout << "timestamp: " << header.timestamp << std::endl;
    std::cout << "imageSource: " << header.imageSource << std::endl;
    std::cout << "success: " << (header.success ? "true" : "false") << std::endl;
    std::cout << "numDetections: " << header.numDetections << std::endl;

    for (auto &d : header.detections)
    {
        std::cout << "tag ID: " << d.id << ", family ID: " << d.family << std::endl;
        std::cout << "\thamming: " << (int)d.hamming << std::endl;
        std::cout << "\tdecisionMargin: " << d.decisionMargin << std::endl;

        std::cout << "\ttagToImageHomography: " << std::endl;
        for (unsigned int col = 0; col < 3; col++)
        {
            std::cout << "\t\t";
            for (unsigned int row = 0; row < 3; row++)
            {
                std::cout << d.tagToImageHomography[row][col] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "\tcenter: " << std::endl;
        for (unsigned int i = 0; i < 2; i++)
            std::cout << "\t\t" << d.center[i] << std::endl;

        std::cout << "\tcorners: " << std::endl;
        for (unsigned int i = 0; i < 4; i++)
            std::cout << "\t\t" << d.corners[i][0] << " " << d.corners[i][1] << std::endl;
    }
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    uint32_t    mtu            = 7200;

    crl::multisense::CameraProfile  profile = crl::multisense::User_Control;
    crl::multisense::system::ApriltagParams params;
    bool apriltag_supported = false;
    std::vector<system::DeviceMode> deviceModes;
    image::Config cfg;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:m:v")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'm': mtu            = atoi(optarg);           break;
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
    // Query firmware version

    system::VersionInfo v;

    Status status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
        std::cerr << "Failed to query sensor version: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Make sure firmware supports AprilTag detections

    status = channelP->getDeviceModes(deviceModes);
    if (Status_Ok != status) {
        std::cerr << "Failed to query device modes: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    apriltag_supported =
        std::any_of(deviceModes.begin(), deviceModes.end(), [](const auto &mode) {
            return mode.supportedDataSources & Source_AprilTag_Detections; });

    if (!apriltag_supported) {
        std::cerr << "AprilTag detector not supported with this firmware" << std::endl;
        goto clean_out;
    }

    //
    // Turn off all streams by default

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Change MTU

    status = channelP->setMtu(mtu);
    if (Status_Ok != status) {
        std::cerr << "Failed to set MTU to " << mtu << ": " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Enable apriltag profile

    status = channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: failed to query image config: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    profile |= crl::multisense::AprilTag;
    cfg.setCameraProfile(profile);

    status = channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: failed to set image config: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Send default parameters

    status = channelP->setApriltagParams(params);
    if (Status_Ok != status) {
        std::cerr << "Failed to set apriltag params: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Add callbacks

    channelP->addIsolatedCallback(apriltagCallback);

    //
    // Start streaming

    status = channelP->startStreams(Source_AprilTag_Detections);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    while(!doneG)
        usleep(100000);

    //
    // Stop streaming

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
    }

    //
    // Report simple stats


clean_out:

    Channel::Destroy(channelP);
    return 0;
}
