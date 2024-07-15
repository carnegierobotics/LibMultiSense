/**
 * @file SecondaryAppTestUtility/SecondaryAppTestUtility.cc
 *
 * Copyright 2013-2023
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
 *   2023-09-19, patrick.smith@carnegierobotics.com, IRAD, Created file.
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

#include <bitset>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include <Utilities/portability/getopt/getopt.h>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG         = false;

volatile uint64_t callback_counter = 0;

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <ip_address>    : IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>           : default=7200" << std::endl;
    std::cerr << "\t-f <fps>           : default=1" << std::endl;

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

void imageCallback(const image::Header& header, void *userDataP) 
{
    (void) userDataP;

    cv::Mat luma = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.imageDataP)).clone();
    cv::imwrite("image.png", luma);
}

void secondaryAppCallback(const secondary_app::Header& header, void* userDataP)
{
    (void) userDataP;
    const char* u8_secondary_data_ptr = static_cast<const char*>(header.secondaryAppDataP);
    uint32_t length = header.length;
    std::string data(u8_secondary_data_ptr, length);
    std::cout << "Data from camera:" << std::endl;
    std::cout << data;
    std::cout << "***********************************" << std::endl;

    callback_counter++;
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    uint32_t    mtu            = 7200;

    image::Config cfg;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;
    float fps = 30.0f;

    while(-1 != (c = getopt(argc, argvPP, "a:m:f:"))) {
        switch(c) {
        case 'a':
            currentAddress = std::string(optarg);
            break;
        case 'm':
            mtu = atoi(optarg);
            break;
        case 'f':
            fps = static_cast<float>(atof(optarg));
            break;
        default:
            usage(*argvPP);
            break;
        }
    }

    std::cout << "Setting framerate: " << fps << " FPS" << std::endl;

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

    system::SecondaryAppConfig sa_cfg;
    status = channelP->getSecondaryAppConfig(sa_cfg);
    if (Status_Ok != status) {
        std::cerr << "Failed to get secondary app config: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Change FPS

    cfg.setFps(fps);
    status = channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Failed to configure FPS: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Add callbacks

    status = channelP->addIsolatedCallback(secondaryAppCallback);
    if (Status_Ok != status) {
        std::cerr << "Failed to add secondary app callback: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    status = channelP->addIsolatedCallback(imageCallback, Source_Luma_Rectified_Aux);
    if (Status_Ok != status) {
        std::cerr << "Failed to add image callback: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Start streaming

    uint64_t topics = Source_Secondary_App_Data | Source_Luma_Rectified_Aux;
    std::cout << "Subscription Flags: " << std::bitset<64>(topics) << std::endl;

    status = channelP->startStreams(topics);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    auto start = std::chrono::high_resolution_clock::now();
    while(!doneG) {
        // usleep(100000);
    }
    auto end = std::chrono::high_resolution_clock::now();

    //
    // Stop streaming

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
    }

    //
    // Report simple stats
    std::chrono::duration<float_t> inference_runtime = end - start;
    std::cout << "Iteration Count: " << callback_counter << " iter" << std::endl;
    std::cout << "Per-Iteration Inference Runtime: " << inference_runtime.count() / callback_counter << " s/iter" << std::endl;
    std::cout << "Total Inference Runtime: " << inference_runtime.count() << " s" << std::endl;
    std::cout << "FPS: " << callback_counter / inference_runtime.count() << std::endl;

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
