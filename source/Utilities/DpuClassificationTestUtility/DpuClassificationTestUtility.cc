/**
 * @file DpuClassificationTestUtility/DpuClassificationTestUtility.cc
 *
 * Copyright 2022
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
 *   2022-06-30, bblakeslee@carnegierobotics.com, 2033.1, Created file.
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

namespace {     // Anonymous

volatile bool quit_flag = false;

void usage(const char* programNamePtr){
    std::cerr << "USAGE: " << programNamePtr << " [<options>]" << std::endl;
    std::cerr << "Available <options>:" << std::endl;
    std::cerr << "\t-a <ip_address> : IPV4 address of camera (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>        : Max packet size in bytes (default=7200)" << std::endl;

    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED(dwCtrlType);
    std::cerr << "Shutting down on signal: CTRL-C" << std::endl;
    quit_flag = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    std::cerr << "Shutting down on signal: " << strsignal(sig) << std::endl;
    quit_flag = true;
}
#endif

void dpuClassificationCallback(const dpu_classification::Header& header, void* userDataPtr)
{
    (void) userDataPtr;

    std::cout << "******************" << std::endl;
    std::cout << "DPU Classification Output:" << std::endl;
    std::cout << "Frame ID: " << header.frameId << std::endl;
    std::cout << "Time Stamp: " << header.timestamp << std::endl;
    std::cout << "Success: " << uint16_t(header.success) << std::endl;
    std::cout << "Classification ID: " << header.classId << std::endl;
}

void destroyChannel(Channel* channelPtr){
    Channel::Destroy(channelPtr);
    exit(0);
}

}   // Anonynous

int main(int argc, char** argv){
    std::string currentAddress = "10.66.171.21";
    uint32_t mtu = 1500;
    bool dpuSupported = false;
    image::Config cfg;

    crl::multisense::CameraProfile profile = crl::multisense::User_Control;
    std::vector<system::DeviceMode> deviceModes;
    image::Config config;

#if WIN32
    SetConsoleCtrlHandler(signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    // Process command line arguments
    int arg;
    while(-1 != (arg = getopt(argc, argv, "a:m"))){
        switch(arg){
            case 'a':
                currentAddress = std::string(optarg);
                break;
            case 'm':
                mtu = atoi(optarg);
                break;
            default:
                usage(*argv);
        }
    }

    // Camera communication initialization
    Channel *channelPtr = Channel::Create(currentAddress);
    if (NULL == channelPtr){
        std::cerr << "Failed to initialize communications with camera at " << currentAddress << std::endl;
        std::cerr << "Please check that IP address is valid." << std::endl;
        return -1;
    }

    // Fetch firmware version
    system::VersionInfo v;
    Status status = channelPtr->getVersionInfo(v);
    if(Status_Ok != status){
        std::cerr << "Failed to obtain sensor version: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Check to see if firmware supports DPU classification
    status = channelPtr->getDeviceModes(deviceModes);
    if (Status_Ok != status) {
        std::cerr << "Failed to get device modes: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    dpuSupported = std::any_of(deviceModes.begin(), deviceModes.end(), [](const auto &mode) {
        return mode.supportedDataSources & Source_DpuClassification_Detections;});

    if (!dpuSupported) {
        std::cerr << "DPU classification not supported with this firmware" << std::endl;
        destroyChannel(channelPtr);
    }
    
    // Shut down all streams
    // FIXME:  Stopping all streams results in the following error:
    //         00026.291 s19: Selected stream to remove does not exist. Port 16017
    //         The port number appears variable.
    status = channelPtr->stopStreams(Source_All);
    if (Status_Ok != status){
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Change MTU size
    // NOTE: This will fail if the MTU is set to over 1500 when using an
    // external USB-C to ethernet adapter.
    status = channelPtr->setMtu(mtu);
    if (Status_Ok != status) {
        std::cerr << "Failed to set MTU to " << mtu << ": " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Enable DPU classification profile
    status = channelPtr->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: Failed to query image config: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }
   
    profile |= crl::multisense::DpuClassification;

    cfg.setCameraProfile(profile);
    // TODO: This is a hack.  Fix this for real.
    cfg.setGain(2.0);

    // FIXME:  This setImageConfig call results in a crash, hitting the error handler.
    //         This corresponds the following error in the S19 binary:
    //         00243.068 s19: Error: Invalid gain setting!
    //         This appears to be caused by the default gain of 1 being outside the 
    //         values of 1.68421 to 16.  Need to find how to set this properly.
    status = channelPtr->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: failed to set image config: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Configure callbacks
    status = channelPtr->addIsolatedCallback(dpuClassificationCallback);
    if (Status_Ok != status) {
        std::cerr << "Failed to add DPU callback." << std::endl;
    }

    // Start streaming
    status = channelPtr->startStreams(Source_DpuClassification_Detections);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    while(!quit_flag){
        usleep(100000);
    }

    // Stop stream
    status = channelPtr->stopStreams(Source_All);
    if (Status_Ok != status){
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }
}

