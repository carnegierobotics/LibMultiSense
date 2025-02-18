/**
 * @file ColorImageUtility/ColorImageUtility.cc
 *
 * Copyright 2020-2025
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
 *   2020-04-03, sivanov@carnegierobotics.com
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h> // htons
#endif

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <cstdint>
#include <array>

#include <Utilities/portability/getopt/getopt.h>
#include <ChannelUtilities.hh>
#include <Io.hh>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

void usage(const char* programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : CURRENT MTU (default=1500)" << std::endl;
    std::cerr << "\t-s <color_source>       : LEFT,RIGHT,AUX (default=aux)" << std::endl;

    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED(dwCtrlType);
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    (void)sig;
    doneG = true;
}
#endif

struct UserData
{
    Channel* driver = nullptr;
    std::shared_ptr<const ImageBufferWrapper> chroma = nullptr;
    std::shared_ptr<const ImageBufferWrapper> luma = nullptr;
    crl::multisense::image::Calibration calibration;
    crl::multisense::system::DeviceInfo deviceInfo;
    std::pair<DataSource, DataSource> colorSource;
};

bool saveColor(const std::string& fileName,
               std::shared_ptr<const ImageBufferWrapper> leftRect,
               std::shared_ptr<const ImageBufferWrapper> leftChromaRect)
{
    std::vector<uint8_t> output(leftRect->data().width * leftRect->data().height * 3);
    ycbcrToBgr(leftRect->data(), leftChromaRect->data(), output.data());

    io::savePpm(fileName, leftRect->data().width, leftRect->data().height, output.data());
    return true;
}

void imageCallback(const image::Header& header,
        void* userDataP)
{
    UserData* userData = reinterpret_cast<UserData*>(userDataP);


    if (!userData->driver) {
        std::cerr << "Invalid MultiSense channel" << std::endl;
        return;
    }

    if (header.source == userData->colorSource.first)
    {
        userData->chroma = std::make_shared<ImageBufferWrapper>(userData->driver, header);
        if (userData->luma && userData->luma->data().frameId == header.frameId)
        {
            // matching frameID's, pass through to create image
        }
        else
        {
            return;
        }
    }
    if (header.source == userData->colorSource.second)
    {
        userData->luma = std::make_shared<ImageBufferWrapper>(userData->driver, header);
        if (userData->chroma && userData->chroma->data().frameId == header.frameId)
        {
            // matching frameID's, pass through to create image
        }
        else
        {
            return;
        }
    }

    if (userData->luma != nullptr && userData->chroma != nullptr)
    {
        saveColor(std::to_string(header.frameId) + ".ppm", userData->luma,
                userData->chroma);
    }
}

std::pair<DataSource, DataSource> colorSourceFromArg(const std::string &srcStr)
{
    if (srcStr == "aux")
    {
        return { Source_Chroma_Rectified_Aux, Source_Luma_Rectified_Aux };
    }
    else if (srcStr == "left")
    {
        return { Source_Chroma_Left, Source_Luma_Left };
    }
    else if (srcStr == "right")
    {
        return { Source_Chroma_Right, Source_Luma_Right };
    }
    else
    {
        throw std::runtime_error("Invalid color source given");
    }
}

} // anonymous

int main(int    argc,
        char** argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t mtu = 0;
    std::pair<DataSource, DataSource> userSource{ Source_Chroma_Rectified_Aux, Source_Luma_Rectified_Aux };

#if WIN32
    SetConsoleCtrlHandler(signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while (-1 != (c = getopt(argc, argvPP, "a:m:s:")))
        switch (c) {
            case 'a': currentAddress = std::string(optarg);        break;
            case 'm': mtu = atoi(optarg);               break;
            case 's': userSource = colorSourceFromArg(optarg); break;
            default: usage(*argvPP);                               break;
        }

    Status status;

    //
    // Initialize communications.

    auto channelP = std::make_unique<ChannelWrapper>(currentAddress);
    if (nullptr == channelP || nullptr == channelP->ptr()) {
        std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Change MTU
    if (mtu >= 1500)
        status = channelP->ptr()->setMtu(mtu);
    else
        status = channelP->ptr()->setBestMtu();
    if (Status_Ok != status) {
        std::cerr << "Failed to set MTU: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Query calibration

    image::Calibration calibration;
    status = channelP->ptr()->getImageCalibration(calibration);
    if (Status_Ok != status) {
        std::cerr << "Failed to query calibraiton: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Query device info

    crl::multisense::system::DeviceInfo deviceInfo;
    status = channelP->ptr()->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        std::cerr << "Failed to query device info: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Change framerate and resolution to 1/4 resolution

    image::Config cfg;

    status = channelP->ptr()->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Failed to get image config: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }
    else {
        cfg.setFps(10.0);
        cfg.setAutoWhiteBalance(true);
        cfg.setAutoExposure(true);
        cfg.setResolution(deviceInfo.imagerWidth / 2, deviceInfo.imagerHeight / 2);

        status = channelP->ptr()->setImageConfig(cfg);
        if (Status_Ok != status) {
            std::cerr << "Failed to configure sensor: " << Channel::statusString(status) << std::endl;
            return EXIT_FAILURE;
        }
    }

    //
    // Setup user data to store camera state for pointcloud reprojection

    UserData userData{ channelP->ptr(), nullptr, nullptr, calibration, deviceInfo, userSource };

    //
    // Add callbacks

    channelP->ptr()->addIsolatedCallback(imageCallback, userSource.first | userSource.second, &userData);

    //
    // Start streaming

    status = channelP->ptr()->startStreams(userSource.first | userSource.second);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    while (!doneG) {
        usleep(1000000);
    }

    status = channelP->ptr()->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
