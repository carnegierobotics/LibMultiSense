/**
 * @file DepthImageUtility/DepthImageUtility.cc
 *
 * Copyright 2020-2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * https://www.carnegierobotics.com
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
 *   2023-10-12, malvarado@carnegierobotics.com, PR1044, Created file.
 **/

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

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
#include <limits>
#include <memory>
#include <sstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <getopt/getopt.h>
#include <ChannelUtilities.hh>
#include <Io.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : CURRENT MTU (default=1500)" << std::endl;
    std::cerr << "\t-c                      : Save color images and depth in the color image frame" << std::endl;

    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED (dwCtrlType);
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    (void) sig;
    doneG = true;
}
#endif

struct UserData
{
    Channel *driver = nullptr;
    std::shared_ptr<const ImageBufferWrapper> auxLumaRectified = nullptr;
    std::shared_ptr<const ImageBufferWrapper> auxChromaRectified = nullptr;
    crl::multisense::image::Calibration calibration;
    crl::multisense::system::DeviceInfo deviceInfo;
    bool hasAux = false;
};

std::vector<uint16_t> createDepthImage(const image::Header &disparity,
                                       const image::Calibration &calibration,
                                       const system::DeviceInfo &deviceInfo,
                                       bool auxFrame)
{
    const size_t width = disparity.width;
    const size_t height = disparity.height;

    //
    // Scale our calibration based on the current sensor resolution. Calibrations stored on the camera
    // are for full resolution images

    const double xScale = 1.0 / ((static_cast<double>(deviceInfo.imagerWidth) /
                                  static_cast<double>(width)));

    const double yScale = 1.0 / ((static_cast<double>(deviceInfo.imagerHeight) /
                                  static_cast<double>(height)));

    //
    // Decompose rectified projection matrices sorted in the form
    //
    // Left:         Right:
    // Fx 0  Cx 0    Fx 0  Cx  FxTx
    // 0  Fy Cy 0    0  Fy Cy  0
    // 0  0  0  1    0  0  0   1

    const double fx = calibration.left.P[0][0] * xScale;
    const double fy = calibration.left.P[1][1] * yScale;
    const double cx = calibration.left.P[0][2] * xScale;
    const double cy = calibration.left.P[1][2] * yScale;
    const double tx = calibration.right.P[0][3] / calibration.right.P[0][0];
    const double cxRight = calibration.right.P[0][2] * xScale;

    const double auxFx = calibration.aux.P[0][0] * xScale;
    const double auxFy = calibration.aux.P[1][1] * yScale;
    const double auxCx = calibration.aux.P[0][2] * xScale;
    const double auxCy = calibration.aux.P[1][2] * yScale;
    const double auxFxTx = calibration.aux.P[0][3] * xScale;
    const double auxFyTy = calibration.aux.P[1][3] * yScale;
    const double auxTz = calibration.aux.P[2][3];


    const uint16_t *disparityP = reinterpret_cast<const uint16_t*>(disparity.imageDataP);

    std::vector<uint16_t> pixels(height * width, 0);

    const double max_ni_depth = std::numeric_limits<uint16_t>::max();

    for (size_t h = 0 ; h < height ; ++h) {
        for (size_t w = 0 ; w < width ; ++w) {

            const size_t index = h * width + w;

            //
            // MultiSense 16 bit disparity images are stored in 1/16 of a pixel. This allows us to send subpixel
            // resolutions with integer values

            const double d = static_cast<double>(disparityP[index]) / 16.0;

            if (d == 0) {
                continue;
            }

            //
            // Q matrix used for reprojection. Manually preform the multiplication with the vector:
            // [w, h, d, 1]^T
            //
            //  FyTx    0     0    -FyCxTx
            //   0     FxTx   0    -FxCyTx
            //   0      0     0     FxFyTx
            //   0      0    -Fy    Fy(Cx - Cx')

            const double xB = ((fy * tx) * w) + (-fy * cx * tx);
            const double yB = ((fx * tx) * h) + (-fx * cy * tx);
            const double zB = (fx * fy * tx);
            const double invB = 1. / (-fy * d) + (fy * (cx - cxRight));

            uint16_t depth = 0;
            size_t depthIndex = 0;
            if (auxFrame) {

                //
                // Take our disparity, convert it into 3D point, and project the 3D  point into the rectified aux
                // image using the aux P matrix which has the form:
                //
                // Fx  0   Cx  FxTx
                // 0   Fy  Cy  FyTy
                // 0   0   1   Tz

                const double x = xB * invB;
                const double y = yB * invB;
                const double z = zB * invB;

                depth = static_cast<uint16_t>(std::min(max_ni_depth, std::max(0.0, (z + auxTz) * 1000)));

                const double u = ((auxFx * x) + (auxCx * z) + auxFxTx) / (z + auxTz);
                const double v = ((auxFy * y) + (auxCy * z) + auxFyTy) / (z + auxTz);

                depthIndex = static_cast<int>(v) * width + static_cast<int>(u);

                if (u < 0 || v < 0 || u >= width || v >= height || depthIndex >= (width * height)) {
                    continue;
                }

            }
            else {

                depth = static_cast<uint16_t>(std::min(max_ni_depth, std::max(0.0, (zB * invB) * 1000)));

                depthIndex = index;
            }

            pixels[depthIndex] = depth;
        }
    }

    return pixels;
}

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
                   void                *userDataP)
{
    UserData *userData = reinterpret_cast<UserData*>(userDataP);

    if (!userData->driver) {
        std::cerr << "Invalid MultiSense channel" << std::endl;
        return;
    }

    switch (header.source) {
        case Source_Luma_Rectified_Left:
        {
            io::savePgm(std::to_string(header.frameId) + "_left_rectified.pgm",
                        header.width,
                        header.height,
                        header.bitsPerPixel,
                        "",
                        header.imageDataP);
            return;
        }
        case Source_Disparity_Left:
        {
            if (header.bitsPerPixel != 16) {
                std::cerr << "Unsupported disparity pixel depth" << std::endl;
                return;
            }

            //
            // Create a depth image using a OpenNI Depth format (i.e. quantizing depth in mm to 16 bits)

            io::savePgm(std::to_string(header.frameId) + "depth.pgm",
                        header.width,
                        header.height,
                        16,
                        "",
                        createDepthImage(header, userData->calibration, userData->deviceInfo, userData->hasAux).data());

            return;
        }
        case Source_Chroma_Rectified_Aux:
        {
            userData->auxChromaRectified = std::make_shared<ImageBufferWrapper>(userData->driver, header);

            if (userData->auxLumaRectified && userData->auxLumaRectified->data().frameId == header.frameId) {
                saveColor(std::to_string(header.frameId) + "_rectified_color.ppm",
                          userData->auxLumaRectified,
                          userData->auxChromaRectified);
            }
            return;
        }
        case Source_Luma_Rectified_Aux:
        {
            userData->auxLumaRectified = std::make_shared<ImageBufferWrapper>(userData->driver, header);

            if (userData->auxChromaRectified && userData->auxChromaRectified->data().frameId == header.frameId) {
                saveColor(std::to_string(header.frameId) + "_rectified_color.ppm",
                          userData->auxLumaRectified,
                          userData->auxChromaRectified);
            }
            return;
        }
        default:
        {
            std::cerr << "Unknown image source: " << header.source << std::endl;
            return;
        }
    }
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t mtu = 0;
    bool useColor = false;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:m:c")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'm': mtu            = atoi(optarg);           break;
        case 'c': useColor       = true;                   break;
        default: usage(*argvPP);                           break;
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
    } else {
        cfg.setFps(10.0);
        cfg.setResolution(deviceInfo.imagerWidth / 2, deviceInfo.imagerHeight / 2);

        status = channelP->ptr()->setImageConfig(cfg);
        if (Status_Ok != status) {
            std::cerr << "Failed to configure sensor: " << Channel::statusString(status) << std::endl;
            return EXIT_FAILURE;
        }
    }

    //
    // If we have a aux camera, stream aux color data instead of left rectified images

    bool hasAux = false;
    DataSource streams = Source_Luma_Rectified_Left | Source_Disparity_Left;

    if (useColor &&
        (system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == deviceInfo.hardwareRevision)) {

        streams = Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux | Source_Disparity_Left;
        hasAux = true;
    }


    //
    // Setup user data to store camera state for pointcloud reprojection

    UserData userData{channelP->ptr(), nullptr, nullptr, calibration, deviceInfo, hasAux};

    //
    // Add callbacks

    channelP->ptr()->addIsolatedCallback(imageCallback,
                                         Source_Luma_Rectified_Left |
                                         Source_Disparity_Left |
                                         Source_Luma_Rectified_Aux |
                                         Source_Chroma_Rectified_Aux,
                                         &userData);


    //
    // Start streaming

    status = channelP->ptr()->startStreams(streams);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    while(!doneG) {
        usleep(1000000);
    }

    status = channelP->ptr()->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
