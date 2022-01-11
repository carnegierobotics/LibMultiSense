/**
 * @file PointCloudUtility/PointCloudUtility.cc
 *
 * Copyright 2020-2022
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
 *   2020-09-28, malvarado@carnegierobotics.com, PR1044, Created file.
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

#include <Utilities/portability/getopt/getopt.h>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : CURRENT MTU (default=7200)" << std::endl;
    std::cerr << "\t-d <min_disparity>      : CURRENT MINIMUM DISPARITY (default=5.0)" << std::endl;

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

struct WorldPoint
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    uint8_t luma = 0u;
};

//
// Wrapper around a Channel pointer to make cleanup easier

class ChannelWrapper
{
    public:

        ChannelWrapper(const std::string &ipAddress):
            channelPtr_(Channel::Create(ipAddress))
        {
        }

        ~ChannelWrapper()
        {
            if (channelPtr_) {
                Channel::Destroy(channelPtr_);
            }
        }

        Channel* ptr() noexcept
        {
            return channelPtr_;
        }

    private:

        ChannelWrapper(const ChannelWrapper&) = delete;
        ChannelWrapper operator=(const ChannelWrapper&) = delete;

        Channel* channelPtr_ = nullptr;
};

//
// Wrapper to preserve image data outside of the image callback

class ImageBufferWrapper
{
public:
    ImageBufferWrapper(crl::multisense::Channel* driver,
                 const crl::multisense::image::Header &data):
        driver_(driver),
        callbackBuffer_(driver->reserveCallbackBuffer()),
        data_(data)
    {
    }

    ~ImageBufferWrapper()
    {
        if (driver_) {
            driver_->releaseCallbackBuffer(callbackBuffer_);
        }
    }

    const image::Header &data() const noexcept
    {
        return data_;
    }

private:

    ImageBufferWrapper(const ImageBufferWrapper&) = delete;
    ImageBufferWrapper operator=(const ImageBufferWrapper&) = delete;

    crl::multisense::Channel * driver_ = nullptr;
    void* callbackBuffer_;
    const image::Header data_;

};

struct UserData
{
    Channel *driver = nullptr;
    std::shared_ptr<const ImageBufferWrapper> disparity = nullptr;
    std::shared_ptr<const ImageBufferWrapper> leftRectified = nullptr;
    crl::multisense::image::Calibration calibration;
    crl::multisense::system::DeviceInfo deviceInfo;
    double minDisparity = 0.0;
};

std::vector<WorldPoint> reprojectDisparity(const std::shared_ptr<const ImageBufferWrapper> disparity,
                                           const std::shared_ptr<const ImageBufferWrapper> leftRectified,
                                           const image::Calibration &calibration,
                                           const system::DeviceInfo &deviceInfo,
                                           double minDisparity)
{
    const size_t width = leftRectified->data().width;
    const size_t height = leftRectified->data().height;

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

    const uint16_t *disparityP = reinterpret_cast<const uint16_t*>(disparity->data().imageDataP);
    const uint8_t *leftRectifiedP = reinterpret_cast<const uint8_t*>(leftRectified->data().imageDataP);

    std::vector<WorldPoint> points;
    points.reserve(height * width);

    for (size_t h = 0 ; h < height ; ++h) {
        for (size_t w = 0 ; w < width ; ++w) {

            const size_t index = h * width + w;

            //
            // MultiSense 16 bit disparity images are stored in 1/16 of a pixel. This allows us to send subpixel
            // resolutions with integer values

            const double d = static_cast<double>(disparityP[index]) / 16.0;

            if (d < minDisparity) {
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

            points.emplace_back(WorldPoint{static_cast<float>(xB * invB),
                                           static_cast<float>(yB * invB),
                                           static_cast<float>(zB * invB),
                                           leftRectifiedP[index]});
        }
    }

    return points;
}

bool savePly(const std::string& fileName,
             const std::vector<WorldPoint> &points)
{
    std::ofstream ply(fileName.c_str());

    ply << "ply\n";
    ply << "format ascii 1.0\n";
    ply << "element vertex " << points.size() << "\n";
    ply << "property float x\n";
    ply << "property float y\n";
    ply << "property float z\n";
    ply << "property uchar red\n";
    ply << "property uchar green\n";
    ply << "property uchar blue\n";
    ply << "end_header\n";

    for (const auto &point : points) {
        const uint32_t luma = static_cast<uint32_t>(point.luma);
        ply << point.x << " " << point.y << " " << point.z << " " << luma << " " << luma << " " << luma << "\n";
    }

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
            userData->leftRectified = std::make_shared<ImageBufferWrapper>(userData->driver, header);
            if (userData->disparity && userData->disparity->data().frameId == header.frameId) {
                //
                // Our disparity and left recitified images match, we have enough info to create a pointcloud
                break;
            }

            return;
        }
        case Source_Disparity_Left:
        {
            if (header.bitsPerPixel != 16) {
                std::cerr << "Unsupported disparity pixel depth" << std::endl;
                return;
            }


            userData->disparity = std::make_shared<ImageBufferWrapper>(userData->driver, header);
            if (userData->leftRectified && userData->leftRectified->data().frameId == header.frameId) {
                //
                // Our disparity and left recitified images match, we have enough info to create a pointcloud
                break;
            }

            return;
        }
        default:
        {
            std::cerr << "Unknown image source: " << header.source << std::endl;
            return;
        }
    };

    //
    // Note this implementation of writing ply files can be slow since they are written in ascii

    std::cout << "Saving pointcloud for image header " << header.frameId << std::endl;
    savePly(std::to_string(header.frameId) + ".ply", reprojectDisparity(userData->disparity,
                                                                        userData->leftRectified,
                                                                        userData->calibration,
                                                                        userData->deviceInfo,
                                                                        userData->minDisparity));
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t mtu = 7200;
    double minDisparity = 5.0;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:m:d:")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'm': mtu            = atoi(optarg);           break;
        case 'd': minDisparity   = std::stod(optarg);      break;
        default: usage(*argvPP);                           break;
        }

    Status status;

    //
    // Initialize communications.

    auto channelP = std::make_unique<ChannelWrapper>(currentAddress);
    if (nullptr == channelP) {
        std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Change MTU

    status = channelP->ptr()->setMtu(mtu);
    if (Status_Ok != status) {
        std::cerr << "Failed to set MTU to " << mtu << ": " << Channel::statusString(status) << std::endl;
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
    // Setup user data to store camera state for pointcloud reprojection

    UserData userData{channelP->ptr(), nullptr, nullptr, calibration, deviceInfo, minDisparity};

    //
    // Add callbacks

    channelP->ptr()->addIsolatedCallback(imageCallback, Source_Luma_Rectified_Left | Source_Disparity_Left, &userData);

    //
    // Start streaming

    status = channelP->ptr()->startStreams(Source_Luma_Rectified_Left | Source_Disparity_Left);
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
