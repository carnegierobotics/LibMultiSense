/**
 * @file FeatureDetectorUtility.cc
 *
 * Copyright 2013-2026
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
 *   2026-03-26, malvarado@carnegierobotics.com, IRAD, Created file.
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

#include <csignal>
#include <iostream>
#include <thread>

#define HAVE_OPENCV 1
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "getopt/getopt.h"

namespace lms = multisense;

namespace
{

volatile bool done = false;

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current-address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-n <number-of-features> : Max number of features to detect (default=1500)" << std::endl;
    std::cerr << "\t-f <fps>                : Framerate (default=10)" << std::endl;
    std::cerr << "\t-h                      : Show help" << std::endl;
    exit(1);
}

#ifdef WIN32
BOOL WINAPI signal_handler(DWORD dwCtrlType)
{
    (void) dwCtrlType;
    done = true;
    return TRUE;
}
#else
void signal_handler(int sig)
{
    (void) sig;
    done = true;
}
#endif

}

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

#if WIN32
    SetConsoleCtrlHandler (signal_handler, TRUE);
#else
    signal(SIGINT, signal_handler);
#endif

    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;
    uint32_t number_of_features = 1500;
    size_t fps = 10;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:n:f:h")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = static_cast<uint16_t>(atoi(optarg)); break;
            case 'n': number_of_features = static_cast<uint32_t>(atoi(optarg)); break;
            case 'f': fps = static_cast<size_t>(atoi(optarg)); break;
            default: usage(*argv); break;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    auto config = channel->get_config();
    config.frames_per_second = fps;

    //
    // Set the feature detector config to enable the feature detector
    //
    config.feature_detector_config = lms::MultiSenseConfig::FeatureDetectorConfig{number_of_features, true, 1};
    if (const auto status = channel->set_config(config); status != lms::Status::OK && status != lms::Status::INCOMPLETE_APPLICATION)
    {
        std::cerr << "Feature detector not supported: " << lms::to_string(status) << std::endl;
        exit(1);
    }

    //
    // Start both the left image and the corresponding feature stream
    //
    const std::vector<lms::DataSource> sources = {lms::DataSource::LEFT_MONO_RAW, lms::DataSource::LEFT_ORB_FEATURES};

    if (const auto status = channel->start_streams(sources); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(!done)
    {
        if (const auto frame = channel->get_next_image_frame(); frame)
        {
            if (frame->has_image(lms::DataSource::LEFT_MONO_RAW))
            {
                cv::Mat img = frame->get_image(lms::DataSource::LEFT_MONO_RAW).cv_mat();
                cv::Mat display_img;
                cv::cvtColor(img, display_img, cv::COLOR_GRAY2BGR);

                if (frame->has_feature(lms::DataSource::LEFT_ORB_FEATURES))
                {
                    const auto& features = frame->get_feature(lms::DataSource::LEFT_ORB_FEATURES);

                    //
                    // Use the native OpenCV utility to convert keypoints and draw them
                    //
                    cv::drawKeypoints(display_img, features.cv_keypoints(), display_img, cv::Scalar(0, 255, 0));
                }

                cv::imshow("MultiSense Features", display_img);
                if (cv::waitKey(1) == 'q')
                {
                    break;
                }
            }
        }
    }

    channel->stop_streams({lms::DataSource::ALL});

    return 0;
}
