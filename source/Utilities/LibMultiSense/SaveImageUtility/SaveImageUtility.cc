/**
 * @file SaveImageUtility.cc
 *
 * Copyright 2013-2025
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
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
    std::cerr << "\t-n <number-of-images>   : Number of images to save (default=1)" << std::endl;
    std::cerr << "\t-d                      : Save depth images" << std::endl;
    std::cerr << "\t-l                      : Save left rectified images" << std::endl;
    std::cerr << "\t-c                      : Save color images" << std::endl;
    exit(1);
}

void save_image(const lms::ImageFrame &frame, const lms::DataSource &source)
{
    const auto base_path = std::to_string(frame.frame_id) +  "_" +
                          std::to_string(static_cast<int>(source));
    switch (source)
    {
        case lms::DataSource::LEFT_DISPARITY_RAW:
        {
            if (const auto depth_image = lms::create_depth_image(frame,
                                                                 lms::Image::PixelFormat::MONO16,
                                                                 source,
                                                                 65535); depth_image)
            {
                lms::write_image(depth_image.value(), base_path + ".pgm");
            }
            break;
        }
        case lms::DataSource::LEFT_RECTIFIED_RAW:
        {
            lms::write_image(frame.get_image(source), base_path + ".pgm");
            break;
        }
        case lms::DataSource::AUX_RAW:
        {
            if (const auto bgr = create_bgr_image(frame, source); bgr)
            {
                lms::write_image(bgr.value(), base_path+".ppm");
            }
            break;
        }
        default: return;
    }
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
    size_t number_of_images = 1;
    bool save_depth = false;
    bool save_left_rect = false;
    bool save_color = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:n:dlc")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = static_cast<uint16_t>(atoi(optarg)); break;
            case 'n': number_of_images = static_cast<size_t>(atoi(optarg)); break;
            case 'd': save_depth = true; break;
            case 'l': save_left_rect = true; break;
            case 'c': save_color = true; break;
            default: usage(*argv); break;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    //
    // Query Static info from the camera
    //
    auto info = channel->get_info();

	std::cout << "Firmware build date :  " << info.version.firmware_build_date << std::endl;
	std::cout << "Firmware version    :  " << info.version.firmware_version.to_string() << std::endl;
	std::cout << "Hardware version    :  0x" << std::hex << info.version.hardware_version << std::endl;
	std::cout << std::dec;

    //
    // QuerySet dynamic config from the camera
    //
    auto config = channel->get_configuration();
    config.frames_per_second = 30.0;
    if (const auto status = channel->set_configuration(config); status != lms::Status::OK)
    {
        std::cerr << "Cannot set config" << std::endl;
        return 1;
    }

    std::vector<lms::DataSource> image_streams{};
    if (save_depth) image_streams.push_back(lms::DataSource::LEFT_DISPARITY_RAW);
    if (save_left_rect) image_streams.push_back(lms::DataSource::LEFT_RECTIFIED_RAW);
    if (save_color) image_streams.push_back(lms::DataSource::AUX_RAW);

    if (image_streams.empty())
    {
        std::cerr << "No image streams requested" << std::endl;
        return 0;
    }

    //
    // Start a single image stream
    //
    if (const auto status = channel->start_streams(image_streams); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    //
    // Only save the first image
    //
    size_t saved_images = 0;

    while(!done)
    {
        if (saved_images < number_of_images)
        {
            if (const auto image_frame = channel->get_next_image_frame(); image_frame)
            {
                for (const auto &stream : image_streams)
                {
                    save_image(image_frame.value(), stream);
                }
            }
            ++saved_images;
        }

        if (const auto status = channel->get_system_status(); status)
        {
            if (status->time) std::cout << "Camera Time(ns): " << status->time->camera_time.count() << ", ";
            std::cout << "System Ok: " << status->system_ok << ", ";
            if (status->temperature) std::cout << "FPGA Temp (C): " << status->temperature->fpga_temperature << ", ";
            if (status->temperature) std::cout << "Left Imager Temp (C): " << status->temperature->left_imager_temperature << ", ";
            if (status->temperature) std::cout << "Right Imager Temp (C): " << status->temperature->right_imager_temperature << ", ";
            if (status->power) std::cout << "Input Voltage (V): " << status->power->input_voltage << ", ";
            if (status->power) std::cout << "Input Current (A): " << status->power->input_current << ", ";
            if (status->power) std::cout << "FPGA Power (W): " << status->power->fpga_power << " , ";
            std::cout << "Received Messages: " << status->client_network.received_messages << " , ";
            std::cout << "Dropped Messages: " << status->client_network.dropped_messages << std::endl;
        }
        else
        {
            std::cerr << "Failed to query sensor status" << std::endl;
        }

        std::this_thread::sleep_for(1s);
    }

    channel->stop_streams({lms::DataSource::ALL});

    return 0;
}
