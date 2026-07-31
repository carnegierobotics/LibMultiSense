/**
 * @file SecondaryAppUtility.cc
 *
 * Copyright 2026
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
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseSecondaryApplication.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "getopt/getopt.h"

namespace lms = multisense;
namespace thermal = multisense::secondary_application::thermal;

namespace
{

volatile bool done = false;

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current-address> : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-n <frame-groups>    : Number of groups to read; 0 runs until Ctrl+C (default=1)" << std::endl;
    std::cerr << "\t-r <0|1>             : Request raw or rectified thermal images" << std::endl;
    exit(1);
}

#ifdef WIN32
BOOL WINAPI signal_handler(DWORD control_type)
{
    (void) control_type;
    done = true;
    return TRUE;
}
#else
void signal_handler(int signal)
{
    (void) signal;
    done = true;
}
#endif

std::pair<uint16_t, uint16_t> pixel_range(const lms::Image &image)
{
    const uint8_t *data = image.raw_data->data() + image.image_data_offset;
    if (image.format == lms::Image::PixelFormat::MONO8)
    {
        const auto result = std::minmax_element(data, data + image.image_data_length);
        return {static_cast<uint16_t>(*result.first), static_cast<uint16_t>(*result.second)};
    }

    uint16_t minimum = std::numeric_limits<uint16_t>::max();
    uint16_t maximum = std::numeric_limits<uint16_t>::min();
    for (size_t offset = 0; offset < image.image_data_length; offset += sizeof(uint16_t))
    {
        uint16_t pixel = 0;
        std::memcpy(&pixel, data + offset, sizeof(pixel));
        minimum = std::min(minimum, pixel);
        maximum = std::max(maximum, pixel);
    }
    return {minimum, maximum};
}

} // namespace

int main(int argc, char **argv)
{
#ifdef WIN32
    SetConsoleCtrlHandler(signal_handler, TRUE);
#else
    signal(SIGINT, signal_handler);
#endif

    std::string ip_address = "10.66.171.21";
    uint16_t mtu = 1500;
    std::size_t requested_groups = 1;
    int rectified = -1;

    int option;
    while (-1 != (option = getopt(argc, argv, "a:m:n:r:h")))
    {
        switch (option)
        {
            case 'a': ip_address = optarg; break;
            case 'm': mtu = static_cast<uint16_t>(std::stoul(optarg)); break;
            case 'n': requested_groups = std::stoul(optarg); break;
            case 'r': rectified = std::stoi(optarg); break;
            default: usage(*argv); break;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    if (const auto status = channel->start_streams({lms::DataSource::THERMAL});
        status != lms::Status::OK)
    {
        std::cerr << "Failed to start thermal stream: " << lms::to_string(status) << std::endl;
        return 1;
    }

    if (rectified != -1)
    {
        if (rectified != 0 && rectified != 1)
        {
            usage(*argv);
        }
        thermal::Control control;
        control.command = thermal::ControlCommand::SET_RECTIFIED;
        control.value = static_cast<uint32_t>(rectified);
        if (const auto status = lms::send_secondary_application_control(*channel, control);
            status != lms::Status::OK)
        {
            std::cerr << "Failed to configure rectification: " << lms::to_string(status) << std::endl;
            channel->stop_streams({lms::DataSource::THERMAL});
            return 1;
        }
    }

    if (const auto config =
            lms::get_secondary_application_config<thermal::Config>(*channel); config)
    {
        std::cout << "thermal config: " << config->width << "x" << config->height
                  << " mono" << static_cast<unsigned>(config->bits_per_pixel)
                  << (config->rectified ? " rectified" : " raw")
                  << ", enables=0x" << std::hex << config->imager_enable_mask << std::dec
                  << std::endl;
    }

    std::size_t received_groups = 0;
    while (!done && (requested_groups == 0 || received_groups < requested_groups))
    {
        const auto packet = channel->get_next_secondary_application_data();
        if (!packet || !packet->payload)
            continue;

        try
        {
            const auto group = lms::deserialize_thermal_frame_group(packet->payload);
            if (!group)
            {
                throw std::runtime_error("Payload is shorter than a thermal frame-group header");
            }
            const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
                group->camera_timestamp.time_since_epoch());
            const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                group->camera_timestamp.time_since_epoch() - seconds);
            std::cout << "frame " << group->frame_id << ": " << group->images.size()
                      << " images, timestamp " << seconds.count() << "."
                      << microseconds.count() << (group->ptp_locked ? " (PTP)" : "")
                      << std::endl;

            for (const auto &thermal_image : group->images)
            {
                const auto range = pixel_range(thermal_image.image);
                const auto bits_per_pixel =
                    thermal_image.image.format == lms::Image::PixelFormat::MONO8 ? 8 : 16;
                std::cout << "  imager " << static_cast<unsigned>(thermal_image.imager_id)
                          << ": " << thermal_image.image.width << "x" << thermal_image.image.height
                          << " mono" << bits_per_pixel
                          << (thermal_image.rectified ? " rectified" : " raw")
                          << ", min=" << range.first << ", max=" << range.second
                          << std::endl;
            }
            ++received_groups;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Invalid thermal frame group: " << error.what() << std::endl;
        }
    }

    channel->stop_streams({lms::DataSource::THERMAL});
    return 0;
}
