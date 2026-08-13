/**
 * @file ThermalUtility.cc
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
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
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

struct Options
{
    std::string ip_address = "10.66.171.21";
    uint16_t mtu = 1500;
    std::size_t frame_groups = 1;
    std::filesystem::path output_directory = ".";
    bool rectified = false;
    std::optional<uint8_t> bits_per_pixel = std::nullopt;
    std::optional<uint16_t> post_proc_mask = std::nullopt;
};

volatile std::sig_atomic_t stop_requested = 0;

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]\n"
              << "Where <options> are:\n"
              << "\t-a <current-address> : CURRENT IPV4 address (default=10.66.171.21)\n"
              << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)\n"
              << "\t-n <frame-groups>    : Number of groups to read; 0 runs until Ctrl+C (default=1)\n"
              << "\t-o <directory>       : Directory for output PGM files (default=current directory)\n"
              << "\t-r                   : Request rectified thermal images\n"
              << "\t-b <8|16>            : Request 8bpp tonemapped or 16bpp raw pixels (restarts the pipeline)\n"
              << "\t-p <mask>            : Set the imager correction mask, e.g. 0x37f (decimal or 0x hex)\n"
              << "\t                       bits 0:FFC 1:Gain 2:Temp 3:BadPixel 4:SCNR 5:SRNR\n"
              << "\t                            6:TF 7:SPNR 8:SFFC 9:TOSS 10:BCNR\n"
              << "\t                       vendor guidance: leave all on except 7 (burn-in) and 9 (artifacts)\n";
}

std::optional<Options> parse_options(int argc, char **argv)
{
    Options options;
    int option = 0;
    while (-1 != (option = getopt(argc, argv, "a:m:n:o:b:p:rh")))
    {
        switch (option)
        {
            case 'a': options.ip_address = optarg; break;
            case 'm': options.mtu = static_cast<uint16_t>(std::stoul(optarg)); break;
            case 'n': options.frame_groups = std::stoul(optarg); break;
            case 'o': options.output_directory = optarg; break;
            case 'r': options.rectified = true; break;
            case 'b':
            {
                const auto bits = std::stoul(optarg);
                if (bits != 8 && bits != 16)
                {
                    std::cerr << "Bits per pixel must be 8 or 16\n";
                    return std::nullopt;
                }
                options.bits_per_pixel = static_cast<uint8_t>(bits);
                break;
            }
            case 'p':
            {
                //
                // base 0 so both 0x37f and 895 work
                //
                const auto mask = std::stoul(optarg, nullptr, 0);
                if (mask & ~static_cast<unsigned long>(thermal::POST_PROCESSING_VALID))
                {
                    std::cerr << "Correction mask has undefined bits (valid 0x7ff)\n";
                    return std::nullopt;
                }
                options.post_proc_mask = static_cast<uint16_t>(mask);
                break;
            }
            default:
            {
                usage(*argv);
                return std::nullopt;
            }
        }
    }

    return options;
}

#ifdef WIN32
BOOL WINAPI signal_handler(DWORD control_type)
{
    (void) control_type;
    stop_requested = 1;
    return TRUE;
}
#else
void signal_handler(int signal)
{
    (void) signal;
    stop_requested = 1;
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

void print_config(lms::Channel &channel)
{
    const auto config = thermal::query_config(channel);
    if (!config)
    {
        return;
    }

    std::cout << "thermal config: " << config->width << "x" << config->height
              << " mono" << static_cast<unsigned>(config->bits_per_pixel)
              << (config->rectified ? " rectified" : " raw")
              << ", enables=0x" << std::hex << config->imager_enable_mask
              << ", correction mask=0x" << config->post_proc_mask.value_or(0) << std::dec
              << '\n';
}

bool configure_device(lms::Channel &channel, const Options &options)
{
    if (!options.rectified && !options.bits_per_pixel && !options.post_proc_mask)
    {
        return true;
    }

    if (options.bits_per_pixel && options.post_proc_mask)
    {
        std::cerr << "Warning: a bits-per-pixel change restarts the pipeline and clears the\n"
                     "         correction mask. Set the mask in a separate run afterwards.\n";
    }

    auto config = thermal::query_config(channel);
    if (!config)
    {
        std::cerr << "Failed to query thermal configuration\n";
        return false;
    }

    if (options.rectified)
    {
        config->rectified = true;
    }
    if (options.bits_per_pixel)
    {
        config->bits_per_pixel = *options.bits_per_pixel;
    }

    //
    // Carry the mask only when it was asked for, so a run that just wants rectification does not re-send one
    //
    config->post_proc_mask = options.post_proc_mask;

    const auto status = thermal::send_config(channel, *config);
    if (status != lms::Status::OK)
    {
        std::cerr << "Failed to configure thermal application: " << lms::to_string(status) << '\n';
        return false;
    }
    return true;
}

void print_frame_group(const thermal::FrameGroup &group)
{
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        group.camera_timestamp.time_since_epoch());
    const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        group.camera_timestamp.time_since_epoch() - seconds);

    std::cout << "frame " << group.frame_id << ": " << group.images.size()
              << " images, timestamp " << seconds.count() << "."
              << microseconds.count() << (group.ptp_locked ? " (PTP)" : "") << '\n';

    for (const auto &thermal_image : group.images)
    {
        const auto &image = thermal_image.image;
        const auto range = pixel_range(image);
        const auto bits_per_pixel = image.format == lms::Image::PixelFormat::MONO8 ? 8 : 16;
        std::cout << "  imager " << static_cast<unsigned>(thermal_image.imager_id)
                  << ": " << image.width << "x" << image.height
                  << " mono" << bits_per_pixel
                  << (thermal_image.rectified ? " rectified" : " raw")
                  << ", min=" << range.first << ", max=" << range.second << '\n';
    }
}

bool prepare_output_directory(const std::filesystem::path &output_directory)
{
    try
    {
        if (!std::filesystem::exists(output_directory))
        {
            std::filesystem::create_directories(output_directory);
        }
        if (!std::filesystem::is_directory(output_directory))
        {
            std::cerr << "Output path is not a directory: " << output_directory << '\n';
            return false;
        }
    }
    catch (const std::filesystem::filesystem_error &error)
    {
        std::cerr << "Failed to create output directory: " << error.what() << '\n';
        return false;
    }
    return true;
}

bool save_frame_group(const thermal::FrameGroup &group,
                      const std::filesystem::path &output_directory)
{
    bool success = true;
    for (const auto &thermal_image : group.images)
    {
        const std::string filename =
            "thermal_frame_" + std::to_string(group.frame_id) +
            "_imager_" + std::to_string(thermal_image.imager_id) +
            (thermal_image.rectified ? "_rectified.pgm" : "_raw.pgm");
        const auto output_path = output_directory / filename;

        if (lms::write_image(thermal_image.image, output_path))
        {
            std::cout << "  saved " << output_path << '\n';
        }
        else
        {
            std::cerr << "Failed to save thermal image: " << output_path << '\n';
            success = false;
        }
    }
    return success;
}

bool read_frame_groups(lms::Channel &channel,
                       const std::size_t requested_groups,
                       const std::filesystem::path &output_directory)
{
    std::size_t received_groups = 0;
    while (!stop_requested &&
           (requested_groups == 0 || received_groups < requested_groups))
    {
        const auto packet = channel.get_next_secondary_application_data();
        if (!packet || !packet->payload)
        {
            continue;
        }

        try
        {
            const auto group = thermal::deserialize_frame_group(packet->payload);
            if (!group)
            {
                throw std::runtime_error("Payload is shorter than a thermal frame-group header");
            }
            print_frame_group(*group);
            if (!save_frame_group(*group, output_directory))
            {
                return false;
            }
            ++received_groups;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Invalid thermal frame group: " << error.what() << '\n';
        }
    }
    return true;
}

int run_thermal_stream(lms::Channel &channel, const Options &options)
{
    if (!configure_device(channel, options))
    {
        return 1;
    }
    if (!prepare_output_directory(options.output_directory))
    {
        return 1;
    }

    print_config(channel);
    return read_frame_groups(channel, options.frame_groups, options.output_directory) ? 0 : 1;
}

} // namespace

int main(int argc, char **argv)
{
#ifdef WIN32
    SetConsoleCtrlHandler(signal_handler, TRUE);
#else
    signal(SIGINT, signal_handler);
#endif

    const auto options = parse_options(argc, argv);
    if (!options)
    {
        return 1;
    }

    const auto channel = lms::Channel::create(
        lms::Channel::Config{options->ip_address, options->mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel\n";
        return 1;
    }

    const auto device = channel->get_info().device;
    if (device.hardware_revision !=
        lms::MultiSenseInfo::DeviceInfo::HardwareRevision::STT6)
    {
        std::cerr << "ThermalUtility requires an STT6 thermal setup; connected device '"
                  << device.camera_name << "' has hardware revision "
                  << static_cast<unsigned>(device.hardware_revision) << '\n';
        return 1;
    }

    const auto start_status = channel->start_streams({lms::DataSource::THERMAL});
    if (start_status != lms::Status::OK)
    {
        std::cerr << "Failed to start thermal stream: "
                  << lms::to_string(start_status) << '\n';
        return 1;
    }

    const int result = run_thermal_stream(*channel, *options);
    channel->stop_streams({lms::DataSource::THERMAL});
    return result;
}
