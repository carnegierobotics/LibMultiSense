/**
 * @file ThermalCalUtility.cc
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

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>

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
    std::string action{};
    uint8_t imager = 0;
    std::filesystem::path calibration_file{};
};

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " -c <action> [<options>]\n"
              << "Where <options> are:\n"
              << "\t-a <current-address> : CURRENT IPV4 address (default=10.66.171.21)\n"
              << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)\n"
              << "\t-c <action>          : One of get, set, verify\n"
              << "\t-i <imager>          : Imager 0-5 in order 0a,0b,1a,1b,2a,2b (default=0)\n"
              << "\t-f <file>            : Required input for set/verify; optional output for get\n";
}

std::optional<Options> parse_options(int argc, char **argv)
{
    Options options;
    int option = 0;
    while (-1 != (option = getopt(argc, argv, "a:m:c:i:f:h")))
    {
        switch (option)
        {
            case 'a': options.ip_address = optarg; break;
            case 'm': options.mtu = static_cast<uint16_t>(std::stoul(optarg)); break;
            case 'c': options.action = optarg; break;
            case 'i':
            {
                const auto imager = std::stoul(optarg);
                if (imager > 5)
                {
                    std::cerr << "Imager must be in the range 0-5\n";
                    return std::nullopt;
                }
                options.imager = static_cast<uint8_t>(imager);
                break;
            }
            case 'f': options.calibration_file = optarg; break;
            default:
            {
                usage(*argv);
                return std::nullopt;
            }
        }
    }

    if (options.action != "get" && options.action != "set" && options.action != "verify")
    {
        std::cerr << "Calibration action must be one of get, set, verify\n";
        usage(*argv);
        return std::nullopt;
    }

    if ((options.action == "set" || options.action == "verify") &&
        options.calibration_file.empty())
    {
        std::cerr << "Calibration action '" << options.action << "' requires -f <file>\n";
        return std::nullopt;
    }

    return options;
}

std::optional<std::string> read_text_file(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        std::cerr << "Failed to open " << path << '\n';
        return std::nullopt;
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

int run_calibration(lms::Channel &channel, const Options &options)
{
    if (options.action == "get")
    {
        const auto calibration = thermal::get_calibration(channel, options.imager);
        if (!calibration)
        {
            std::cerr << "Failed to read calibration for imager "
                      << static_cast<unsigned>(options.imager) << '\n';
            return 1;
        }

        const std::string serialized = thermal::serialize_calibration(*calibration);

        if (options.calibration_file.empty())
        {
            std::cout << serialized;
        }
        else
        {
            std::ofstream output(options.calibration_file, std::ios::binary);
            if (!output)
            {
                std::cerr << "Failed to open " << options.calibration_file << '\n';
                return 1;
            }
            output << serialized;
            std::cout << "wrote " << options.calibration_file << '\n';
        }
        return 0;
    }

    const auto contents = read_text_file(options.calibration_file);
    if (!contents)
    {
        return 1;
    }

    const auto calibration = thermal::deserialize_calibration(*contents);
    if (!calibration)
    {
        std::cerr << "Invalid calibration in " << options.calibration_file << '\n';
        return 1;
    }

    const std::string serialized = thermal::serialize_calibration(*calibration);
    const auto status = thermal::set_calibration(channel, options.imager, *calibration);
    if (status != lms::Status::OK)
    {
        std::cerr << "Failed to upload and load calibration for imager "
                  << static_cast<unsigned>(options.imager) << ": "
                  << lms::to_string(status) << '\n';
        return 1;
    }
    std::cout << "uploaded and loaded calibration for imager "
              << static_cast<unsigned>(options.imager) << '\n';

    if (options.action == "set")
    {
        return 0;
    }

    //
    // set_calibration restarts the thermal pipeline before returning, so this
    // verifies the calibration after it has been loaded.
    //
    const auto readback = thermal::get_calibration(channel, options.imager);
    if (!readback)
    {
        std::cerr << "Verify failed: could not read calibration back\n";
        return 1;
    }

    if (thermal::serialize_calibration(*readback) != serialized)
    {
        std::cerr << "Verify FAILED: readback calibration differs\n";
        return 1;
    }

    std::cout << "Verify OK: readback calibration matches\n";
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
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
        std::cerr << "ThermalCalUtility requires an STT6 thermal setup; connected device '"
                  << device.camera_name << "' has hardware revision "
                  << static_cast<unsigned>(device.hardware_revision) << '\n';
        return 1;
    }

    //
    // Activating the semantic thermal stream also activates the application,
    // which makes its calibration control interface available.
    //
    const auto start_status = channel->start_streams({lms::DataSource::THERMAL});
    if (start_status != lms::Status::OK)
    {
        std::cerr << "Failed to start thermal application: "
                  << lms::to_string(start_status) << '\n';
        return 1;
    }

    const int result = run_calibration(*channel, *options);
    channel->stop_streams({lms::DataSource::THERMAL});
    return result;
}
