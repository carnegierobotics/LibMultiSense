/**
 * @file ImageCalUtility.cc
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
 *   2025-03-20, malvarado@carnegierobotics.com, IRAD, Created file.
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

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string.h>

#include <MultiSense/MultiSenseChannel.hh>

#include "CalibrationYaml.hh"
#include "getopt/getopt.h"

namespace lms = multisense;

namespace
{

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " -e <extrinsics-file> -i <intrinsics-file> [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address> : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-s                   : Set the input calibration(default=false)" << std::endl;
    std::cerr << "\t-y                   : Disable confirmation prompt (default=false)" << std::endl;
    exit(1);
}

template <size_t H, size_t W>
std::array<std::array<float, W>, H> convert_matrix(const std::vector<float> &source)
{
    if ((H * W) != source.size())
    {
        throw std::runtime_error("Matrix sizes do not match");
    }

    std::array<std::array<float, W>, H> output;

    memcpy(&output[0][0], source.data(), sizeof(float) * source.size());

    return output;
}

lms::CameraCalibration::DistortionType get_distortion_type(size_t length)
{
    switch (length)
    {
        case 0:
            return lms::CameraCalibration::DistortionType::NONE;
        case 5:
            return lms::CameraCalibration::DistortionType::PLUMBBOB;
        case 8:
            return lms::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL;
    }

    return lms::CameraCalibration::DistortionType::NONE;
}

lms::CameraCalibration read_cal(const std::map<std::string, std::vector<float>> &intrinsics,
                                const std::map<std::string, std::vector<float>> &extrinsics,
                                size_t index)
{
    lms::CameraCalibration output{};

    output.K = convert_matrix<3, 3>(intrinsics.at("M" + std::to_string(index)));
    output.R = convert_matrix<3, 3>(extrinsics.at("R" + std::to_string(index)));
    output.P = convert_matrix<3, 4>(extrinsics.at("P" + std::to_string(index)));
    output.D = intrinsics.at("D" + std::to_string(index));
    output.distortion_type = get_distortion_type(output.D.size());

    return output;
}

void write_cal(std::ofstream &intrinsics, std::ofstream &extrinsics, const lms::CameraCalibration &cal, size_t index)
{
    writeMatrix(intrinsics, "M" + std::to_string(index), 3, 3, &cal.K[0][0]);
    writeMatrix(intrinsics, "D" + std::to_string(index), 1, cal.D.size(), cal.D.data());
    writeMatrix(extrinsics, "R" + std::to_string(index), 3, 3, &cal.R[0][0]);
    writeMatrix(extrinsics, "P" + std::to_string(index), 3, 4, &cal.P[0][0]);
}

}

int main(int argc, char** argv)
{
    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;
    std::filesystem::path intrinsics_path{};
    std::filesystem::path extrinsics_path{};
    bool set_cal = false;
    bool disable_confirmation = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:e:i:sy")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = static_cast<uint16_t>(atoi(optarg)); break;
            case 'i': intrinsics_path = optarg; break;
            case 'e': extrinsics_path = optarg; break;
            case 's': set_cal = true; break;
            case 'y': disable_confirmation = true; break;
            default: usage(*argv); break;
        }
    }

    if (set_cal && (!std::filesystem::exists(intrinsics_path) || !std::filesystem::exists(extrinsics_path)))
    {
        std::cerr << "Invalid input or calibration paths" << std::endl;
        usage(*argv);
        return 1;
    }

    if (!set_cal && !disable_confirmation &&
        (std::filesystem::exists(intrinsics_path) || std::filesystem::exists(extrinsics_path)))
    {
        std::cout << "One or both of the input file already exists\n" << std::endl;
        std::cout << "Really overwrite these files? (y/n):" << std::endl;

        if (const int reply = getchar(); reply != 'Y' && reply != 'y')
        {
            std::cerr << "Aborting" << std::endl;
            return 1;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    auto calibration = channel->get_calibration();

    if (set_cal)
    {
        std::cout << "Attempting to set the MultiSense calibration" << std::endl;
        std::ifstream intrinsics{};
        std::ifstream extrinsics{};

        intrinsics.open(intrinsics_path);
        extrinsics.open(extrinsics_path);

        if (!intrinsics.is_open() || !extrinsics.is_open())
        {
            std::cerr << "Error opening calibration files" << std::endl;
            return 1;
        }

        std::map<std::string, std::vector<float>> intrinsics_data{};
        parseYaml(intrinsics, intrinsics_data);

        std::map<std::string, std::vector<float>> extrinsics_data{};
        parseYaml(extrinsics, extrinsics_data);

        calibration.left = read_cal(intrinsics_data, extrinsics_data, 1);
        calibration.right = read_cal(intrinsics_data, extrinsics_data, 2);

        if (calibration.aux)
        {
            calibration.aux = read_cal(intrinsics_data, extrinsics_data, 3);
        }

        if (const auto status = channel->set_calibration(calibration); status != lms::Status::OK)
        {
            std::cerr << "Unable to set the calibration" << std::endl;
            return 1;
        }

        std::cout << "Image calibration successfully updated" << std::endl;
    }
    else
    {
        std::ofstream intrinsics{};
        std::ofstream extrinsics{};

        intrinsics.open(intrinsics_path, std::ios_base::out | std::ios_base::trunc);
        extrinsics.open(extrinsics_path, std::ios_base::out | std::ios_base::trunc);

        if (!intrinsics.is_open() || !extrinsics.is_open())
        {
            std::cerr << "Error opening calibration files" << std::endl;
            return 1;
        }

        intrinsics << "%YAML:1.0\n";
        extrinsics << "%YAML:1.0\n";

        write_cal(intrinsics, extrinsics, calibration.left, 1);
        write_cal(intrinsics, extrinsics, calibration.right, 2);

        if (calibration.aux)
        {
            write_cal(intrinsics, extrinsics, calibration.right, 3);
        }
    }

    return 0;
}

