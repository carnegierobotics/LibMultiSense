/**
 * @file RectifiedFocalLengthUtility.cc
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
 *   2025-02-08, malvarado@carnegierobotics.com, IRAD, Created file.
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

#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "getopt/getopt.h"

namespace lms = multisense;

namespace
{

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>            : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                        : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-r <new-rectified-focal-length> : The new rectified focal length" << std::endl;
    std::cerr << "\t-s                              : Set the new calibration (Default is query)" << std::endl;
    exit(1);
}

lms::StereoCalibration update_calibration(lms::StereoCalibration cal, float new_focal_length)
{
    std::cout << "Updating focal length from " << cal.left.P[0][0] << " to " << new_focal_length << std::endl;

    cal.left.P[0][0] = new_focal_length;
    cal.left.P[1][1] = new_focal_length;

    const float right_tx = cal.right.P[0][3] / cal.right.P[0][0];
    cal.right.P[0][0] = new_focal_length;
    cal.right.P[1][1] = new_focal_length;
    cal.right.P[0][3] = new_focal_length * right_tx;

    if (cal.aux)
    {
        const float aux_tx = cal.aux->P[0][3] / cal.aux->P[0][0];
        const float aux_ty = cal.aux->P[1][3] / cal.aux->P[1][1];

        cal.aux->P[0][0] = new_focal_length;
        cal.aux->P[1][1] = new_focal_length;
        cal.aux->P[0][3] = new_focal_length * aux_tx;
        cal.aux->P[1][3] = new_focal_length * aux_ty;
    }

    return cal;
}

}

int main(int argc, char** argv)
{
    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;
    std::optional<float> rectified_focal_length = std::nullopt;
    bool set = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:r:s")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = static_cast<uint16_t>(atoi(optarg)); break;
            case 'r': rectified_focal_length = std::stof(optarg); break;
            case 's': set = true; break;
            default: usage(*argv); break;
        }
    }

    if (!rectified_focal_length || rectified_focal_length.value() < 0.0)
    {
        std::cerr << "Invalid input rectified focal length" << std::endl;;
        usage(*argv);
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    const auto current_calibration = channel->get_calibration();

    const auto new_calibration = update_calibration(current_calibration, rectified_focal_length.value());

    if (set)
    {
        if (channel->set_calibration(new_calibration) != lms::Status::OK)
        {
            std::cerr << "Failed to set the updated calibration" << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "Please add the \"-s\" argument to write the new rectified focal length to the camera" << std::endl;
    }

    return 0;
}
