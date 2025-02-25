/**
 * @file calibration.cc
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
 *   2025-01-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include "details/legacy/calibration.hh"

namespace multisense {
namespace legacy {

bool is_valid(const crl::multisense::details::wire::CameraCalData &cal)
{
    //
    // Do a crude check the un-rectified focal length image center and distortion values. If these values
    // are zero, then the cal is invalid
    //
    if (cal.M[0][0] < 0.1 || cal.M[0][2] < 0.1 || cal.M[1][1] < 0.1 || cal.M[1][2] < 0.1 || std::abs(cal.D[0]) < 1e-10)
    {
        return false;
    }

    return true;
}

CameraCalibration convert(const crl::multisense::details::wire::CameraCalData &cal)
{
    const auto distortion_type = (cal.D[5] == 0.f && cal.D[6] == 0.f && cal.D[7] == 0.f) ?
                                 CameraCalibration::DistortionType::PLUMBOB :
                                 CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL;

    CameraCalibration output{};

    CPY_ARRAY_2(output.K, cal.M, 3, 3);
    CPY_ARRAY_2(output.R, cal.R, 3, 3);
    CPY_ARRAY_2(output.P, cal.P, 3, 4);

    output.distortion_type = distortion_type;

    if (distortion_type == CameraCalibration::DistortionType::PLUMBOB)
    {
        output.D.resize(5);
        memcpy(&output.D[0], cal.D, sizeof(float) * 5);
    }
    else
    {
        output.D.resize(8);
        memcpy(&output.D[0], cal.D, sizeof(float) * 8);
    }

    return output;
}

crl::multisense::details::wire::CameraCalData convert(const CameraCalibration &cal)
{
    using namespace crl::multisense::details;

    wire::CameraCalData output;

    if (cal.D.size() > (sizeof(output.D)/sizeof(float)))
    {
        CRL_EXCEPTION("Invalid input distortion size");
    }

    CPY_ARRAY_2(output.M, cal.K, 3, 3);
    memset(&output.D[0], 0, sizeof(output.D));
    memcpy(&output.D[0], cal.D.data(), cal.D.size() * sizeof(float));
    CPY_ARRAY_2(output.R, cal.R, 3, 3);
    CPY_ARRAY_2(output.P, cal.P, 3, 4);

    return output;
}

StereoCalibration convert(const crl::multisense::details::wire::SysCameraCalibration &cal)
{
    using namespace crl::multisense::details;

    StereoCalibration output{};

    output.left = convert(cal.left);
    output.right = convert(cal.right);

    if (is_valid(cal.aux))
    {
        output.aux = convert(cal.aux);
    }

    return output;
}

crl::multisense::details::wire::SysCameraCalibration convert(const StereoCalibration &cal)
{
    using namespace crl::multisense::details;

    wire::SysCameraCalibration output;

    output.left = convert(cal.left);
    output.right = convert(cal.right);

    if (cal.aux)
    {
        output.aux = convert(cal.aux.value());
    }
    else
    {
        memset(output.aux.M, 0, sizeof(float) * 3 * 3);
        memset(output.aux.D, 0, sizeof(float) * 8);
        memset(output.aux.R, 0, sizeof(float) * 3 * 3);
        memset(output.aux.P, 0, sizeof(float) * 3 * 4);
    }

    return output;
}

CameraCalibration select_calibration(const StereoCalibration &input, const DataSource &source)
{
    switch(source)
    {
        case DataSource::LEFT_MONO_RAW:
        case DataSource::LEFT_MONO_COMPRESSED:
        case DataSource::LEFT_RECTIFIED_RAW:
        case DataSource::LEFT_RECTIFIED_COMPRESSED:
        case DataSource::LEFT_DISPARITY_RAW:
        case DataSource::LEFT_DISPARITY_COMPRESSED:
        case DataSource::COST_RAW:
        {
            return input.left;
        }
        case DataSource::RIGHT_MONO_RAW:
        case DataSource::RIGHT_MONO_COMPRESSED:
        case DataSource::RIGHT_RECTIFIED_RAW:
        case DataSource::RIGHT_RECTIFIED_COMPRESSED:
        {
            return input.right;
        }
        case DataSource::AUX_COMPRESSED:
        case DataSource::AUX_RECTIFIED_COMPRESSED:
        case DataSource::AUX_LUMA_RAW:
        case DataSource::AUX_LUMA_RECTIFIED_RAW:
        case DataSource::AUX_CHROMA_RAW:
        case DataSource::AUX_CHROMA_RECTIFIED_RAW:
        {
            if (!input.aux)
            {
                CRL_EXCEPTION("Input source corresponds to invalid aux calibration");
            }
            return input.aux.value();
        }
        default: {CRL_EXCEPTION("Input source does not correspond to a image calibration");}
    }
}

CameraCalibration scale_calibration(const CameraCalibration &input, double x_scale, double y_scale)
{
    auto output = input;

    output.K[0][0] = static_cast<float>(static_cast<double>(output.K[0][0]) * x_scale); // fx
    output.K[0][2] = static_cast<float>(static_cast<double>(output.K[0][2]) * x_scale); // cx
    output.K[1][1] = static_cast<float>(static_cast<double>(output.K[1][1]) * y_scale); // fy
    output.K[1][2] = static_cast<float>(static_cast<double>(output.K[1][2]) * y_scale); // cy

    output.P[0][0] = static_cast<float>(static_cast<double>(output.P[0][0]) * x_scale); // fx
    output.P[0][2] = static_cast<float>(static_cast<double>(output.P[0][2]) * x_scale); // cx
    output.P[0][3] = static_cast<float>(static_cast<double>(output.P[0][3]) * x_scale); // fx * tx
    output.P[1][1] = static_cast<float>(static_cast<double>(output.P[1][1]) * y_scale); // fy
    output.P[1][2] = static_cast<float>(static_cast<double>(output.P[1][2]) * y_scale); // cy

    return output;
}

StereoCalibration scale_calibration(const StereoCalibration &input, double x_scale, double y_scale)
{
    auto output = input;

    output.left = scale_calibration(input.left, x_scale, y_scale);
    output.right = scale_calibration(input.right, x_scale, y_scale);
    if (input.aux)
    {
        output.aux = scale_calibration(input.aux.value(), x_scale, y_scale);
    }

    return output;
}

}
}
