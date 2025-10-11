/**
 * @file multisense_utilities_test.cc
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
 *   2025-01-10, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <cmath>

#include <gtest/gtest.h>

#include <MultiSense/MultiSenseUtilities.hh>

using namespace multisense;

//
// Create a disparity image of a circular disk centered in the image
//
Image create_example_disparity_image(const CameraCalibration &left_calibration,
                                     const CameraCalibration &right_calibration,
                                     double obstacle_radius_m,
                                     double obstacle_distance_m,
                                     size_t width,
                                     size_t height)
{
    const double left_fx = left_calibration.P[0][0];
    const double left_cx = left_calibration.P[0][2];
    const double left_fy = left_calibration.P[1][1];
    const double left_cy = left_calibration.P[1][2];

    const double right_fx = right_calibration.P[0][0];
    const double right_cx = right_calibration.P[0][2];
    const double tx = right_calibration.P[0][3] / right_calibration.P[0][0];

    const double obstacle_distance_m2 = obstacle_distance_m * obstacle_distance_m;
    const double obstacle_radius_m2 = obstacle_radius_m * obstacle_radius_m;

    std::vector<uint8_t> data(width * height * sizeof(uint16_t), 0);

    uint16_t* raw_data = reinterpret_cast<uint16_t*>(data.data());

    for (size_t v = 0 ; v < height ; ++v)
    {
        for (size_t u = 0 ; u < width ; ++u)
        {
            // convert pixel to ray
            const double ray_x = (u / left_fx) - (left_cx / left_fx);
            const double ray_y = (v / left_fy) - (left_cy / left_fy);

            // scale ray so that z = obstacle_distance_m and test if we are within the disc radius compute our disparity
            if (((ray_x * ray_x * obstacle_distance_m2) + (ray_y * ray_y * obstacle_distance_m2) ) < obstacle_radius_m2)
            {
                // project into the right image. Note v will be the same
                const double right_u = ((right_fx * ray_x * obstacle_distance_m) + (right_cx * obstacle_distance_m) + (tx * right_fx)) / obstacle_distance_m;

                const uint16_t disparity = static_cast<uint16_t>((static_cast<double>(u) - right_u) * 16.0);

                raw_data[v * width + u] = disparity;
            }
        }
    }

    return Image{std::make_shared<const std::vector<uint8_t>>(data),
                 0,
                 width * height * sizeof(uint16_t),
                 Image::PixelFormat::MONO16,
                 static_cast<int>(width),
                 static_cast<int>(height),
                 {},
                 {},
                 DataSource::LEFT_DISPARITY_RAW,
                 left_calibration};
}

TEST(QMatrix, reproject)
{
    const float fx = 1500.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;
    const float tx = -0.27;

    CameraCalibration left_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    CameraCalibration right_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, fx * tx}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    QMatrix q{left_calibration, right_calibration};

    // Project a dummy point into the left/right camera
    const double x = 0.5;
    const double y = 1.5;
    const double z = 5.5;

    // Left image
    const double left_u = ((fx * x) + (cx * z)) / z;
    const double left_v = ((fy * y) + (cy * z)) / z;

    // right image
    const double right_u = ((fx * x) + (cx * z) + (tx * fx)) / z;

    const double disparity = left_u - right_u;

    const auto repojected_pt = q.reproject(Pixel{static_cast<size_t>(std::round(left_u)), static_cast<size_t>(std::round(left_v))}, disparity);

    const double epsilon = 1e-2;
    EXPECT_NEAR(x, repojected_pt.x, epsilon);
    EXPECT_NEAR(y, repojected_pt.y, epsilon);
    EXPECT_NEAR(z, repojected_pt.z, epsilon);
}

TEST(create_depth_image, mono_and_float)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;
    const float tx = -0.27;
    const float aux_tx = -0.033;

    CameraCalibration left_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    CameraCalibration right_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, fx * tx}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, fx * aux_tx}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    const double disk_distance_m = 4.0;

    const auto disparity_image = create_example_disparity_image(left_calibration, right_calibration, 3.0, disk_distance_m, 1920, 1200);

    ImageFrame frame{0, {}, StereoCalibration{left_calibration, right_calibration, aux_calibration}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity_image);

    const float invalid = 7000.0;

    const auto float_depth_image = create_depth_image(frame,
                                                      Image::PixelFormat::FLOAT32,
                                                      DataSource::LEFT_DISPARITY_RAW,
                                                      false,
                                                      invalid);

    const auto aux_float_depth_image = create_depth_image(frame,
                                                          Image::PixelFormat::FLOAT32,
                                                          DataSource::LEFT_DISPARITY_RAW,
                                                          true,
                                                          invalid);

    const auto mono16_depth_image = create_depth_image(frame,
                                                       Image::PixelFormat::MONO16,
                                                       DataSource::LEFT_DISPARITY_RAW,
                                                       false,
                                                       invalid);

    ASSERT_TRUE(float_depth_image);
    ASSERT_TRUE(mono16_depth_image);
    ASSERT_TRUE(aux_float_depth_image);

    for (int v = 0 ; v < disparity_image.height ; ++v)
    {
        for (int u = 0 ; u < disparity_image.width ; ++u)
        {
            if (disparity_image.at<uint16_t>(u, v).value() == 0)
            {
                EXPECT_DOUBLE_EQ(float_depth_image->at<float>(u, v).value(), invalid);
                EXPECT_EQ(mono16_depth_image->at<uint16_t>(u, v).value(), invalid);
            }
            else
            {
                EXPECT_DOUBLE_EQ(float_depth_image->at<float>(u, v).value(), disk_distance_m);

                // depths are in mm
                EXPECT_EQ(mono16_depth_image->at<uint16_t>(u, v).value(), static_cast<uint16_t>(disk_distance_m * 1000));

                // compute our aux pixel location for this depth measurement
                const int aux_u = u - static_cast<int>((aux_tx/ tx) * disparity_image.at<uint16_t>(u, v).value() / 16.0);
                EXPECT_EQ(aux_float_depth_image->at<float>(aux_u, v).value(), disk_distance_m);
            }
        }
    }
}
