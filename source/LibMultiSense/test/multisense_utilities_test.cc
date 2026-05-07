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
#include <filesystem>

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

TEST(QMatrix, matrix)
{
    const float fx = 1500.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;
    const double tx = -0.27;

    // Use double for the product to ensure we get an exact -405.0f
    const float fxtx = static_cast<float>(static_cast<double>(fx) * tx);

    CameraCalibration left_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    CameraCalibration right_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, fxtx}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    QMatrix q{left_calibration, right_calibration};

    const auto matrix = q.matrix();

    const double expected_fytx = static_cast<double>(fy) * tx;
    const double expected_fxtx = static_cast<double>(fx) * tx;
    const double expected_fycxtx = static_cast<double>(fy) * static_cast<double>(cx) * tx;
    const double expected_fxcytx = static_cast<double>(fx) * static_cast<double>(cy) * tx;
    const double expected_fxfytx = static_cast<double>(fx) * static_cast<double>(fy) * tx;
    const double expected_fycxcxprime = 0.0;

    const double epsilon = 1e-6;

    EXPECT_NEAR(matrix[0][0], expected_fytx, epsilon);
    EXPECT_NEAR(matrix[0][1], 0.0, epsilon);
    EXPECT_NEAR(matrix[0][2], 0.0, epsilon);
    EXPECT_NEAR(matrix[0][3], -expected_fycxtx, epsilon);

    EXPECT_NEAR(matrix[1][0], 0.0, epsilon);
    EXPECT_NEAR(matrix[1][1], expected_fxtx, epsilon);
    EXPECT_NEAR(matrix[1][2], 0.0, epsilon);
    EXPECT_NEAR(matrix[1][3], -expected_fxcytx, epsilon);

    EXPECT_NEAR(matrix[2][0], 0.0, epsilon);
    EXPECT_NEAR(matrix[2][1], 0.0, epsilon);
    EXPECT_NEAR(matrix[2][2], 0.0, epsilon);
    EXPECT_NEAR(matrix[2][3], expected_fxfytx, epsilon);

    EXPECT_NEAR(matrix[3][0], 0.0, epsilon);
    EXPECT_NEAR(matrix[3][1], 0.0, epsilon);
    EXPECT_NEAR(matrix[3][2], -static_cast<double>(fy), epsilon);
    EXPECT_NEAR(matrix[3][3], expected_fycxcxprime, epsilon);
}

TEST(scale_calibration, basic)
{
    const float fx = 1500.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;
    const float tx = -0.27;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, fx * tx}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    const double scale = 0.5;
    const auto scaled_cal = scale_calibration(cal, scale);

    EXPECT_FLOAT_EQ(scaled_cal.K[0][0], fx * scale);
    EXPECT_FLOAT_EQ(scaled_cal.K[0][2], cx * scale);
    EXPECT_FLOAT_EQ(scaled_cal.K[1][1], fy * scale);
    EXPECT_FLOAT_EQ(scaled_cal.K[1][2], cy * scale);

    EXPECT_FLOAT_EQ(scaled_cal.P[0][0], fx * scale);
    EXPECT_FLOAT_EQ(scaled_cal.P[0][2], cx * scale);
    EXPECT_FLOAT_EQ(scaled_cal.P[0][3], fx * tx * scale);
    EXPECT_FLOAT_EQ(scaled_cal.P[1][1], fy * scale);
    EXPECT_FLOAT_EQ(scaled_cal.P[1][2], cy * scale);
    EXPECT_FLOAT_EQ(scaled_cal.P[1][3], 0.0);

    // Ensure other values are unchanged
    EXPECT_FLOAT_EQ(scaled_cal.K[0][1], cal.K[0][1]);
    EXPECT_FLOAT_EQ(scaled_cal.K[1][0], cal.K[1][0]);
    EXPECT_FLOAT_EQ(scaled_cal.K[2][0], cal.K[2][0]);
    EXPECT_FLOAT_EQ(scaled_cal.K[2][1], cal.K[2][1]);
    EXPECT_FLOAT_EQ(scaled_cal.K[2][2], cal.K[2][2]);
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

    ImageFrame frame{0, {}, {}, StereoCalibration{left_calibration, right_calibration, aux_calibration}, {}, {}, {}, {}, {}, {}};
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

TEST(get_aux_3d_point, basic_tests)
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

    ImageFrame frame{0, {}, {}, StereoCalibration{left_calibration, right_calibration, aux_calibration}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity_image);

    const auto invalid_point = get_aux_3d_point(frame, Pixel{0, 0}, 100, 0.01);

    ASSERT_FALSE(invalid_point);

    const auto valid_point = get_aux_3d_point(frame, Pixel{static_cast<size_t>(cx), static_cast<size_t>(cy)}, 1000, 0.5);

    ASSERT_TRUE(valid_point);

    ASSERT_DOUBLE_EQ(valid_point->z, disk_distance_m);
}

TEST(create_bgr_from_ycbcr420, gray_image)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    const size_t width = 1920;
    const size_t height = 1200;

    const uint8_t val = 42;

    std::vector<uint8_t> y_data(width * height, val);

    // Values of 128 for cb/cr result in 0 values for the corresponding color pixels
    std::vector<uint8_t> cbcr_data(width * height/2, 128);

    const Image y{std::make_shared<const std::vector<uint8_t>>(std::move(y_data)),
                  0,
                  width * height,
                  Image::PixelFormat::MONO8,
                  static_cast<int>(width),
                  static_cast<int>(height),
                  {},
                  {},
                  DataSource::AUX_LUMA_RAW,
                  aux_calibration};

    const Image cbcr{std::make_shared<const std::vector<uint8_t>>(std::move(cbcr_data)),
                     0,
                     width / 2 * height / 2,
                     Image::PixelFormat::MONO16,
                     static_cast<int>(width/2),
                     static_cast<int>(height/2),
                     {},
                     {},
                     DataSource::AUX_CHROMA_RAW,
                     aux_calibration};

    const auto bgr_image = create_bgr_from_ycbcr420(y, cbcr, DataSource::AUX_RAW);

    ASSERT_TRUE(bgr_image);
    ASSERT_EQ(bgr_image->width , static_cast<int>(width));
    ASSERT_EQ(bgr_image->height , static_cast<int>(height));

    for (size_t h = 0 ; h < height ; ++h)
    {
        for (size_t w = 0 ; w < width ; ++w)
        {
            const auto pixel = bgr_image->at<std::array<uint8_t, 3>>(w, h);
            ASSERT_TRUE(pixel);
            ASSERT_EQ(pixel->at(0), val);
            ASSERT_EQ(pixel->at(1), val);
            ASSERT_EQ(pixel->at(2), val);
        }
    }
}

TEST(create_pointcloud, basic_tests)
{
    const float fx = 1000.0;
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

    const double disk_distance_m = 4.0;

    const auto disparity_image = create_example_disparity_image(left_calibration, right_calibration, 3.0, disk_distance_m, 1920, 1200);

    ImageFrame frame{0, {}, {}, StereoCalibration{left_calibration, right_calibration, std::nullopt}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity_image);

    const auto point_cloud = create_pointcloud(frame, disk_distance_m + 3.0);

    ASSERT_TRUE(point_cloud);
    ASSERT_TRUE(!point_cloud->cloud.empty());

    for (const auto &point : point_cloud->cloud)
    {
        EXPECT_TRUE(point.z == disk_distance_m);
    }

    const auto point_cloud_empty = create_pointcloud(frame, disk_distance_m - 0.1);

    ASSERT_TRUE(point_cloud_empty);
    ASSERT_TRUE(point_cloud_empty->cloud.empty());
}

TEST(to_string, status)
{
    EXPECT_EQ(to_string(Status::OK), "OK");
    EXPECT_EQ(to_string(Status::TIMEOUT), "TIMEOUT");
    EXPECT_EQ(to_string(Status::INTERNAL_ERROR), "ERROR");
    EXPECT_EQ(to_string(Status::FAILED), "FAILED");
    EXPECT_EQ(to_string(Status::UNSUPPORTED), "UNSUPPORTED");
    EXPECT_EQ(to_string(Status::UNKNOWN), "UNKNOWN");
    EXPECT_EQ(to_string(Status::EXCEPTION), "EXCEPTION");
    EXPECT_EQ(to_string(Status::UNINITIALIZED), "UNINITIALIZED");
    EXPECT_EQ(to_string(Status::INCOMPLETE_APPLICATION), "INCOMPLETE_APPLICATION");
    EXPECT_EQ(to_string(static_cast<Status>(999)), "UNKNOWN");
}

TEST(to_string, data_source)
{
    EXPECT_EQ(to_string(DataSource::UNKNOWN), "UNKNOWN");
    EXPECT_EQ(to_string(DataSource::ALL), "ALL");
    EXPECT_EQ(to_string(DataSource::LEFT_MONO_RAW), "LEFT");
    EXPECT_EQ(to_string(DataSource::RIGHT_MONO_RAW), "RIGHT");
    EXPECT_EQ(to_string(DataSource::LEFT_MONO_COMPRESSED), "LEFT_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::RIGHT_MONO_COMPRESSED), "RIGHT_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::LEFT_RECTIFIED_RAW), "LEFT_RECTIFIED");
    EXPECT_EQ(to_string(DataSource::RIGHT_RECTIFIED_RAW), "RIGHT_RECTIFIED");
    EXPECT_EQ(to_string(DataSource::LEFT_RECTIFIED_COMPRESSED), "LEFT_RECTIFIED_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::RIGHT_RECTIFIED_COMPRESSED), "RIGHT_RECTIFIED_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::LEFT_DISPARITY_RAW), "DISPARITY");
    EXPECT_EQ(to_string(DataSource::LEFT_DISPARITY_COMPRESSED), "DISPARITY_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::AUX_COMPRESSED), "AUX_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::AUX_RECTIFIED_COMPRESSED), "AUX_RECTIFIED_COMPRESSED");
    EXPECT_EQ(to_string(DataSource::AUX_LUMA_RAW), "AUX_LUMA");
    EXPECT_EQ(to_string(DataSource::AUX_LUMA_RECTIFIED_RAW), "AUX_LUMA_RECTIFIED");
    EXPECT_EQ(to_string(DataSource::AUX_CHROMA_RAW), "AUX_CHROMA");
    EXPECT_EQ(to_string(DataSource::AUX_CHROMA_RECTIFIED_RAW), "AUX_CHROMA_RECTIFIED");
    EXPECT_EQ(to_string(DataSource::AUX_RAW), "AUX");
    EXPECT_EQ(to_string(DataSource::AUX_RECTIFIED_RAW), "AUX_RECTIFIED");
    EXPECT_EQ(to_string(DataSource::COST_RAW), "COST");
    EXPECT_EQ(to_string(DataSource::IMU), "IMU");
    EXPECT_EQ(to_string(DataSource::LEFT_ORB_FEATURES), "LEFT_ORB");
    EXPECT_EQ(to_string(DataSource::RIGHT_ORB_FEATURES), "RIGHT_ORB");
    EXPECT_EQ(to_string(DataSource::AUX_ORB_FEATURES), "AUX_ORB");
    EXPECT_EQ(to_string(DataSource::LEFT_RECTIFIED_ORB_FEATURES), "LEFT_RECTIFIED_ORB");
    EXPECT_EQ(to_string(DataSource::RIGHT_RECTIFIED_ORB_FEATURES), "RIGHT_RECTIFIED_ORB");
    EXPECT_EQ(to_string(DataSource::AUX_RECTIFIED_ORB_FEATURES), "AUX_RECTIFIED_ORB");
    EXPECT_EQ(to_string(static_cast<DataSource>(0xFFFFFFFF)), "UNKNOWN");
}

TEST(write_image, unsupported_extension)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10, 0);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              100,
              Image::PixelFormat::MONO8,
              10,
              10,
              {},
              {},
              DataSource::LEFT_MONO_RAW,
              aux_calibration};

    EXPECT_FALSE(write_image(img, "test.png"));
}

TEST(write_image, pgm_mono8)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10, 128);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              100,
              Image::PixelFormat::MONO8,
              10,
              10,
              {},
              {},
              DataSource::LEFT_MONO_RAW,
              aux_calibration};

    EXPECT_TRUE(write_image(img, "test.pgm"));
    std::filesystem::remove("test.pgm");
}

TEST(write_image, pgm_mono16)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10 * 2, 128);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              200,
              Image::PixelFormat::MONO16,
              10,
              10,
              {},
              {},
              DataSource::LEFT_DISPARITY_RAW,
              aux_calibration};

    EXPECT_TRUE(write_image(img, "test.pgm"));
    std::filesystem::remove("test.pgm");
}

TEST(write_image, ppm_bgr8)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10 * 3, 128);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              300,
              Image::PixelFormat::BGR8,
              10,
              10,
              {},
              {},
              DataSource::LEFT_MONO_RAW,
              aux_calibration};

    EXPECT_TRUE(write_image(img, "test.ppm"));
    std::filesystem::remove("test.ppm");
}

TEST(write_image, unhandled_format)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10 * 4, 128);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              400,
              Image::PixelFormat::FLOAT32,
              10,
              10,
              {},
              {},
              DataSource::LEFT_MONO_RAW,
              aux_calibration};

    EXPECT_FALSE(write_image(img, "test.ppm"));
    std::filesystem::remove("test.ppm");
}

TEST(write_image, invalid_file_path)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10, 128);
    Image img{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              100,
              Image::PixelFormat::MONO8,
              10,
              10,
              {},
              {},
              DataSource::LEFT_MONO_RAW,
              aux_calibration};

    EXPECT_FALSE(write_image(img, "/invalid_path_that_does_not_exist/test.pgm"));
}

TEST(create_depth_image, missing_disparity)
{
    ImageFrame frame{0, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    EXPECT_FALSE(create_depth_image(frame, Image::PixelFormat::FLOAT32, DataSource::LEFT_DISPARITY_RAW, false, 0.0));
}

TEST(create_depth_image, invalid_disparity_format)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10, 0);
    Image disparity{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              100,
              Image::PixelFormat::MONO8, // Should be MONO16
              10,
              10,
              {},
              {},
              DataSource::LEFT_DISPARITY_RAW,
              cal};

    ImageFrame frame{0, {}, {}, StereoCalibration{cal, cal, std::nullopt}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity);

    EXPECT_FALSE(create_depth_image(frame, Image::PixelFormat::FLOAT32, DataSource::LEFT_DISPARITY_RAW, false, 0.0));
}

TEST(create_depth_image, missing_aux_calib)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    auto disparity = create_example_disparity_image(cal, cal, 3.0, 4.0, 10, 10);
    ImageFrame frame{0, {}, {}, StereoCalibration{cal, cal, std::nullopt}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity);

    EXPECT_FALSE(create_depth_image(frame, Image::PixelFormat::FLOAT32, DataSource::LEFT_DISPARITY_RAW, true, 0.0));
}

TEST(create_depth_image, unsupported_depth_format)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    auto disparity = create_example_disparity_image(cal, cal, 3.0, 4.0, 10, 10);
    ImageFrame frame{0, {}, {}, StereoCalibration{cal, cal, cal}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity);

    EXPECT_FALSE(create_depth_image(frame, Image::PixelFormat::BGR8, DataSource::LEFT_DISPARITY_RAW, false, 0.0));
}

TEST(get_aux_3d_point, missing_disparity)
{
    ImageFrame frame{0, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    EXPECT_FALSE(get_aux_3d_point(frame, Pixel{0, 0}, 100, 0.01));
}

TEST(get_aux_3d_point, invalid_disparity_format)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    std::vector<uint8_t> data(10 * 10, 0);
    Image disparity{std::make_shared<const std::vector<uint8_t>>(data),
              0,
              100,
              Image::PixelFormat::MONO8, // Should be MONO16
              10,
              10,
              {},
              {},
              DataSource::LEFT_DISPARITY_RAW,
              cal};

    ImageFrame frame{0, {}, {}, StereoCalibration{cal, cal, cal}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity);

    EXPECT_FALSE(get_aux_3d_point(frame, Pixel{0, 0}, 100, 0.01));
}

TEST(get_aux_3d_point, missing_aux_calib)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration cal{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    auto disparity = create_example_disparity_image(cal, cal, 3.0, 4.0, 10, 10);
    ImageFrame frame{0, {}, {}, StereoCalibration{cal, cal, std::nullopt}, {}, {}, {}, {}, {}, {}};
    frame.add_image(disparity);

    EXPECT_FALSE(get_aux_3d_point(frame, Pixel{0, 0}, 100, 0.01));
}

TEST(create_bgr_from_ycbcr420, invalid_formats)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    const size_t width = 100;
    const size_t height = 100;

    std::vector<uint8_t> y_data(width * height, 42);
    std::vector<uint8_t> cbcr_data(width * height/2, 128);

    Image y_valid{std::make_shared<const std::vector<uint8_t>>(y_data),
                  0, width * height, Image::PixelFormat::MONO8,
                  static_cast<int>(width), static_cast<int>(height), {}, {}, DataSource::AUX_LUMA_RAW, aux_calibration};
    Image y_invalid{std::make_shared<const std::vector<uint8_t>>(y_data),
                  0, width * height, Image::PixelFormat::MONO16, // INVALID
                  static_cast<int>(width), static_cast<int>(height), {}, {}, DataSource::AUX_LUMA_RAW, aux_calibration};
    
    Image cbcr_valid{std::make_shared<const std::vector<uint8_t>>(cbcr_data),
                     0, width / 2 * height / 2, Image::PixelFormat::MONO16,
                     static_cast<int>(width/2), static_cast<int>(height/2), {}, {}, DataSource::AUX_CHROMA_RAW, aux_calibration};
    Image cbcr_invalid{std::make_shared<const std::vector<uint8_t>>(cbcr_data),
                     0, width / 2 * height / 2, Image::PixelFormat::MONO8, // INVALID
                     static_cast<int>(width/2), static_cast<int>(height/2), {}, {}, DataSource::AUX_CHROMA_RAW, aux_calibration};

    EXPECT_FALSE(create_bgr_from_ycbcr420(y_invalid, cbcr_valid, DataSource::AUX_RAW));
    EXPECT_FALSE(create_bgr_from_ycbcr420(y_valid, cbcr_invalid, DataSource::AUX_RAW));
}

TEST(create_bgr_image, not_ycbcr)
{
    ImageFrame frame{0, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    frame.aux_color_encoding = ColorImageEncoding::NONE;
    EXPECT_FALSE(create_bgr_image(frame, DataSource::AUX_RAW));
}

TEST(create_bgr_image, invalid_output_source)
{
    ImageFrame frame{0, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    frame.aux_color_encoding = ColorImageEncoding::YCBCR420;
    EXPECT_FALSE(create_bgr_image(frame, DataSource::LEFT_MONO_RAW));
}

TEST(create_bgr_image, missing_images)
{
    ImageFrame frame{0, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    frame.aux_color_encoding = ColorImageEncoding::YCBCR420;
    EXPECT_FALSE(create_bgr_image(frame, DataSource::AUX_RAW));
    EXPECT_FALSE(create_bgr_image(frame, DataSource::AUX_RECTIFIED_RAW));
}

TEST(create_bgr_image, valid_images)
{
    const float fx = 1000.0;
    const float fy = 1000.0;
    const float cx = 960.0;
    const float cy = 600.0;

    CameraCalibration aux_calibration{
        {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}},
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
        {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}},
        CameraCalibration::DistortionType::NONE,
        {}};

    const size_t width = 100;
    const size_t height = 100;

    std::vector<uint8_t> y_data(width * height, 42);
    std::vector<uint8_t> cbcr_data(width * height/2, 128);

    Image y_valid{std::make_shared<const std::vector<uint8_t>>(y_data),
                  0, width * height, Image::PixelFormat::MONO8,
                  static_cast<int>(width), static_cast<int>(height), {}, {}, DataSource::AUX_LUMA_RECTIFIED_RAW, aux_calibration};
    
    Image cbcr_valid{std::make_shared<const std::vector<uint8_t>>(cbcr_data),
                     0, width / 2 * height / 2, Image::PixelFormat::MONO16,
                     static_cast<int>(width/2), static_cast<int>(height/2), {}, {}, DataSource::AUX_CHROMA_RECTIFIED_RAW, aux_calibration};

    ImageFrame frame{0, {}, {}, StereoCalibration{aux_calibration, aux_calibration, aux_calibration}, {}, {}, {}, {}, {}, {}};
    frame.aux_color_encoding = ColorImageEncoding::YCBCR420;
    frame.add_image(y_valid);
    frame.add_image(cbcr_valid);

    EXPECT_TRUE(create_bgr_image(frame, DataSource::AUX_RECTIFIED_RAW));
}

