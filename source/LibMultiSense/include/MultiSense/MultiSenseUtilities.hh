/**
 * @file MultiSenseUtilities.hh
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
 *   2025-01-15, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "MultiSenseTypes.hh"

namespace multisense
{

///
/// Make sure our Points and point clouds are packed for applications which might need to handle
/// the underlying raw data
///
#pragma pack(push, 1)

template<typename Color>
struct Point
{
    float x = 0;
    float y = 0;
    float z = 0;
    Color color;
};

template<>
struct Point<void>
{
    float x = 0;
    float y = 0;
    float z = 0;
};

template<typename Color = void>
struct PointCloud
{
    std::vector<Point<Color>> cloud;
};

#pragma pack(pop)

///
/// @brief Convert a status object to a user readable string
///
MULTISENSE_API std::string to_string(const Status &status);

///
/// @brief Write a image to a specific path on disk. The type of serialization is determined by the
///        input path
///
MULTISENSE_API bool write_image(const Image &image, const std::filesystem::path &path);

///
/// @brief Create a depth image from a image frame
///
/// @param depth_format Supported formats include MONO16 and FLOAT32. Note MONO16 will be quantized to millimeters)
/// @param invalid_value The value to set invalid depth measurements to. (i.e. points where disparity = 0)
///
MULTISENSE_API std::optional<Image> create_depth_image(const ImageFrame &frame,
                                                       const Image::PixelFormat &depth_format,
                                                       const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW,
                                                       float invalid_value = 0);

///
/// @brief Convert a YCbCr420 luma + chroma image into a BGR color image
///
MULTISENSE_API std::optional<Image> create_bgr_image(const Image &luma, const Image &chroma, const DataSource &output_source);

///
/// @brief Convert a YCbCr420 luma + chroma image into a BGR color image
///
MULTISENSE_API std::optional<Image> create_bgr(const ImageFrame &frame, const DataSource &output_source);

///
/// @brief Create a point cloud from a image frame and a color source.
///
template<typename Color>
MULTISENSE_API std::optional<PointCloud<Color>> create_color_pointcloud(const Image &disparity,
                                                                         const std::optional<Image> &color,
                                                                         double max_range,
                                                                         const StereoCalibration &calibration)
{
    size_t color_step = 0;
    double color_disparity_scale = 0.0;

    if constexpr (std::is_same_v<Color, void>)
    {
        if (disparity.format != Image::PixelFormat::MONO16 || disparity.width < 0 || disparity.height < 0)
        {
            return std::nullopt;
        }
    }
    else
    {
        if (!color)
        {
            return std::nullopt;
        }

        color_step = sizeof(Color);

        if (disparity.format != Image::PixelFormat::MONO16 ||
            color->width != disparity.width ||
            color->height != disparity.height ||
            disparity.width < 0 ||
            disparity.height < 0)
        {
            return std::nullopt;
        }

        const double tx = calibration.right.P[0][3] / calibration.right.P[0][0];
        const double color_tx = color->calibration.P[0][3] / color->calibration.P[0][0];
        color_disparity_scale = color_tx / tx;
    }

    constexpr double scale = 1.0 / 16.0;

    const double squared_range = max_range * max_range;

    const double fx = disparity.calibration.P[0][0];
    const double fy = disparity.calibration.P[1][1];
    const double cx = disparity.calibration.P[0][2];
    const double cy = disparity.calibration.P[1][2];
    const double tx = calibration.right.P[0][3] / calibration.right.P[0][0];
    const double cx_prime = calibration.right.P[0][2];

    const double fytx = fy * tx;
    const double fxtx = fx * tx;

    const double fycxtx = fy * cx * tx;
    const double fxcytx = fx * cy * tx;
    const double fxfytx = fx * fy * tx;
    const double fycxcxprime = fy * (cx - cx_prime);

    PointCloud<Color> output;
    output.cloud.reserve(disparity.width * disparity.height);

    for (size_t h = 0 ; h < static_cast<size_t>(disparity.height) ; ++h)
    {
        for (size_t w = 0 ; w < static_cast<size_t>(disparity.width) ; ++w)
        {
            const size_t index = disparity.image_data_offset +
                                 (h * disparity.width * sizeof(uint16_t)) +
                                 (w * sizeof(uint16_t));

            const double d =
                static_cast<double>(*reinterpret_cast<const uint16_t*>(disparity.raw_data->data() + index)) * scale;

            if (d == 0.0)
            {
                continue;
            }

            const double inversebeta = 1.0 / (-fy * d + fycxcxprime);
            const double x = ((fytx * w) + (-fycxtx)) * inversebeta;
            const double y = ((fxtx * h) + (-fxcytx)) * inversebeta;
            const double z = fxfytx * inversebeta;

            if ((x*x + y*y + z*z) > squared_range)
            {
                continue;
            }

            if constexpr (std::is_same_v<Color, void>)
            {
                output.cloud.push_back(Point<Color>{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
            }
            else
            {
                //
                // Use the approximation that color_pixel_u = disp_u - (tx_color/ tx) * d
                //
                const size_t color_index = color->image_data_offset +
                                           (h * color->width * color_step) +
                                           static_cast<size_t>((static_cast<double>(w) - (color_disparity_scale * d))) * color_step;

                const Color color_pixel = *reinterpret_cast<const Color*>(color->raw_data->data() + color_index);

                output.cloud.push_back(Point<Color>{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
                                                    color_pixel});
            }
        }
    }

    return output;
}

///
/// @brief Create a point cloud from a image frame and a color source.
///
template<typename Color>
MULTISENSE_API std::optional<PointCloud<Color>> create_color_pointcloud(const ImageFrame &frame,
                                                                        double max_range,
                                                                        const DataSource &color_source = DataSource::UNKNOWN,
                                                                        const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW)
{
    if constexpr (std::is_same_v<Color, void>)
    {
        if (!frame.has_image(disparity_source))
        {
            return std::nullopt;
        }

        return create_color_pointcloud<Color>(frame.get_image(disparity_source), std::nullopt, max_range, frame.calibration);
    }
    else
    {
        if (!frame.has_image(color_source) || !frame.has_image(disparity_source))
        {
            return std::nullopt;
        }

        return create_color_pointcloud<Color>(frame.get_image(disparity_source),
                                              frame.get_image(color_source),
                                              max_range,
                                              frame.calibration);
    }
}

MULTISENSE_API std::optional<PointCloud<void>> create_pointcloud(const ImageFrame &frame,
                                                                 double max_range,
                                                                 const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW);

///
/// @brief Write a point cloud to a ASCII ply file
///
template <typename Color>
MULTISENSE_API bool write_pointcloud_ply(const PointCloud<Color> &point_cloud, const std::filesystem::path &path)
{
    std::ofstream ply(path, std::ios::binary);
    if (!ply.good())
    {
        return false;
    }

    std::ostringstream header;
    header << "ply\n";
    header << "format binary_little_endian 1.0\n";
    header << "element vertex " << point_cloud.cloud.size() << "\n";
    header << "property float x\n";
    header << "property float y\n";
    header << "property float z\n";

    if constexpr (std::is_same_v<Color, uint8_t>)
    {
        header << "property uchar intensity\n";
    }
    else if constexpr (std::is_same_v<Color, uint16_t>)
    {
        header << "property ushort intensity\n";
    }
    else if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
    {
        header << "property uchar red\n";
        header << "property uchar green\n";
        header << "property uchar blue\n";
    }
    else if (!std::is_same_v<Color, void>)
    {
        throw std::runtime_error("Unsupported color type");
    }

    header << "end_header\n";

    std::string header_str = header.str();
    ply.write(header_str.c_str(), header_str.size());

    for (const auto &point : point_cloud.cloud)
    {
        ply.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
        ply.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
        ply.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));

        if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
        {
            uint8_t red   = point.color[2];
            uint8_t green = point.color[1];
            uint8_t blue  = point.color[0];
            ply.write(reinterpret_cast<const char*>(&red), sizeof(red));
            ply.write(reinterpret_cast<const char*>(&green), sizeof(green));
            ply.write(reinterpret_cast<const char*>(&blue), sizeof(blue));
        }
        else if constexpr (std::is_same_v<Color, void>)
        {
            // No color data to write.
        }
        else
        {
            // Write the color value directly (works for uint8_t or uint16_t).
            ply.write(reinterpret_cast<const char*>(&point.color), sizeof(point.color));
        }
    }

    return true;
}

}
