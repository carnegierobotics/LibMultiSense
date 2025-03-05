/**
 * @file utilities.cc
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

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>

#else
#include <arpa/inet.h>
#endif

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifdef HAVE_OPENCV
#include <opencv2/imgcodecs.hpp>
#endif

#include "MultiSense/MultiSenseUtilities.hh"

namespace multisense {
namespace {

#ifndef HAVE_OPENCV
bool write_binary_image(const Image &image, const std::filesystem::path &path)
{
    std::ofstream output(path, std::ios::binary | std::ios::out);

    if (!output.good())
    {
        std::cerr << "Failed to open: " << path << std::endl;
        return false;
    }

    switch(image.format)
    {
        case Image::PixelFormat::MONO8:
        {

            output << "P5\n"
                   << image.width << " " << image.height << "\n"
                   << 0xFF << "\n";

            output.write(reinterpret_cast<const char*>(image.raw_data->data()) + image.image_data_offset,
                         image.image_data_length);
            break;
        }
        case Image::PixelFormat::MONO16:
        {
            output << "P5\n"
                   << image.width << " " << image.height << "\n"
                   << 0xFFFF << "\n";

            //
            // Make sure we swap our byte order if needed
            //
            const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(image.raw_data->data() + image.image_data_offset);
            for (int i = 0 ; i < (image.width * image.height) ; ++i)
            {
                const uint16_t o = htons(raw_data[i]);
                output.write(reinterpret_cast<const char*>(&o), sizeof(uint16_t));
            }

            break;
        }
        case Image::PixelFormat::BGR8:
        {
            output << "P6\n"
                   << image.width << " " << image.height << "\n"
                   << 0xFF << "\n";

            //
            // Convert BGR to RGB
            //
            for (int i = 0 ; i < (image.width * image.height) ; ++i)
            {
                const auto bgr = reinterpret_cast<const std::array<uint8_t, 3>*>(image.raw_data->data() + image.image_data_offset + (i * 3));
                const std::array<uint8_t, 3> rgb{bgr->at(2), bgr->at(1), bgr->at(0)};
                output.write(reinterpret_cast<const char*>(rgb.data()), sizeof(rgb));
            }

            break;
        }
        default:
        {
            std::cerr << "Unhandled image format. Cannot write to disk" << std::endl;
            return false;
        }
    }

    output.close();
    return true;
}
#endif

}

std::string to_string(const Status &status)
{
    switch(status)
    {
        case Status::OK: {return "OK";}
        case Status::TIMEOUT: {return "TIMEOUT";}
        case Status::INTERNAL_ERROR: {return "ERROR";}
        case Status::FAILED: {return "FAILED";}
        case Status::UNSUPPORTED: {return "UNSUPPORTED";}
        case Status::UNKNOWN: {return "UNKNOWN";}
        case Status::EXCEPTION: {return "EXCEPTION";}
        case Status::UNINITIALIZED: {return "UNINITIALIZED";}
        default: {return "UNKNOWN";}
    }
}


#ifdef HAVE_OPENCV
cv::Mat Image::cv_mat() const
{
    int cv_type = 0;
    switch(format)
    {
        case Image::PixelFormat::MONO8: {cv_type = CV_8UC1; break;}
        case Image::PixelFormat::BGR8: {cv_type = CV_8UC3; break;}
        case Image::PixelFormat::MONO16: {cv_type = CV_16UC1; break;}
        case Image::PixelFormat::FLOAT32: {cv_type = CV_32FC1; break;}
        default: {throw std::runtime_error("invalid pixel format");}
    }

    return cv::Mat{height,
                   width,
                   cv_type,
                   const_cast<uint8_t*>(raw_data->data() + image_data_offset)};
}
#endif

bool write_image(const Image &image, const std::filesystem::path &path)
{
#ifdef HAVE_OPENCV
    return cv::imwrite(path.string(), image.cv_mat());
#else
    const auto extension = path.extension();
    if (extension == ".pgm" || extension == ".PGM" || extension == ".ppm" || extension == ".PPM")
    {
        return write_binary_image(image, path);
    }
    throw std::runtime_error("Unsupported path extension: " + extension.string() + ". Try compiling with OpenCV");
#endif
    return false;
}

std::optional<Image> create_depth_image(const ImageFrame &frame,
                                        const Image::PixelFormat &depth_format,
                                        const DataSource &disparity_source,
                                        float invalid_value)
{
    if (!frame.has_image(disparity_source))
    {
        return std::nullopt;
    }

    const auto disparity = frame.get_image(disparity_source);

    if (disparity.format != Image::PixelFormat::MONO16 ||
        disparity.width < 0 ||
        disparity.height < 0)
    {
        return std::nullopt;
    }

    const double fx = disparity.calibration.P[0][0];
    const double tx = frame.calibration.right.P[0][3] / frame.calibration.right.P[0][0];

    size_t bytes_per_pixel = 0;
    switch (depth_format)
    {
        case Image::PixelFormat::MONO16:
        {
            bytes_per_pixel = sizeof(uint16_t);
            break;
        }
        case Image::PixelFormat::FLOAT32:
        {
            bytes_per_pixel = sizeof(float);
            break;
        }
        default:
        {
            std::cerr << "Unsupported depth pixel format" << std::endl;
            return std::nullopt;
        }
    }

    auto data = std::make_shared<std::vector<uint8_t>>(disparity.width * disparity.height * bytes_per_pixel,
                                                       static_cast<uint8_t>(0));

    //
    // MONO16 disparity images are quantized to 1/16th of a pixel
    //
    constexpr double scale = 1.0 / 16.0;

    for (size_t i = 0 ; i < static_cast<size_t>(disparity.width * disparity.height) ; ++i)
    {
        const size_t index = disparity.image_data_offset + (i * sizeof(uint16_t));

        const double d =
            static_cast<double>(*reinterpret_cast<const uint16_t*>(disparity.raw_data->data() + index)) * scale;

        switch (depth_format)
        {
            case Image::PixelFormat::MONO16:
            {
                //
                // Quantize to millimeters
                //
                const uint16_t depth = (d == 0.0) ? static_cast<uint16_t>(invalid_value) :
                                                    static_cast<uint16_t>(1000 * fx * -tx / d);

                auto data_pointer = reinterpret_cast<uint16_t*>(data->data() + (sizeof(uint16_t) * i));
                *data_pointer = depth;
                continue;
            }
            case Image::PixelFormat::FLOAT32:
            {
                const float depth = (d == 0.0) ? invalid_value :
                                                 static_cast<float>(fx * -tx / d);

                auto data_pointer = reinterpret_cast<float*>(data->data() + (sizeof(float) * i));
                *data_pointer = depth;
                continue;
            }
            default:
            {
                std::cerr << "Unsupported depth pixel format" << std::endl;
                return std::nullopt;
            }
        }
    }

    return Image{data,
                 0,
                 data->size(),
                 depth_format,
                 disparity.width,
                 disparity.height,
                 disparity.camera_timestamp,
                 disparity.ptp_timestamp,
                 disparity.source,
                 disparity.calibration};
}


std::optional<Image> create_bgr_from_ycbcr420(const Image &luma, const Image &chroma, const DataSource &output_source)
{
    if (luma.format != Image::PixelFormat::MONO8 || chroma.format != Image::PixelFormat::MONO16)
    {
        return std::nullopt;
    }

    const size_t color_length = luma.image_data_length * 3;

    std::vector<uint8_t> raw_data(color_length, static_cast<uint8_t>(0));

    for (int h = 0 ; h < luma.height ; ++h)
    {
        const size_t row_offset = h * luma.width * 3;

        for (int w = 0 ; w < luma.width ; ++w)
        {
            const size_t luma_offset = (h * luma.width) + w;
            const size_t chroma_offset = 2 * (((h / 2) * (luma.width / 2)) + (w / 2));

            const float px_y = static_cast<float>(*(luma.raw_data->data() + luma.image_data_offset + luma_offset));
            const float px_cb = static_cast<float>(*(chroma.raw_data->data() + chroma.image_data_offset + chroma_offset)) - 128.0f;
            const float px_cr = static_cast<float>(*(chroma.raw_data->data() + chroma.image_data_offset + chroma_offset + 1)) - 128.0f;

            const float px_r = std::clamp(px_y + 1.13983f * px_cr, 0.0f, 255.0f);
            const float px_g = std::clamp(px_y - 0.39465f * px_cb - 0.58060f * px_cr, 0.0f, 255.0f);
            const float px_b = std::clamp(px_y + 2.03211f * px_cb, 0.0f, 255.0f);

            auto bgr_pixel_ptr = reinterpret_cast<uint8_t*>(raw_data.data() + row_offset + (3 * w));

            bgr_pixel_ptr[0] = static_cast<uint8_t>(px_b);
            bgr_pixel_ptr[1] = static_cast<uint8_t>(px_g);
            bgr_pixel_ptr[2] = static_cast<uint8_t>(px_r);
        }
    }

    return Image{std::make_shared<std::vector<uint8_t>>(std::move(raw_data)),
                 0,
                 color_length,
                 Image::PixelFormat::BGR8,
                 luma.width,
                 luma.height,
                 luma.camera_timestamp,
                 luma.ptp_timestamp,
                 output_source,
                 luma.calibration};
}

std::optional<Image> create_bgr_image(const ImageFrame &frame, const DataSource &output_source)
{
    if (frame.aux_color_encoding == ColorImageEncoding::YCBCR420)
    {
        DataSource luma_source = DataSource::UNKNOWN;
        DataSource chroma_source = DataSource::UNKNOWN;
        switch (output_source)
        {
            case DataSource::AUX_RAW:
            {
                luma_source = DataSource::AUX_LUMA_RAW;
                chroma_source = DataSource::AUX_CHROMA_RAW;
                break;
            }
            case DataSource::AUX_RECTIFIED_RAW:
            {
                luma_source = DataSource::AUX_LUMA_RECTIFIED_RAW;
                chroma_source = DataSource::AUX_CHROMA_RECTIFIED_RAW;
                break;
            }
            default: {return std::nullopt;}
        }

        if (frame.has_image(luma_source) && frame.has_image(chroma_source))
        {
            return create_bgr_from_ycbcr420(frame.get_image(luma_source), frame.get_image(chroma_source), output_source);
        }
    }

    return std::nullopt;
}


std::optional<PointCloud<void>> create_pointcloud(const ImageFrame &frame,
                                                  double max_range,
                                                  const DataSource &disparity_source)
{
    return create_color_pointcloud<void>(frame, max_range, DataSource::UNKNOWN, disparity_source);
}

}
