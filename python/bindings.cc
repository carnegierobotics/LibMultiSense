/**
 * @file bindings.cc
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
 *   2025-01-19, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#ifdef BUILD_JSON
#include <MultiSense/MultiSenseSerialization.hh>
#endif

#ifdef BUILD_JSON
#define PYBIND11_JSON_SUPPORT(Type)                                                     \
    .def(py::init([](const py::dict &d) {                                               \
        py::module json = py::module::import("json");                                   \
        py::object json_str = json.attr("dumps")(d);                                    \
        const nlohmann::json j = nlohmann::json::parse(json_str.cast<std::string>());   \
        Type t{};                                                                       \
        j.get_to(t);                                                                    \
        return t;                                                                       \
    }))                                                                                 \
    .def_property_readonly("json", [](const Type &obj) {                                \
        const nlohmann::json j = obj;                                                   \
        py::module json = py::module::import("json");                                   \
        py::object result = json.attr("loads")(j.dump());                               \
        return result.cast<py::dict>();                                                 \
    })                                                                                  \
    .def("__repr__", [](const Type &obj) {                                              \
        const nlohmann::json j = obj;                                                   \
        return j.dump();                                                                \
    })
#else
#define PYBIND11_JSON_SUPPORT(Type)
#endif

namespace py = pybind11;

PYBIND11_MODULE(_libmultisense, m) {
    m.doc() = "Pybind11 bindings for the LibMultiSense C++ Library";

    // Status
    py::enum_<multisense::Status>(m, "Status")
        .value("OK", multisense::Status::OK)
        .value("TIMEOUT", multisense::Status::TIMEOUT)
        .value("INTERNAL_ERROR", multisense::Status::INTERNAL_ERROR)
        .value("FAILED", multisense::Status::FAILED)
        .value("UNSUPPORTED", multisense::Status::UNSUPPORTED)
        .value("UNKNOWN", multisense::Status::UNKNOWN)
        .value("EXCEPTION", multisense::Status::EXCEPTION)
        .value("UNINITIALIZED", multisense::Status::UNINITIALIZED)
        .value("INCOMPLETE_APPLICATION", multisense::Status::INCOMPLETE_APPLICATION);

    // DataSource
    py::enum_<multisense::DataSource>(m, "DataSource")
        .value("UNKNOWN", multisense::DataSource::UNKNOWN)
        .value("ALL", multisense::DataSource::ALL)
        .value("LEFT_MONO_RAW", multisense::DataSource::LEFT_MONO_RAW)
        .value("RIGHT_MONO_RAW", multisense::DataSource::RIGHT_MONO_RAW)
        .value("LEFT_MONO_COMPRESSED", multisense::DataSource::LEFT_MONO_COMPRESSED)
        .value("RIGHT_MONO_COMPRESSED", multisense::DataSource::RIGHT_MONO_COMPRESSED)
        .value("LEFT_RECTIFIED_RAW", multisense::DataSource::LEFT_RECTIFIED_RAW)
        .value("RIGHT_RECTIFIED_RAW", multisense::DataSource::RIGHT_RECTIFIED_RAW)
        .value("LEFT_RECTIFIED_COMPRESSED", multisense::DataSource::LEFT_RECTIFIED_COMPRESSED)
        .value("RIGHT_RECTIFIED_COMPRESSED", multisense::DataSource::RIGHT_RECTIFIED_COMPRESSED)
        .value("LEFT_DISPARITY_RAW", multisense::DataSource::LEFT_DISPARITY_RAW)
        .value("LEFT_DISPARITY_COMPRESSED", multisense::DataSource::LEFT_DISPARITY_COMPRESSED)
        .value("AUX_COMPRESSED", multisense::DataSource::AUX_COMPRESSED)
        .value("AUX_RECTIFIED_COMPRESSED", multisense::DataSource::AUX_RECTIFIED_COMPRESSED)
        .value("AUX_LUMA_RAW", multisense::DataSource::AUX_LUMA_RAW)
        .value("AUX_LUMA_RECTIFIED_RAW", multisense::DataSource::AUX_LUMA_RECTIFIED_RAW)
        .value("AUX_CHROMA_RAW", multisense::DataSource::AUX_CHROMA_RAW)
        .value("AUX_CHROMA_RECTIFIED_RAW", multisense::DataSource::AUX_CHROMA_RECTIFIED_RAW)
        .value("AUX_RAW", multisense::DataSource::AUX_RAW)
        .value("AUX_RECTIFIED_RAW", multisense::DataSource::AUX_RECTIFIED_RAW)
        .value("COST_RAW", multisense::DataSource::COST_RAW);

    // ColorImageEncoding
    py::enum_<multisense::ColorImageEncoding>(m, "ColorImageEncoding")
        .value("NONE", multisense::ColorImageEncoding::NONE)
        .value("YCBCR420", multisense::ColorImageEncoding::YCBCR420);

    // CameraCalibration::DistortionType
    py::enum_<multisense::CameraCalibration::DistortionType>(m, "DistortionType")
        .value("NONE", multisense::CameraCalibration::DistortionType::NONE)
        .value("PLUMBBOB", multisense::CameraCalibration::DistortionType::PLUMBBOB)
        .value("RATIONAL_POLYNOMIAL", multisense::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL);

    // CameraCalibration
    py::class_<multisense::CameraCalibration>(m, "CameraCalibration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::CameraCalibration)
        .def_readwrite("K", &multisense::CameraCalibration::K)
        .def_readwrite("R", &multisense::CameraCalibration::R)
        .def_readwrite("P", &multisense::CameraCalibration::P)
        .def_readwrite("distortion_type", &multisense::CameraCalibration::distortion_type)
        .def_readwrite("D", &multisense::CameraCalibration::D);

    // StereoCalibration
    py::class_<multisense::StereoCalibration>(m, "StereoCalibration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::StereoCalibration)
        .def_readwrite("left", &multisense::StereoCalibration::left)
        .def_readwrite("right", &multisense::StereoCalibration::right)
        .def_readwrite("aux", &multisense::StereoCalibration::aux);

    // Image::PixelFormat
    py::enum_<multisense::Image::PixelFormat>(m, "PixelFormat")
        .value("UNKNOWN", multisense::Image::PixelFormat::UNKNOWN)
        .value("MONO8", multisense::Image::PixelFormat::MONO8)
        .value("BGR8", multisense::Image::PixelFormat::BGR8)
        .value("MONO16", multisense::Image::PixelFormat::MONO16)
        .value("FLOAT32", multisense::Image::PixelFormat::FLOAT32)
        .value("JPEG", multisense::Image::PixelFormat::JPEG)
        .value("H264", multisense::Image::PixelFormat::H264);

    // Image
    py::class_<multisense::Image>(m, "Image")
        .def(py::init<>())
        .def_property_readonly("as_array", [](const multisense::Image& image)
        {

            std::vector<size_t> shape = {static_cast<size_t>(image.height), static_cast<size_t>(image.width)};
            std::vector<size_t> strides;
            size_t element_size = 0;
            std::string format;

            switch (image.format)
            {
                case multisense::Image::PixelFormat::MONO8:
                {
                    element_size = sizeof(uint8_t);
                    format = py::format_descriptor<uint8_t>::format();
                    strides = {sizeof(uint8_t) * image.width, sizeof(uint8_t)};
                    break;
                }
                case multisense::Image::PixelFormat::MONO16:
                {
                    element_size = sizeof(uint16_t);
                    format = py::format_descriptor<uint16_t>::format();
                    strides = {sizeof(uint16_t) * image.width, sizeof(uint16_t)};
                    break;
                }
                case multisense::Image::PixelFormat::BGR8:
                {
                    element_size = sizeof(uint8_t);
                    format = py::format_descriptor<uint8_t>::format();
                    shape.push_back(3);
                    strides = {sizeof(uint8_t) * image.width * 3, sizeof(uint8_t) * 3, sizeof(uint8_t)};
                    break;
                }
                case multisense::Image::PixelFormat::FLOAT32:
                {
                    element_size = sizeof(float);
                    format = py::format_descriptor<float>::format();
                    strides = {sizeof(float) * image.width, sizeof(float)};
                    break;
                }
                default: {throw std::runtime_error("Unknown pixel format");}
            }

            // Map the cv::Mat to a NumPy array without copying the data
            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(image.raw_data->data() + image.image_data_offset),
                             element_size,
                             format,
                             shape.size(),
                             shape,
                             strides));
        })
        .def_readonly("format", &multisense::Image::format)
        .def_readonly("width", &multisense::Image::width)
        .def_readonly("height", &multisense::Image::height)
        .def_readonly("camera_timestamp", &multisense::Image::camera_timestamp)
        .def_readonly("ptp_timestamp", &multisense::Image::ptp_timestamp)
        .def_readonly("source", &multisense::Image::source)
        .def_readonly("calibration", &multisense::Image::calibration);

    // ImageFrame
    py::class_<multisense::ImageFrame>(m, "ImageFrame")
        .def(py::init<>())
        .def("add_image", &multisense::ImageFrame::add_image)
        .def("get_image", &multisense::ImageFrame::get_image)
        .def("has_image", &multisense::ImageFrame::has_image)
        .def_readonly("frame_id", &multisense::ImageFrame::frame_id)
        .def_readonly("images", &multisense::ImageFrame::images)
        .def_readonly("calibration", &multisense::ImageFrame::calibration)
        .def_readonly("frame_time", &multisense::ImageFrame::frame_time)
        .def_readonly("ptp_frame_time", &multisense::ImageFrame::ptp_frame_time)
        .def_readonly("aux_color_encoding", &multisense::ImageFrame::aux_color_encoding);

    // ImuRate
    py::class_<multisense::ImuRate>(m, "ImuRate")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::ImuRate)
        .def(py::self == py::self)
        .def_readwrite("sample_rate", &multisense::ImuRate::sample_rate)
        .def_readwrite("bandwith_cutoff", &multisense::ImuRate::bandwith_cutoff);

    // ImuRange
    py::class_<multisense::ImuRange>(m, "ImuRange")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::ImuRange)
        .def(py::self == py::self)
        .def_readwrite("range", &multisense::ImuRange::range)
        .def_readwrite("resolution", &multisense::ImuRange::resolution);


    // MultiSenseConfig::StereoCalibration
    py::class_<multisense::MultiSenseConfig::StereoConfig>(m, "StereoConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::StereoConfig)
        .def(py::self == py::self)
        .def_readwrite("postfilter_strength", &multisense::MultiSenseConfig::StereoConfig::postfilter_strength);

    // MultiSenseConfig::ManualExposureConfig
    py::class_<multisense::MultiSenseConfig::ManualExposureConfig>(m, "ManualExposureConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::ManualExposureConfig)
        .def(py::self == py::self)
        .def_readwrite("gain", &multisense::MultiSenseConfig::ManualExposureConfig::gain)
        .def_readwrite("exposure_time", &multisense::MultiSenseConfig::ManualExposureConfig::exposure_time);

    // MultiSenseConfig::AutoExposureRoiConfig
    py::class_<multisense::MultiSenseConfig::AutoExposureRoiConfig>(m, "AutoExposureRoiConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::AutoExposureRoiConfig)
        .def(py::self == py::self)
        .def_readwrite("top_left_x_position", &multisense::MultiSenseConfig::AutoExposureRoiConfig::top_left_x_position)
        .def_readwrite("top_left_y_position", &multisense::MultiSenseConfig::AutoExposureRoiConfig::top_left_y_position)
        .def_readwrite("width", &multisense::MultiSenseConfig::AutoExposureRoiConfig::width)
        .def_readwrite("height", &multisense::MultiSenseConfig::AutoExposureRoiConfig::height);

    // MultiSenseConfig::AutoExposureConfig
    py::class_<multisense::MultiSenseConfig::AutoExposureConfig>(m, "AutoExposureConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::AutoExposureConfig)
        .def(py::self == py::self)
        .def_readwrite("max_exposure_time", &multisense::MultiSenseConfig::AutoExposureConfig::max_exposure_time)
        .def_readwrite("decay", &multisense::MultiSenseConfig::AutoExposureConfig::decay)
        .def_readwrite("target_intensity", &multisense::MultiSenseConfig::AutoExposureConfig::target_intensity)
        .def_readwrite("target_threshold", &multisense::MultiSenseConfig::AutoExposureConfig::target_threshold)
        .def_readwrite("max_gain", &multisense::MultiSenseConfig::AutoExposureConfig::max_gain);

    // MultiSenseConfig::ManualWhiteBalanceConfig
    py::class_<multisense::MultiSenseConfig::ManualWhiteBalanceConfig>(m, "ManualWhiteBalanceConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::ManualWhiteBalanceConfig)
        .def(py::self == py::self)
        .def_readwrite("red", &multisense::MultiSenseConfig::ManualWhiteBalanceConfig::red)
        .def_readwrite("blue", &multisense::MultiSenseConfig::ManualWhiteBalanceConfig::blue);

    // MultiSenseConfig::AutoWhiteBalanceConfig
    py::class_<multisense::MultiSenseConfig::AutoWhiteBalanceConfig>(m, "AutoWhiteBalanceConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::AutoWhiteBalanceConfig)
        .def(py::self == py::self)
        .def_readwrite("decay", &multisense::MultiSenseConfig::AutoWhiteBalanceConfig::decay)
        .def_readwrite("threshold", &multisense::MultiSenseConfig::AutoWhiteBalanceConfig::threshold);

    // MultiSenseConfig::ImageConfig
    py::class_<multisense::MultiSenseConfig::ImageConfig>(m, "ImageConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::ImageConfig)
        .def(py::self == py::self)
        .def_readwrite("gamma", &multisense::MultiSenseConfig::ImageConfig::gamma)
        .def_readwrite("auto_exposure_enabled", &multisense::MultiSenseConfig::ImageConfig::auto_exposure_enabled)
        .def_readwrite("manual_exposure", &multisense::MultiSenseConfig::ImageConfig::manual_exposure)
        .def_readwrite("auto_exposure", &multisense::MultiSenseConfig::ImageConfig::auto_exposure)
        .def_readwrite("auto_white_balance_enabled", &multisense::MultiSenseConfig::ImageConfig::auto_white_balance_enabled)
        .def_readwrite("manual_white_balance", &multisense::MultiSenseConfig::ImageConfig::manual_white_balance)
        .def_readwrite("auto_white_balance", &multisense::MultiSenseConfig::ImageConfig::auto_white_balance);

    // MultiSenseConfig::AuxConfig
    py::class_<multisense::MultiSenseConfig::AuxConfig>(m, "AuxConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::AuxConfig)
        .def(py::self == py::self)
        .def_readwrite("image_config", &multisense::MultiSenseConfig::AuxConfig::image_config)
        .def_readwrite("sharpening_enabled", &multisense::MultiSenseConfig::AuxConfig::sharpening_enabled)
        .def_readwrite("sharpening_percentage", &multisense::MultiSenseConfig::AuxConfig::sharpening_percentage)
        .def_readwrite("sharpening_limit", &multisense::MultiSenseConfig::AuxConfig::sharpening_limit);

    // MultiSenseConfig::MaxDisparities
    py::enum_<multisense::MultiSenseConfig::MaxDisparities>(m, "MaxDisparities")
        .value("D64", multisense::MultiSenseConfig::MaxDisparities::D64)
        .value("D128", multisense::MultiSenseConfig::MaxDisparities::D128)
        .value("D256", multisense::MultiSenseConfig::MaxDisparities::D256);

    // MultiSenseConfig::TimeConfig
    py::class_<multisense::MultiSenseConfig::TimeConfig>(m, "TimeConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::TimeConfig)
        .def(py::self == py::self)
        .def_readwrite("ptp_enabled", &multisense::MultiSenseConfig::TimeConfig::ptp_enabled);

    // MultiSenseConfig::NetworkTransmissionConfig
    py::class_<multisense::MultiSenseConfig::NetworkTransmissionConfig>(m, "NetworkTransmissionConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::NetworkTransmissionConfig)
        .def(py::self == py::self)
        .def_readwrite("packet_delay_enabled", &multisense::MultiSenseConfig::NetworkTransmissionConfig::packet_delay_enabled);

    // MultiSenseConfig::ImuConfig::OperatingMode
    py::class_<multisense::MultiSenseConfig::ImuConfig::OperatingMode>(m, "ImuOperatingMode")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::ImuConfig::OperatingMode)
        .def(py::self == py::self)
        .def_readwrite("enabled", &multisense::MultiSenseConfig::ImuConfig::OperatingMode::enabled)
        .def_readwrite("rate", &multisense::MultiSenseConfig::ImuConfig::OperatingMode::rate)
        .def_readwrite("range", &multisense::MultiSenseConfig::ImuConfig::OperatingMode::range);

    // MultiSenseConfig::ImuConfig
    py::class_<multisense::MultiSenseConfig::ImuConfig>(m, "ImuConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::ImuConfig)
        .def(py::self == py::self)
        .def_readwrite("samples_per_frame", &multisense::MultiSenseConfig::ImuConfig::samples_per_frame)
        .def_readwrite("accelerometer", &multisense::MultiSenseConfig::ImuConfig::accelerometer)
        .def_readwrite("gyroscope", &multisense::MultiSenseConfig::ImuConfig::gyroscope)
        .def_readwrite("magnetometer", &multisense::MultiSenseConfig::ImuConfig::magnetometer);

    // MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode
    py::enum_<multisense::MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode>(m, "FlashMode")
        .value("NONE", multisense::MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::NONE)
        .value("SYNC_WITH_MAIN_STEREO", multisense::MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_MAIN_STEREO)
        .value("SYNC_WITH_AUX", multisense::MultiSenseConfig::LightingConfig::ExternalConfig::FlashMode::SYNC_WITH_AUX);

    // MultiSenseConfig::LightingConfig::InternalConfig
    py::class_<multisense::MultiSenseConfig::LightingConfig::InternalConfig>(m, "LightingConfigInternalConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::LightingConfig::InternalConfig)
        .def(py::self == py::self)
        .def_readwrite("intensity", &multisense::MultiSenseConfig::LightingConfig::InternalConfig::intensity)
        .def_readwrite("flash", &multisense::MultiSenseConfig::LightingConfig::InternalConfig::flash);

    // MultiSenseConfig::LightingConfig::ExternalConfig
    py::class_<multisense::MultiSenseConfig::LightingConfig::ExternalConfig>(m, "LightingConfigExternalConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::LightingConfig::ExternalConfig)
        .def(py::self == py::self)
        .def_readwrite("intensity", &multisense::MultiSenseConfig::LightingConfig::ExternalConfig::intensity)
        .def_readwrite("flash", &multisense::MultiSenseConfig::LightingConfig::ExternalConfig::flash)
        .def_readwrite("pulses_per_exposure", &multisense::MultiSenseConfig::LightingConfig::ExternalConfig::pulses_per_exposure)
        .def_readwrite("startup_time", &multisense::MultiSenseConfig::LightingConfig::ExternalConfig::startup_time);

    // MultiSenseConfig::LightingConfig
    py::class_<multisense::MultiSenseConfig::LightingConfig>(m, "LightingConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig::LightingConfig)
        .def(py::self == py::self)
        .def(py::self == py::self)
        .def_readwrite("internal", &multisense::MultiSenseConfig::LightingConfig::internal)
        .def_readwrite("external", &multisense::MultiSenseConfig::LightingConfig::external);

    // MultiSenseConfig
    py::class_<multisense::MultiSenseConfig>(m, "MultiSenseConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfig)
        .def(py::self == py::self)
        .def_readwrite("width", &multisense::MultiSenseConfig::width)
        .def_readwrite("height", &multisense::MultiSenseConfig::height)
        .def_readwrite("disparities", &multisense::MultiSenseConfig::disparities)
        .def_readwrite("frames_per_second", &multisense::MultiSenseConfig::frames_per_second)
        .def_readwrite("stereo_config", &multisense::MultiSenseConfig::stereo_config)
        .def_readwrite("image_config", &multisense::MultiSenseConfig::image_config)
        .def_readwrite("aux_config", &multisense::MultiSenseConfig::aux_config)
        .def_readwrite("time_config", &multisense::MultiSenseConfig::time_config)
        .def_readwrite("network_config", &multisense::MultiSenseConfig::network_config)
        .def_readwrite("imu_config", &multisense::MultiSenseConfig::imu_config)
        .def_readwrite("lighting_config", &multisense::MultiSenseConfig::lighting_config);

    // MultiSenseStatus::PtpStatus
    py::class_<multisense::MultiSenseStatus::PtpStatus>(m, "PtpStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::PtpStatus)
        .def_readwrite("grandmaster_present", &multisense::MultiSenseStatus::PtpStatus::grandmaster_present)
        .def_readwrite("grandmaster_id", &multisense::MultiSenseStatus::PtpStatus::grandmaster_id)
        .def_readwrite("grandmaster_offset", &multisense::MultiSenseStatus::PtpStatus::grandmaster_offset)
        .def_readwrite("path_delay", &multisense::MultiSenseStatus::PtpStatus::path_delay)
        .def_readwrite("steps_from_local_to_grandmaster", &multisense::MultiSenseStatus::PtpStatus::steps_from_local_to_grandmaster);

    // MultiSenseStatus::CameraStatus
    py::class_<multisense::MultiSenseStatus::CameraStatus>(m, "CameraStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::CameraStatus)
        .def_readwrite("cameras_ok", &multisense::MultiSenseStatus::CameraStatus::cameras_ok)
        .def_readwrite("processing_pipeline_ok", &multisense::MultiSenseStatus::CameraStatus::processing_pipeline_ok);

    // MultiSenseStatus::TemperatureStatus
    py::class_<multisense::MultiSenseStatus::TemperatureStatus>(m, "TemperatureStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::TemperatureStatus)
        .def_readwrite("fpga_temperature", &multisense::MultiSenseStatus::TemperatureStatus::fpga_temperature)
        .def_readwrite("left_imager_temperature", &multisense::MultiSenseStatus::TemperatureStatus::left_imager_temperature)
        .def_readwrite("right_imager_temperature", &multisense::MultiSenseStatus::TemperatureStatus::right_imager_temperature)
        .def_readwrite("power_supply_temperature", &multisense::MultiSenseStatus::TemperatureStatus::power_supply_temperature);

    // MultiSenseStatus::PowerStatus
    py::class_<multisense::MultiSenseStatus::PowerStatus>(m, "PowerStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::PowerStatus)
        .def_readwrite("input_voltage", &multisense::MultiSenseStatus::PowerStatus::input_voltage)
        .def_readwrite("input_current", &multisense::MultiSenseStatus::PowerStatus::input_current)
        .def_readwrite("fpga_power", &multisense::MultiSenseStatus::PowerStatus::fpga_power);

    // MultiSenseStatus::ClientNetworkStatus
    py::class_<multisense::MultiSenseStatus::ClientNetworkStatus>(m, "ClientNetworkStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::ClientNetworkStatus)
        .def_readwrite("received_messages", &multisense::MultiSenseStatus::ClientNetworkStatus::received_messages)
        .def_readwrite("dropped_messages", &multisense::MultiSenseStatus::ClientNetworkStatus::dropped_messages)
        .def_readwrite("invalid_packets", &multisense::MultiSenseStatus::ClientNetworkStatus::invalid_packets);

    // MultiSenseStatus::TimeStatus
    py::class_<multisense::MultiSenseStatus::TimeStatus>(m, "TimeStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::TimeStatus)
        .def_readwrite("camera_time", &multisense::MultiSenseStatus::TimeStatus::camera_time)
        .def_readwrite("client_host_time", &multisense::MultiSenseStatus::TimeStatus::client_host_time)
        .def_readwrite("network_delay", &multisense::MultiSenseStatus::TimeStatus::network_delay)
        .def("offset_to_host", &multisense::MultiSenseStatus::TimeStatus::offset_to_host)
        .def("apply_offset_to_host", &multisense::MultiSenseStatus::TimeStatus::apply_offset_to_host);

    // MultiSenseStatus
    py::class_<multisense::MultiSenseStatus>(m, "MultiSenseStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus)
        .def_readwrite("system_ok", &multisense::MultiSenseStatus::system_ok)
        .def_readwrite("ptp", &multisense::MultiSenseStatus::ptp)
        .def_readwrite("camera", &multisense::MultiSenseStatus::camera)
        .def_readwrite("temperature", &multisense::MultiSenseStatus::temperature)
        .def_readwrite("power", &multisense::MultiSenseStatus::power)
        .def_readwrite("client_network", &multisense::MultiSenseStatus::client_network)
        .def_readwrite("time", &multisense::MultiSenseStatus::time);

    // MultiSenseInfo::DeviceInfo::PcbInfo
    py::class_<multisense::MultiSenseInfo::DeviceInfo::PcbInfo>(m, "PcbInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::DeviceInfo::PcbInfo)
        .def_readwrite("name", &multisense::MultiSenseInfo::DeviceInfo::PcbInfo::name)
        .def_readwrite("revision", &multisense::MultiSenseInfo::DeviceInfo::PcbInfo::revision);

    // MultiSenseInfo::DeviceInfo::HardwareRevision
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::HardwareRevision>(m, "HardwareRevision")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::UNKNOWN)
        .value("S7", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S7)
        .value("S21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S21)
        .value("ST21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::ST21)
        .value("S27", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S27)
        .value("S30", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S30)
        .value("KS21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21)
        .value("MONOCAM", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::MONOCAM)
        .value("KS21_SILVER", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21_SILVER)
        .value("ST25", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::ST25)
        .value("KS21i", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i);

    // MultiSenseInfo::DeviceInfo::ImagerType
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::ImagerType>(m, "ImagerType")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::ImagerType::UNKNOWN)
        .value("CMV2000_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_GREY)
        .value("CMV2000_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_COLOR)
        .value("CMV4000_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_GREY)
        .value("CMV4000_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_COLOR)
        .value("FLIR_TAU2", multisense::MultiSenseInfo::DeviceInfo::ImagerType::FLIR_TAU2)
        .value("AR0234_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::AR0234_GREY)
        .value("AR0239_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::AR0239_COLOR);

    // MultiSenseInfo::DeviceInfo::LightingType
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::LightingType>(m, "LightingType")
        .value("NONE", multisense::MultiSenseInfo::DeviceInfo::LightingType::NONE)
        .value("INTERNAL", multisense::MultiSenseInfo::DeviceInfo::LightingType::INTERNAL)
        .value("EXTERNAL", multisense::MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL)
        .value("PATTERN_PROJECTOR", multisense::MultiSenseInfo::DeviceInfo::LightingType::PATTERN_PROJECTOR);

    // MultiSenseInfo::DeviceInfo::LensType
    py::enum_<multisense::MultiSenseInfo::MultiSenseInfo::DeviceInfo::LensType>(m, "LensType")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::LensType::UNKNOWN)
        .value("STANDARD", multisense::MultiSenseInfo::DeviceInfo::LensType::STANDARD)
        .value("FISHEYE", multisense::MultiSenseInfo::DeviceInfo::LensType::FISHEYE);

    // MultiSenseInfo::NetworkInfo
    py::class_<multisense::MultiSenseInfo::MultiSenseInfo::NetworkInfo>(m, "NetworkInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::NetworkInfo)
        .def_readwrite("ip_address", &multisense::MultiSenseInfo::NetworkInfo::ip_address)
        .def_readwrite("gateway", &multisense::MultiSenseInfo::NetworkInfo::gateway)
        .def_readwrite("netmask", &multisense::MultiSenseInfo::NetworkInfo::netmask);

    // MultiSenseInfo::DeviceInfo
    py::class_<multisense::MultiSenseInfo::DeviceInfo>(m, "DeviceInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::DeviceInfo)
        .def_readwrite("camera_name", &multisense::MultiSenseInfo::DeviceInfo::camera_name)
        .def_readwrite("build_date", &multisense::MultiSenseInfo::DeviceInfo::build_date)
        .def_readwrite("serial_number", &multisense::MultiSenseInfo::DeviceInfo::serial_number)
        .def_readwrite("hardware_revision", &multisense::MultiSenseInfo::DeviceInfo::hardware_revision)
        .def_readwrite("pcb_info", &multisense::MultiSenseInfo::DeviceInfo::pcb_info)
        .def_readwrite("imager_name", &multisense::MultiSenseInfo::DeviceInfo::imager_name)
        .def_readwrite("imager_type", &multisense::MultiSenseInfo::DeviceInfo::imager_type)
        .def_readwrite("imager_width", &multisense::MultiSenseInfo::DeviceInfo::imager_width)
        .def_readwrite("imager_height", &multisense::MultiSenseInfo::DeviceInfo::imager_height)
        .def_readwrite("lens_name", &multisense::MultiSenseInfo::DeviceInfo::lens_name)
        .def_readwrite("lens_type", &multisense::MultiSenseInfo::DeviceInfo::lens_type)
        .def_readwrite("nominal_stereo_baseline", &multisense::MultiSenseInfo::DeviceInfo::nominal_stereo_baseline)
        .def_readwrite("nominal_focal_length", &multisense::MultiSenseInfo::DeviceInfo::nominal_focal_length)
        .def_readwrite("nominal_relative_aperture", &multisense::MultiSenseInfo::DeviceInfo::nominal_relative_aperture)
        .def_readwrite("lighting_type", &multisense::MultiSenseInfo::DeviceInfo::lighting_type)
        .def_readwrite("number_of_lights", &multisense::MultiSenseInfo::DeviceInfo::number_of_lights)
        .def("has_aux_camera", &multisense::MultiSenseInfo::DeviceInfo::has_aux_camera);

    // MultiSenseInfo::Version
    py::class_<multisense::MultiSenseInfo::Version>(m, "Version")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::Version)
        .def("__lt__", &multisense::MultiSenseInfo::Version::operator<)
        .def_readwrite("major", &multisense::MultiSenseInfo::Version::major)
        .def_readwrite("minor", &multisense::MultiSenseInfo::Version::minor)
        .def_readwrite("patch", &multisense::MultiSenseInfo::Version::patch)
        .def("to_string", &multisense::MultiSenseInfo::Version::to_string);

    // MultiSenseInfo::SensorVersion
    py::class_<multisense::MultiSenseInfo::MultiSenseInfo::SensorVersion>(m, "SensorVersion")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::SensorVersion)
        .def_readwrite("firmware_build_date", &multisense::MultiSenseInfo::SensorVersion::firmware_build_date)
        .def_readwrite("firmware_version", &multisense::MultiSenseInfo::SensorVersion::firmware_version)
        .def_readwrite("hardware_version", &multisense::MultiSenseInfo::SensorVersion::hardware_version);

    // MultiSenseInfo::SupportedOperatingMode
    py::class_<multisense::MultiSenseInfo::SupportedOperatingMode>(m, "SupportedOperatingMode")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::SupportedOperatingMode)
        .def_readwrite("width", &multisense::MultiSenseInfo::SupportedOperatingMode::width)
        .def_readwrite("height", &multisense::MultiSenseInfo::SupportedOperatingMode::height)
        .def_readwrite("disparities", &multisense::MultiSenseInfo::SupportedOperatingMode::disparities)
        .def_readwrite("supported_sources", &multisense::MultiSenseInfo::SupportedOperatingMode::supported_sources);

    // MultiSenseInfo::ImuInfo::Source
    py::class_<multisense::MultiSenseInfo::ImuInfo::Source>(m, "ImuSource")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::ImuInfo::Source)
        .def_readwrite("name", &multisense::MultiSenseInfo::ImuInfo::Source::name)
        .def_readwrite("device", &multisense::MultiSenseInfo::ImuInfo::Source::device)
        .def_readwrite("rates", &multisense::MultiSenseInfo::ImuInfo::Source::rates)
        .def_readwrite("ranges", &multisense::MultiSenseInfo::ImuInfo::Source::ranges);

    // MultiSenseInfo::ImuInfo
    py::class_<multisense::MultiSenseInfo::ImuInfo>(m, "ImuInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::ImuInfo)
        .def_readwrite("accelerometer", &multisense::MultiSenseInfo::ImuInfo::accelerometer)
        .def_readwrite("gyroscope", &multisense::MultiSenseInfo::ImuInfo::gyroscope)
        .def_readwrite("magnetometer", &multisense::MultiSenseInfo::ImuInfo::magnetometer);


    // MultiSenseInfo
    py::class_<multisense::MultiSenseInfo>(m, "MultiSenseInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo)
        .def_readwrite("device", &multisense::MultiSenseInfo::device)
        .def_readwrite("version", &multisense::MultiSenseInfo::version)
        .def_readwrite("operating_modes", &multisense::MultiSenseInfo::operating_modes)
        .def_readwrite("imu", &multisense::MultiSenseInfo::imu)
        .def_readwrite("network", &multisense::MultiSenseInfo::network);

    // ChannelImplementation
    py::enum_<multisense::Channel::ChannelImplementation>(m, "ChannelImplementation")
        .value("LEGACY", multisense::Channel::ChannelImplementation::LEGACY);

    // Channel::ReceiveBufferConfig
    py::class_<multisense::Channel::ReceiveBufferConfig>(m, "ReceiveBufferConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::Channel::ReceiveBufferConfig)
        .def_readwrite("num_small_buffers", &multisense::Channel::ReceiveBufferConfig::num_small_buffers)
        .def_readwrite("small_buffer_size", &multisense::Channel::ReceiveBufferConfig::small_buffer_size)
        .def_readwrite("num_large_buffers", &multisense::Channel::ReceiveBufferConfig::num_large_buffers)
        .def_readwrite("large_buffer_size", &multisense::Channel::ReceiveBufferConfig::large_buffer_size);

    // Channel::ChannelConfig
    py::class_<multisense::Channel::Config>(m, "ChannelConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::Channel::Config)
        .def_readwrite("ip_address", &multisense::Channel::Config::ip_address)
        .def_readwrite("mtu", &multisense::Channel::Config::mtu)
        .def_readwrite("receive_timeout", &multisense::Channel::Config::receive_timeout)
        .def_readwrite("command_port", &multisense::Channel::Config::command_port)
        .def_readwrite("interface", &multisense::Channel::Config::interface)
        .def_readwrite("receive_buffer_configuration", &multisense::Channel::Config::receive_buffer_configuration);

    // Channel
    py::class_<multisense::Channel, std::unique_ptr<multisense::Channel>>(m, "Channel")
        .def_static("create", &multisense::Channel::create, py::return_value_policy::move,
                py::arg("config"),
                py::arg("impl") = multisense::Channel::ChannelImplementation::LEGACY)
        .def("__enter__", [](multisense::Channel &self) -> multisense::Channel &
        {
            return self;
        })
        .def("__exit__", [](multisense::Channel &, py::object, py::object, py::object)
        {
            return false;
        })
        .def("start_streams", &multisense::Channel::start_streams, py::call_guard<py::gil_scoped_release>())
        .def("stop_streams", &multisense::Channel::stop_streams, py::call_guard<py::gil_scoped_release>())
        .def("add_image_frame_callback", &multisense::Channel::add_image_frame_callback, py::call_guard<py::gil_scoped_acquire>())
        .def("add_imu_frame_callback", &multisense::Channel::add_imu_frame_callback, py::call_guard<py::gil_scoped_acquire>())
        .def("connect", &multisense::Channel::connect)
        .def("disconnect", &multisense::Channel::disconnect)
        .def("get_next_image_frame", &multisense::Channel::get_next_image_frame, py::call_guard<py::gil_scoped_release>())
        .def("get_configuration", &multisense::Channel::get_configuration, py::call_guard<py::gil_scoped_release>())
        .def("set_configuration", &multisense::Channel::set_configuration, py::call_guard<py::gil_scoped_release>())
        .def("get_calibration", &multisense::Channel::get_calibration, py::call_guard<py::gil_scoped_release>())
        .def("set_calibration", &multisense::Channel::set_calibration, py::call_guard<py::gil_scoped_release>())
        .def("get_info", &multisense::Channel::get_info, py::call_guard<py::gil_scoped_release>())
        .def("set_device_info", &multisense::Channel::set_device_info, py::call_guard<py::gil_scoped_release>())
        .def("get_system_status", &multisense::Channel::get_system_status, py::call_guard<py::gil_scoped_release>());

    // Utilities
    py::class_<multisense::Point<void>>(m, "Point")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<void>::x)
        .def_readwrite("y", &multisense::Point<void>::y)
        .def_readwrite("z", &multisense::Point<void>::z);

    py::class_<multisense::Point<uint8_t>>(m, "PointLuma8")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<uint8_t>::x)
        .def_readwrite("y", &multisense::Point<uint8_t>::y)
        .def_readwrite("z", &multisense::Point<uint8_t>::z)
        .def_readwrite("color", &multisense::Point<uint8_t>::color);

    py::class_<multisense::Point<uint16_t>>(m, "PointLuma16")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<uint16_t>::x)
        .def_readwrite("y", &multisense::Point<uint16_t>::y)
        .def_readwrite("z", &multisense::Point<uint16_t>::z)
        .def_readwrite("color", &multisense::Point<uint16_t>::color);

    py::class_<multisense::Point<std::array<uint8_t, 3>>>(m, "PointBGR")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<std::array<uint8_t, 3>>::x)
        .def_readwrite("y", &multisense::Point<std::array<uint8_t, 3>>::y)
        .def_readwrite("z", &multisense::Point<std::array<uint8_t, 3>>::z)
        .def_readwrite("color", &multisense::Point<std::array<uint8_t, 3>>::color);

    py::class_<multisense::PointCloud<void>>(m, "PointCloud")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<void>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<void> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            const std::vector<size_t> strides = {sizeof(multisense::Point<void>), sizeof(float)};
            const size_t element_size = sizeof(float);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             shape.size(),
                             shape,
                             strides));
        });

    py::class_<multisense::PointCloud<uint8_t>>(m, "PointCloudLuma8")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<uint8_t>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<uint8_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            //
            // Make sure we skip over the color
            //
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint8_t>), sizeof(float)};
            const size_t element_size = sizeof(multisense::Point<uint8_t>);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             2,
                             shape,
                             strides));
        })
        .def_property_readonly("as_raw_array", [](const multisense::PointCloud<uint8_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size())};
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint8_t>)};
            const size_t element_size = sizeof(multisense::Point<uint8_t>);

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             "13B",
                             1,
                             shape,
                             strides));
        });


    py::class_<multisense::PointCloud<uint16_t>>(m, "PointCloudLuma16")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<uint16_t>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<uint16_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            //
            // Make sure we skip over the color
            //
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint16_t>), sizeof(float)};
            const size_t element_size = sizeof(multisense::Point<uint16_t>);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             2,
                             shape,
                             strides));
        })
        .def_property_readonly("as_raw_array", [](const multisense::PointCloud<uint16_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size())};
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint16_t>)};
            const size_t element_size = sizeof(multisense::Point<uint16_t>);

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             "14B",
                             1,
                             shape,
                             strides));
        });

    py::class_<multisense::PointCloud<std::array<uint8_t, 3>>>(m, "PointCloudBGR")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<std::array<uint8_t, 3>>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<std::array<uint8_t, 3>> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            //
            // Make sure we skip over the color
            //
            const std::vector<size_t> strides = {sizeof(multisense::Point<std::array<uint8_t, 3>>), sizeof(float)};
            const size_t element_size = sizeof(multisense::Point<std::array<uint8_t, 3>>);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             2,
                             shape,
                             strides));
        })
        .def_property_readonly("as_raw_array", [](const multisense::PointCloud<std::array<uint8_t, 3>> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size())};
            const std::vector<size_t> strides = {sizeof(multisense::Point<std::array<uint8_t, 3>>)};
            const size_t element_size = sizeof(multisense::Point<std::array<uint8_t, 3>>);

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             "15B",
                             1,
                             shape,
                             strides));
        });

    m.def("write_image",
          [](const multisense::Image &image, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_image(image, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<void> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<uint8_t> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<uint16_t> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<std::array<uint8_t, 3>> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("create_pointcloud", &multisense::create_pointcloud, py::call_guard<py::gil_scoped_release>());

    //
    // Handle creation of colorized pointclouds with either just an ImageFrame, or the explicit disparity
    // and color images
    //
    m.def("create_gray8_pointcloud",
            static_cast<std::optional<multisense::PointCloud<uint8_t>>(*)(const multisense::ImageFrame&,
                                                                          double,
                                                                          const multisense::DataSource&,
                                                                          const multisense::DataSource&)>(
                &multisense::create_color_pointcloud<uint8_t>),
            py::call_guard<py::gil_scoped_release>());
    m.def("create_gray8_pointcloud",
            static_cast<std::optional<multisense::PointCloud<uint8_t>>(*)(const multisense::Image&,
                                                                          const std::optional<multisense::Image>&,
                                                                          double,
                                                                          const multisense::StereoCalibration&)>(
                &multisense::create_color_pointcloud<uint8_t>),
            py::call_guard<py::gil_scoped_release>());

    m.def("create_gray16_pointcloud",
            static_cast<std::optional<multisense::PointCloud<uint16_t>>(*)(const multisense::ImageFrame&,
                                                                          double,
                                                                          const multisense::DataSource&,
                                                                          const multisense::DataSource&)>(
                &multisense::create_color_pointcloud<uint16_t>),
            py::call_guard<py::gil_scoped_release>());
    m.def("create_gray16_pointcloud",
            static_cast<std::optional<multisense::PointCloud<uint16_t>>(*)(const multisense::Image&,
                                                                          const std::optional<multisense::Image>&,
                                                                          double,
                                                                          const multisense::StereoCalibration&)>(
                &multisense::create_color_pointcloud<uint16_t>),
            py::call_guard<py::gil_scoped_release>());

    m.def("create_bgr_pointcloud",
            static_cast<std::optional<multisense::PointCloud<std::array<uint8_t, 3>>>(*)(const multisense::ImageFrame&,
                                                                                         double,
                                                                                         const multisense::DataSource&,
                                                                                         const multisense::DataSource&)>(
                &multisense::create_color_pointcloud<std::array<uint8_t, 3>>),
            py::call_guard<py::gil_scoped_release>());
    m.def("create_bgr_pointcloud",
            static_cast<std::optional<multisense::PointCloud<std::array<uint8_t, 3>>>(*)(const multisense::Image&,
                                                                                         const std::optional<multisense::Image>&,
                                                                                         double,
                                                                                         const multisense::StereoCalibration&)>(
                &multisense::create_color_pointcloud<std::array<uint8_t, 3>>),
            py::call_guard<py::gil_scoped_release>());

    m.def("create_depth_image", &multisense::create_depth_image, py::call_guard<py::gil_scoped_release>());

    m.def("create_bgr_from_ycbcr420", &multisense::create_bgr_from_ycbcr420, py::call_guard<py::gil_scoped_release>());

    m.def("create_bgr_image", &multisense::create_bgr_image, py::call_guard<py::gil_scoped_release>());
}
