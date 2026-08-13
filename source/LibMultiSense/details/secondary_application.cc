/**
 * @file secondary_application.cc
 *
 * Thermal secondary-application protocol conversion.
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#include <algorithm>
#include <array>
#include <chrono>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <MultiSense/MultiSenseSecondaryApplication.hh>
#include <MultiSense/wire/ThermalMessage.hh>

#include "CalibrationYaml.hh"

namespace multisense
{
namespace
{

namespace thermal = secondary_application::thermal;
namespace thermal_wire = crl::multisense::details::wire;

template<typename WireType>
std::optional<WireType> deserialize_thermal_wire_payload(const uint8_t *payload,
    std::size_t payload_size, const uint32_t expected_magic = thermal_wire::THERMAL_GROUP_MAGIC)
{
    const auto wire_message = deserialize_secondary_application_payload<WireType>(
        payload, payload_size);
    if (!wire_message || wire_message->magic != expected_magic ||
        wire_message->version != thermal_wire::THERMAL_GROUP_VERSION)
    {
        return std::nullopt;
    }
    return wire_message;
}

Status send_thermal_control(Channel &channel, const uint16_t command, const uint32_t value)
{
    thermal_wire::ThermalControl control;
    control.command = command;
    control.value = value;
    return send_secondary_application_control(channel, control);
}

template<std::size_t Rows, std::size_t Columns>
void assign_matrix(std::array<std::array<float, Columns>, Rows> &output,
                   const std::vector<float> &input)
{
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            output[row][column] = input[row * Columns + column];
        }
    }
}

template<std::size_t Rows, std::size_t Columns>
std::array<float, Rows * Columns> flatten_matrix(
    const std::array<std::array<float, Columns>, Rows> &input)
{
    std::array<float, Rows * Columns> output{};
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            output[row * Columns + column] = input[row][column];
        }
    }
    return output;
}

std::optional<std::string> extract_thermal_calibration_yaml(const std::string &data)
{
    //
    // The thermal application may prefix calibration data with a status line,
    // for example "reading blocks ... [OK]". Keep deserialize_calibration()
    // strict for callers parsing YAML directly, and remove the device-specific
    // preamble here at the transfer boundary.
    //
    if (data.rfind("M:", 0) == 0)
    {
        return data;
    }

    const auto start = data.find("\nM:");
    if (start == std::string::npos)
    {
        return std::nullopt;
    }
    return data.substr(start + 1);
}

} // namespace

template<>
std::optional<thermal::Config>
deserialize_secondary_application_payload<thermal::Config>(
    const uint8_t *payload, std::size_t payload_size)
{
    const auto wire_config = deserialize_thermal_wire_payload<thermal_wire::ThermalConfig>(
        payload, payload_size);
    if (!wire_config)
    {
        return std::nullopt;
    }

    return thermal::Config{
        wire_config->rectified != 0,
        wire_config->bitsPerPixel,
        wire_config->postProcMask,
        wire_config->maxImagers,
        wire_config->imagerEnableMask,
        wire_config->width,
        wire_config->height};
}

template<>
std::vector<uint8_t> serialize_secondary_application_payload(thermal::Config config)
{
    thermal_wire::ThermalConfig wire_config;
    wire_config.magic = thermal_wire::THERMAL_GROUP_MAGIC;
    wire_config.version = thermal_wire::THERMAL_GROUP_VERSION;
    wire_config.rectified = config.rectified ? 1 : 0;
    wire_config.bitsPerPixel = config.bits_per_pixel;
    wire_config.postProcMask = config.post_proc_mask.value_or(0);
    wire_config.maxImagers = config.max_imagers;
    wire_config.imagerEnableMask = config.imager_enable_mask;
    wire_config.width = config.width;
    wire_config.height = config.height;
    return serialize_secondary_application_payload(wire_config);
}

std::optional<thermal::Config> thermal::query_config(Channel &channel)
{
    return get_secondary_application_config<Config>(channel);
}

Status thermal::send_config(Channel &channel, const Config &config)
{
    if (config.bits_per_pixel != 8 && config.bits_per_pixel != 16)
    {
        throw std::invalid_argument("Thermal bits per pixel must be 8 or 16");
    }

    if (config.post_proc_mask &&
        (*config.post_proc_mask & ~thermal_wire::THERMAL_POST_PROCESSING_VALID))
    {
        throw std::invalid_argument("Thermal post processing mask has undefined bits");
    }

    const auto rectified_status = send_thermal_control(
        channel,
        thermal_wire::THERMAL_CONTROL_SET_RECTIFIED,
        config.rectified ? 1u : 0u);
    if (rectified_status != Status::OK)
    {
        return rectified_status;
    }

    const auto bits_status = send_thermal_control(
        channel,
        thermal_wire::THERMAL_CONTROL_SET_BITS_PER_PIXEL,
        config.bits_per_pixel);
    if (bits_status != Status::OK)
    {
        return bits_status;
    }

    if (!config.post_proc_mask)
    {
        return Status::OK;
    }

    return send_thermal_control(
        channel,
        thermal_wire::THERMAL_CONTROL_SET_POST_PROCESSING,
        *config.post_proc_mask);
}

std::optional<CameraCalibration> thermal::deserialize_calibration(const std::string &data)
{
    std::istringstream stream(data);
    stream.imbue(std::locale::classic());
    std::map<std::string, std::vector<float>> matrices;
    parseYaml(stream, matrices);

    const auto intrinsics = matrices.find("M");
    const auto distortion = matrices.find("D");
    const auto rotation = matrices.find("R");
    const auto projection = matrices.find("P");
    if (stream.fail() || matrices.size() != 4 ||
        intrinsics == matrices.end() || intrinsics->second.size() != 9 ||
        distortion == matrices.end() ||
        rotation == matrices.end() || rotation->second.size() != 9 ||
        projection == matrices.end() || projection->second.size() != 12)
    {
        return std::nullopt;
    }

    CameraCalibration output;
    assign_matrix(output.K, intrinsics->second);
    assign_matrix(output.R, rotation->second);
    assign_matrix(output.P, projection->second);

    if (distortion->second.empty())
    {
        output.distortion_type = CameraCalibration::DistortionType::NONE;
    }
    else if (distortion->second.size() == 5 ||
             (distortion->second.size() == 8 && distortion->second[5] == 0.0f &&
                                                   distortion->second[6] == 0.0f &&
                                                   distortion->second[7] == 0.0f))
    {
        output.distortion_type = CameraCalibration::DistortionType::PLUMBBOB;
        output.D.assign(distortion->second.begin(), distortion->second.begin() + 5);
    }
    else if (distortion->second.size() == 8)
    {
        output.distortion_type = CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL;
        output.D = distortion->second;
    }
    else
    {
        return std::nullopt;
    }

    return output;
}

std::string thermal::serialize_calibration(const CameraCalibration &calibration)
{
    std::vector<float> device_distortion = calibration.D;
    switch (calibration.distortion_type)
    {
        case CameraCalibration::DistortionType::NONE:
            if (!calibration.D.empty())
            {
                throw std::invalid_argument("A calibration without distortion must not have coefficients");
            }
            break;
        case CameraCalibration::DistortionType::PLUMBBOB:
            if (calibration.D.size() != 5)
            {
                throw std::invalid_argument("A plumb-bob calibration must have 5 distortion coefficients");
            }
            device_distortion.resize(8, 0.0f);
            break;
        case CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL:
            if (calibration.D.size() != 8)
            {
                throw std::invalid_argument("A rational-polynomial calibration must have 8 distortion coefficients");
            }
            break;
        default:
            throw std::invalid_argument("Unknown calibration distortion type");
    }

    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    const auto intrinsics = flatten_matrix(calibration.K);
    const auto rotation = flatten_matrix(calibration.R);
    const auto projection = flatten_matrix(calibration.P);
    writeCompactMatrix(stream, "M", 3, 3, intrinsics.data());
    writeCompactMatrix(stream, "D", 1, static_cast<uint32_t>(device_distortion.size()),
                       device_distortion.data());
    writeCompactMatrix(stream, "R", 3, 3, rotation.data());
    writeCompactMatrix(stream, "P", 3, 4, projection.data());
    return stream.str();
}

///
/// @brief Arm or clear the device's calibration selector
///
Status select_thermal_calibration(Channel &channel, uint32_t value)
{
    return send_thermal_control(channel, thermal_wire::THERMAL_CONTROL_SELECT_CALIBRATION, value);
}

///
/// @brief Read the config channel and split it into calibration header + bytes
///
std::optional<std::pair<thermal_wire::ThermalCalibrationResponse, std::string>>
read_thermal_calibration_chunk(Channel &channel)
{
    const auto payload = channel.get_secondary_application_config();
    if (!payload)
    {
        return std::nullopt;
    }

    const auto header = deserialize_thermal_wire_payload<
        thermal_wire::ThermalCalibrationResponse>(
            payload->data(),
            payload->size(),
            thermal_wire::THERMAL_CALIBRATION_MAGIC);
    if (!header)
    {
        return std::nullopt;
    }

    const std::size_t offset = thermal_wire::ThermalCalibrationResponse::WIRE_SIZE;
    if (header->chunkLength > payload->size() - offset)
    {
        return std::nullopt;
    }

    const auto *begin = reinterpret_cast<const char *>(payload->data()) + offset;
    return std::make_pair(*header, std::string(begin, begin + header->chunkLength));
}

std::optional<CameraCalibration> thermal::get_calibration(Channel &channel,
                                                         const uint8_t imager_id)
{
    std::string data;

    uint8_t chunk = 0;
    uint8_t chunk_count = 1;
    uint32_t total_length = 0;

    do
    {
        const uint32_t selector = static_cast<uint32_t>(imager_id) | (static_cast<uint32_t>(chunk) << 8);
        if (select_thermal_calibration(channel, selector) != Status::OK)
        {
            select_thermal_calibration(channel, thermal_wire::THERMAL_CALIBRATION_SELECT_NONE);
            return std::nullopt;
        }

        const auto piece = read_thermal_calibration_chunk(channel);
        if (!piece || piece->first.status != thermal_wire::THERMAL_CALIBRATION_OK ||
            piece->first.imagerId != imager_id || piece->first.chunk != chunk)
        {
            select_thermal_calibration(channel, thermal_wire::THERMAL_CALIBRATION_SELECT_NONE);
            return std::nullopt;
        }

        chunk_count = piece->first.chunkCount;
        total_length = piece->first.totalLength;
        data += piece->second;

        if (data.size() > piece->first.totalLength)
        {
            select_thermal_calibration(channel, thermal_wire::THERMAL_CALIBRATION_SELECT_NONE);
            return std::nullopt;
        }
    }
    while (++chunk < chunk_count);

    //
    // Leave the config channel answering with ThermalConfig again
    //
    select_thermal_calibration(channel, thermal_wire::THERMAL_CALIBRATION_SELECT_NONE);

    if (data.size() != total_length)
    {
        return std::nullopt;
    }

    const auto calibration_yaml = extract_thermal_calibration_yaml(data);
    if (!calibration_yaml)
    {
        return std::nullopt;
    }

    return deserialize_calibration(*calibration_yaml);
}

Status thermal::set_calibration(Channel &channel,
                                const uint8_t imager_id,
                                const CameraCalibration &calibration)
{
    const std::string data = serialize_calibration(calibration);
    if (data.size() > thermal_wire::THERMAL_CALIBRATION_TOTAL_MAX)
    {
        throw std::invalid_argument("Thermal calibration size is out of range");
    }

    const std::size_t max_per = thermal_wire::THERMAL_CALIBRATION_MAX_BYTES;
    const std::size_t count = (data.size() + max_per - 1) / max_per;

    for (std::size_t chunk = 0; chunk < count; ++chunk)
    {
        const std::size_t offset = chunk * max_per;
        const std::size_t length = std::min(max_per, data.size() - offset);

        thermal_wire::ThermalControl control;
        control.command = thermal_wire::THERMAL_CONTROL_SET_CALIBRATION;
        control.value = imager_id;

        thermal_wire::ThermalCalibrationTransfer transfer;
        transfer.imagerId = imager_id;
        transfer.chunk = static_cast<uint8_t>(chunk);
        transfer.chunkCount = static_cast<uint8_t>(count);
        transfer.totalLength = static_cast<uint32_t>(data.size());
        transfer.chunkLength = static_cast<uint32_t>(length);

        auto payload = serialize_secondary_application_payload(control);
        const auto transfer_bytes = serialize_secondary_application_payload(transfer);

        payload.insert(payload.end(), transfer_bytes.begin(), transfer_bytes.end());
        payload.insert(payload.end(), data.begin() + offset, data.begin() + offset + length);

        if (const auto status = channel.send_secondary_application_control(payload);
            status != Status::OK)
        {
            return status;
        }
    }

    return Status::OK;
}

std::optional<thermal::FrameGroup> thermal::deserialize_frame_group(
    const BufferWrapper &payload)
{
    if (payload.data() == nullptr ||
        payload.size() < thermal_wire::ThermalFrameGroup::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(payload.data(), payload.size());
    const thermal_wire::ThermalFrameGroup wire_group(reader);
    const auto &header = wire_group.header();
    const auto timestamp = TimeT{std::chrono::seconds{header.timeSeconds} +
                                 std::chrono::microseconds{header.timeMicroseconds}};

    auto storage = payload.shared_data();
    size_t payload_offset = payload.shared_data_offset();
    if (!storage)
    {
        storage = std::make_shared<const std::vector<uint8_t>>(
            payload.data(), payload.data() + payload.size());
        payload_offset = 0;
    }

    thermal::FrameGroup output;
    output.frame_id = header.frameId;
    output.camera_timestamp = timestamp;
    output.ptp_locked = header.ptpLocked != 0;
    output.imager_enable_mask = header.imagerEnableMask;
    output.images.reserve(wire_group.size());

    for (size_t index = 0; index < wire_group.size(); ++index)
    {
        const auto &descriptor = wire_group.at(index).descriptor();

        Image image;
        image.raw_data = storage;
        image.image_data_offset = static_cast<int64_t>(payload_offset + descriptor.offset);
        image.image_data_length = descriptor.length;
        image.format = descriptor.bitsPerPixel == 8
                           ? Image::PixelFormat::MONO8
                           : Image::PixelFormat::MONO16;
        image.width = descriptor.width;
        image.height = descriptor.height;
        image.camera_timestamp = timestamp;
        image.ptp_timestamp = output.ptp_locked ? timestamp : TimeT{};
        image.source = DataSource::THERMAL;

        output.images.push_back(thermal::ThermalImage{
            descriptor.imagerId,
            (descriptor.flags & thermal_wire::THERMAL_IMAGE_FLAG_RECTIFIED) != 0,
            std::move(image)});
    }

    return output;
}

std::optional<thermal::FrameGroup> thermal::deserialize_frame_group(
    const std::shared_ptr<const BufferWrapper> &payload)
{
    if (!payload)
    {
        return std::nullopt;
    }
    return deserialize_frame_group(*payload);
}

} // namespace multisense
