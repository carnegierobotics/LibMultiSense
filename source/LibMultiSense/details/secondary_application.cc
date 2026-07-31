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

#include <chrono>
#include <memory>
#include <utility>

#include <MultiSense/MultiSenseSecondaryApplication.hh>
#include <MultiSense/wire/ThermalMessage.hh>

namespace multisense
{
namespace
{

namespace thermal = secondary_application::thermal;
namespace thermal_wire = crl::multisense::details::wire;

template<typename WireType>
std::optional<WireType> deserialize_thermal_wire_payload(
    const uint8_t *payload, std::size_t payload_size)
{
    const auto wire_message = deserialize_secondary_application_payload<WireType>(
        payload, payload_size);
    if (!wire_message || wire_message->magic != thermal_wire::THERMAL_GROUP_MAGIC ||
        wire_message->version != thermal_wire::THERMAL_GROUP_VERSION)
    {
        return std::nullopt;
    }
    return wire_message;
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
    wire_config.maxImagers = config.max_imagers;
    wire_config.imagerEnableMask = config.imager_enable_mask;
    wire_config.width = config.width;
    wire_config.height = config.height;
    return serialize_secondary_application_payload(wire_config);
}

template<>
std::optional<thermal::Control>
deserialize_secondary_application_payload<thermal::Control>(
    const uint8_t *payload, std::size_t payload_size)
{
    const auto wire_control = deserialize_thermal_wire_payload<thermal_wire::ThermalControl>(
        payload, payload_size);
    if (!wire_control)
    {
        return std::nullopt;
    }

    return thermal::Control{
        static_cast<thermal::ControlCommand>(wire_control->command),
        wire_control->value};
}

template<>
std::vector<uint8_t> serialize_secondary_application_payload(thermal::Control control)
{
    thermal_wire::ThermalControl wire_control;
    wire_control.command = static_cast<uint16_t>(control.command);
    wire_control.value = control.value;
    return serialize_secondary_application_payload(wire_control);
}

std::optional<thermal::FrameGroup> deserialize_thermal_frame_group(
    const BufferWrapper &payload)
{
    if (payload.size() < thermal_wire::ThermalFrameGroup::WIRE_SIZE)
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

std::optional<thermal::FrameGroup> deserialize_thermal_frame_group(
    const std::shared_ptr<const BufferWrapper> &payload)
{
    if (!payload)
    {
        return std::nullopt;
    }
    return deserialize_thermal_frame_group(*payload);
}

} // namespace multisense
