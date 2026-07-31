/**
 * @file MultiSenseSecondaryApplication.hh
 *
 * Public types and protocol helpers for MultiSense secondary applications.
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <MultiSense/utility/BufferStream.hh>
#include <MultiSense/wire/ThermalMessage.hh>

#include "MultiSenseChannel.hh"

namespace multisense
{
namespace secondary_application
{
namespace thermal
{

inline constexpr char APPLICATION_NAME[] = "crl_thermal";

/// @brief Commands supported by the thermal secondary application.
enum class ControlCommand : uint16_t
{
    SET_RECTIFIED = 1,
    SET_BITS_PER_PIXEL = 2,
    SET_POST_PROCESSING = 3,
};

/// @brief Current thermal application configuration.
struct Config
{
    bool rectified = false;
    uint8_t bits_per_pixel = 0;
    uint8_t max_imagers = 0;
    uint32_t imager_enable_mask = 0;
    uint16_t width = 0;
    uint16_t height = 0;
};

/// @brief A control request sent to the thermal application.
struct Control
{
    ControlCommand command = ControlCommand::SET_RECTIFIED;
    uint32_t value = 0;
};

/// @brief Thermal-specific metadata paired with the standard MultiSense image type.
struct ThermalImage
{
    uint8_t imager_id = 0;
    bool rectified = false;
    multisense::Image image{};
};

/// @brief A synchronized set of images produced by the thermal application.
struct FrameGroup
{
    int64_t frame_id = 0;
    TimeT camera_timestamp{};
    bool ptp_locked = false;
    uint32_t imager_enable_mask = 0;
    std::vector<ThermalImage> images{};
};

} // namespace thermal
} // namespace secondary_application

/// @brief Deserialize an opaque secondary-application payload as a MultiSenseWire type.
///
/// WireType must provide a static WIRE_SIZE member and a constructor accepting
/// BufferStreamReader&.
template<typename WireType>
std::optional<WireType> deserialize_secondary_application_payload(
    const uint8_t *payload, std::size_t payload_size)
{
    if (payload == nullptr || payload_size < WireType::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(payload, payload_size);
    return WireType(reader);
}

template<typename WireType>
std::optional<WireType> deserialize_secondary_application_payload(
    const std::vector<uint8_t> &payload)
{
    return deserialize_secondary_application_payload<WireType>(payload.data(), payload.size());
}

/// @brief Deserialize a zero-copy secondary-application payload view.
template<typename WireType>
std::optional<WireType> deserialize_secondary_application_payload(
    const BufferWrapper &payload)
{
    return deserialize_secondary_application_payload<WireType>(payload.data(), payload.size());
}

/// @brief Serialize a MultiSenseWire type as an opaque secondary-application payload.
///
/// WireType must provide a static WIRE_SIZE member and serialize(BufferStreamWriter&).
template<typename WireType>
std::vector<uint8_t> serialize_secondary_application_payload(WireType message)
{
    crl::multisense::details::utility::BufferStreamWriter writer(WireType::WIRE_SIZE);
    message.serialize(writer);
    const auto *data = static_cast<const uint8_t *>(writer.data());
    return std::vector<uint8_t>(data, data + writer.tell());
}

template<>
inline std::optional<secondary_application::thermal::Config>
deserialize_secondary_application_payload<secondary_application::thermal::Config>(
    const uint8_t *payload, std::size_t payload_size)
{
    namespace thermal_wire = crl::multisense::details::wire;
    if (payload == nullptr || payload_size < thermal_wire::ThermalConfig::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(payload, payload_size);
    const thermal_wire::ThermalConfig wire_config(reader);
    if (wire_config.magic != thermal_wire::THERMAL_GROUP_MAGIC ||
        wire_config.version != thermal_wire::THERMAL_GROUP_VERSION)
    {
        return std::nullopt;
    }

    return secondary_application::thermal::Config{
        wire_config.rectified != 0,
        wire_config.bitsPerPixel,
        wire_config.maxImagers,
        wire_config.imagerEnableMask,
        wire_config.width,
        wire_config.height};
}

template<>
inline std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Config config)
{
    namespace thermal_wire = crl::multisense::details::wire;
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
inline std::optional<secondary_application::thermal::Control>
deserialize_secondary_application_payload<secondary_application::thermal::Control>(
    const uint8_t *payload, std::size_t payload_size)
{
    namespace thermal_wire = crl::multisense::details::wire;
    if (payload == nullptr || payload_size < thermal_wire::ThermalControl::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(payload, payload_size);
    const thermal_wire::ThermalControl wire_control(reader);
    if (wire_control.magic != thermal_wire::THERMAL_GROUP_MAGIC ||
        wire_control.version != thermal_wire::THERMAL_GROUP_VERSION)
    {
        return std::nullopt;
    }
    return secondary_application::thermal::Control{
        static_cast<secondary_application::thermal::ControlCommand>(wire_control.command),
        wire_control.value};
}

template<>
inline std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Control control)
{
    namespace thermal_wire = crl::multisense::details::wire;
    thermal_wire::ThermalControl wire_control;
    wire_control.command = static_cast<uint16_t>(control.command);
    wire_control.value = control.value;
    return serialize_secondary_application_payload(wire_control);
}

/// @brief Validate and convert a thermal payload into public, zero-copy Image views.
inline std::optional<secondary_application::thermal::FrameGroup> deserialize_thermal_frame_group(
    const BufferWrapper &payload)
{
    namespace thermal = secondary_application::thermal;
    namespace thermal_wire = crl::multisense::details::wire;
    if (payload.size() < thermal_wire::ThermalFrameGroup::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(payload.data(), payload.size());
    const thermal_wire::ThermalFrameGroup wire_group(reader);
    const auto &wire_header = wire_group.header();
    const auto timestamp = TimeT{std::chrono::seconds{wire_header.timeSeconds} +
                                 std::chrono::microseconds{wire_header.timeMicroseconds}};

    auto storage = payload.shared_data();
    size_t payload_offset = payload.shared_data_offset();
    if (!storage)
    {
        storage = std::make_shared<const std::vector<uint8_t>>(
            payload.data(), payload.data() + payload.size());
        payload_offset = 0;
    }

    thermal::FrameGroup output;
    output.frame_id = wire_header.frameId;
    output.camera_timestamp = timestamp;
    output.ptp_locked = wire_header.ptpLocked != 0;
    output.imager_enable_mask = wire_header.imagerEnableMask;
    output.images.reserve(wire_group.size());

    for (size_t index = 0; index < wire_group.size(); ++index)
    {
        const auto &wire_image = wire_group.at(index);
        const auto &descriptor = wire_image.descriptor();

        multisense::Image image;
        image.raw_data = storage;
        image.image_data_offset = static_cast<int64_t>(payload_offset + descriptor.offset);
        image.image_data_length = descriptor.length;
        image.format = descriptor.bitsPerPixel == 8
                           ? multisense::Image::PixelFormat::MONO8
                           : multisense::Image::PixelFormat::MONO16;
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

inline std::optional<secondary_application::thermal::FrameGroup> deserialize_thermal_frame_group(
    const std::shared_ptr<const BufferWrapper> &payload)
{
    if (!payload)
    {
        return std::nullopt;
    }
    return deserialize_thermal_frame_group(*payload);
}

/// @brief Retrieve and deserialize the active secondary application's configuration.
///
/// ConfigType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
template<typename ConfigType>
std::optional<ConfigType> get_secondary_application_config(Channel &channel)
{
    const auto payload = channel.get_secondary_application_config();
    if (!payload)
    {
        return std::nullopt;
    }
    return deserialize_secondary_application_payload<ConfigType>(*payload);
}

/// @brief Serialize and send a typed control request to the active secondary application.
///
/// ControlType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
template<typename ControlType>
Status send_secondary_application_control(Channel &channel, ControlType control)
{
    return channel.send_secondary_application_control(
        serialize_secondary_application_payload(std::move(control)));
}

} // namespace multisense
