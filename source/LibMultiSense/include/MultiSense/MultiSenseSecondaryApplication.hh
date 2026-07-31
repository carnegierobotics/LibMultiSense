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

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <MultiSense/utility/BufferStream.hh>

#include "MultiSenseChannel.hh"

namespace multisense
{
namespace secondary_application
{
namespace thermal
{

///
/// @brief The name used to identify the thermal secondary application
///
inline constexpr char APPLICATION_NAME[] = "crl_thermal";

///
/// @brief Commands supported by the thermal secondary application
///
enum class ControlCommand : uint16_t
{
    SET_RECTIFIED = 1,
    SET_BITS_PER_PIXEL = 2,
    SET_POST_PROCESSING = 3,
};

///
/// @brief The current thermal secondary-application configuration
///
struct Config
{
    ///
    /// @brief True when the application produces rectified images
    ///
    bool rectified = false;

    ///
    /// @brief The number of bits used to represent each pixel
    ///
    uint8_t bits_per_pixel = 0;

    ///
    /// @brief The maximum number of imagers supported by the application
    ///
    uint8_t max_imagers = 0;

    ///
    /// @brief A bitmask identifying the enabled imagers
    ///
    uint32_t imager_enable_mask = 0;

    ///
    /// @brief The image width in pixels
    ///
    uint16_t width = 0;

    ///
    /// @brief The image height in pixels
    ///
    uint16_t height = 0;
};

///
/// @brief A control request sent to the thermal secondary application
///
struct Control
{
    ///
    /// @brief The control command to execute
    ///
    ControlCommand command = ControlCommand::SET_RECTIFIED;

    ///
    /// @brief The command-specific value
    ///
    uint32_t value = 0;
};

///
/// @brief Thermal-specific metadata paired with the standard MultiSense image type
///
struct ThermalImage
{
    ///
    /// @brief The application-defined imager identifier
    ///
    uint8_t imager_id = 0;

    ///
    /// @brief True when the image has been rectified
    ///
    bool rectified = false;

    ///
    /// @brief The thermal image and its standard image metadata
    ///
    multisense::Image image{};
};

///
/// @brief A synchronized set of images produced by the thermal secondary application
///
struct FrameGroup
{
    ///
    /// @brief The frame identifier shared by the images in this group
    ///
    int64_t frame_id = 0;

    ///
    /// @brief The camera timestamp associated with this frame group
    ///
    TimeT camera_timestamp{};

    ///
    /// @brief True when the camera timestamp is synchronized to a PTP master
    ///
    bool ptp_locked = false;

    ///
    /// @brief A bitmask identifying the enabled imagers
    ///
    uint32_t imager_enable_mask = 0;

    ///
    /// @brief The images contained in this frame group
    ///
    std::vector<ThermalImage> images{};
};

} // namespace thermal
} // namespace secondary_application

// Generic payload codecs

///
/// @brief Deserialize an opaque secondary-application payload as a MultiSenseWire type
///
/// WireType must provide a static WIRE_SIZE member and a constructor accepting
/// BufferStreamReader&.
///
/// @tparam WireType The MultiSenseWire type represented by the payload
/// @param payload A pointer to the first payload byte
/// @param payload_size The number of bytes in the payload
/// @return The deserialized object, or std::nullopt when the payload is too small
///
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

///
/// @brief Deserialize an opaque secondary-application payload stored in a byte vector
///
/// @tparam WireType The MultiSenseWire type represented by the payload
/// @param payload The payload bytes
/// @return The deserialized object, or std::nullopt when the payload is too small
///
template<typename WireType>
std::optional<WireType> deserialize_secondary_application_payload(
    const std::vector<uint8_t> &payload)
{
    return deserialize_secondary_application_payload<WireType>(payload.data(), payload.size());
}

///
/// @brief Deserialize an opaque secondary-application payload view
///
/// @tparam WireType The MultiSenseWire type represented by the payload
/// @param payload The payload view
/// @return The deserialized object, or std::nullopt when the payload is too small
///
template<typename WireType>
std::optional<WireType> deserialize_secondary_application_payload(
    const BufferWrapper &payload)
{
    return deserialize_secondary_application_payload<WireType>(payload.data(), payload.size());
}

///
/// @brief Serialize a MultiSenseWire type as an opaque secondary-application payload
///
/// WireType must provide a static WIRE_SIZE member and serialize(BufferStreamWriter&).
///
/// @tparam WireType The MultiSenseWire type to serialize
/// @param message The message to serialize
/// @return The serialized payload bytes
///
template<typename WireType>
std::vector<uint8_t> serialize_secondary_application_payload(WireType message)
{
    crl::multisense::details::utility::BufferStreamWriter writer(WireType::WIRE_SIZE);
    message.serialize(writer);
    const auto *data = static_cast<const uint8_t *>(writer.data());
    return std::vector<uint8_t>(data, data + writer.tell());
}

// Built-in thermal payload codecs

///
/// @brief Deserialize a thermal secondary-application configuration payload
///
/// @param payload A pointer to the first payload byte
/// @param payload_size The number of bytes in the payload
/// @return The decoded configuration, or std::nullopt when the payload is invalid
///
template<>
MULTISENSE_API std::optional<secondary_application::thermal::Config>
deserialize_secondary_application_payload<secondary_application::thermal::Config>(
    const uint8_t *payload, std::size_t payload_size);

///
/// @brief Serialize a thermal secondary-application configuration
///
/// @param config The configuration to serialize
/// @return The serialized configuration payload
///
template<>
MULTISENSE_API std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Config config);

///
/// @brief Deserialize a thermal secondary-application control payload
///
/// @param payload A pointer to the first payload byte
/// @param payload_size The number of bytes in the payload
/// @return The decoded control request, or std::nullopt when the payload is invalid
///
template<>
MULTISENSE_API std::optional<secondary_application::thermal::Control>
deserialize_secondary_application_payload<secondary_application::thermal::Control>(
    const uint8_t *payload, std::size_t payload_size);

///
/// @brief Serialize a thermal secondary-application control request
///
/// @param control The control request to serialize
/// @return The serialized control payload
///
template<>
MULTISENSE_API std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Control control);

///
/// @brief Validate and convert a thermal payload into public, zero-copy image views
///
/// @param payload The thermal frame-group payload
/// @return The decoded frame group, or std::nullopt when the payload is too small
///
MULTISENSE_API std::optional<secondary_application::thermal::FrameGroup>
deserialize_thermal_frame_group(const BufferWrapper &payload);

///
/// @brief Validate and convert a shared thermal payload into public, zero-copy image views
///
/// @param payload The shared thermal frame-group payload
/// @return The decoded frame group, or std::nullopt when the payload is null or too small
///
MULTISENSE_API std::optional<secondary_application::thermal::FrameGroup>
deserialize_thermal_frame_group(const std::shared_ptr<const BufferWrapper> &payload);

// Typed channel helpers

///
/// @brief Retrieve and deserialize the active secondary application's configuration
///
/// ConfigType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
///
/// @tparam ConfigType The configuration type to retrieve
/// @param channel The channel connected to the device
/// @return The decoded configuration, or std::nullopt when it could not be retrieved
///
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

///
/// @brief Serialize and send a typed control request to the active secondary application
///
/// ControlType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
///
/// @tparam ControlType The control type to send
/// @param channel The channel connected to the device
/// @param control The control request to serialize and send
/// @return The status of the control request
///
template<typename ControlType>
Status send_secondary_application_control(Channel &channel, ControlType control)
{
    return channel.send_secondary_application_control(
        serialize_secondary_application_payload(std::move(control)));
}

} // namespace multisense
