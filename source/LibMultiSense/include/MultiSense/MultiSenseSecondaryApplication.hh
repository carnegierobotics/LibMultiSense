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

// Generic payload codecs

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

// Built-in thermal payload codecs

template<>
MULTISENSE_API std::optional<secondary_application::thermal::Config>
deserialize_secondary_application_payload<secondary_application::thermal::Config>(
    const uint8_t *payload, std::size_t payload_size);

template<>
MULTISENSE_API std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Config config);

template<>
MULTISENSE_API std::optional<secondary_application::thermal::Control>
deserialize_secondary_application_payload<secondary_application::thermal::Control>(
    const uint8_t *payload, std::size_t payload_size);

template<>
MULTISENSE_API std::vector<uint8_t> serialize_secondary_application_payload(
    secondary_application::thermal::Control control);

/// @brief Validate and convert a thermal payload into public, zero-copy Image views.
MULTISENSE_API std::optional<secondary_application::thermal::FrameGroup>
deserialize_thermal_frame_group(const BufferWrapper &payload);

MULTISENSE_API std::optional<secondary_application::thermal::FrameGroup>
deserialize_thermal_frame_group(const std::shared_ptr<const BufferWrapper> &payload);

// Typed channel helpers

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
