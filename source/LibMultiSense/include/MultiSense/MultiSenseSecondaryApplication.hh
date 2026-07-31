/**
 * @file MultiSenseSecondaryApplication.hh
 *
 * Public application-protocol types for MultiSense secondary applications.
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

/// @brief Retrieve and deserialize the active secondary application's configuration.
///
/// ConfigType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
template<typename ConfigType>
std::optional<ConfigType> get_secondary_application_config(Channel &channel)
{
    const auto payload = channel.get_secondary_application_config();
    if (!payload || payload->size() < ConfigType::WIRE_SIZE)
    {
        return std::nullopt;
    }

    crl::multisense::details::utility::BufferStreamReader reader(
        payload->data(), payload->size());
    return ConfigType(reader);
}

/// @brief Serialize and send a typed control request to the active secondary application.
///
/// ControlType may be a MultiSenseWire message or a public application type with an
/// explicit specialization supplied by LibMultiSense.
template<typename ControlType>
Status send_secondary_application_control(Channel &channel, ControlType control)
{
    crl::multisense::details::utility::BufferStreamWriter writer(ControlType::WIRE_SIZE);
    control.serialize(writer);
    const auto *data = static_cast<const uint8_t *>(writer.data());
    return channel.send_secondary_application_control(
        std::vector<uint8_t>(data, data + writer.tell()));
}

/// Public thermal types intentionally have no wire-format members. These specializations
/// perform their protocol conversion inside LibMultiSense.
template<>
MULTISENSE_API std::optional<secondary_application::thermal::Config>
get_secondary_application_config<secondary_application::thermal::Config>(Channel &channel);

template<>
MULTISENSE_API Status send_secondary_application_control<secondary_application::thermal::Control>(
    Channel &channel, secondary_application::thermal::Control control);

} // namespace multisense
