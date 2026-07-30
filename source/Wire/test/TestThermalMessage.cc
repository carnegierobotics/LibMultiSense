#include <gtest/gtest.h>

#include "MultiSense/wire/ThermalMessage.hh"

namespace {

using namespace crl::multisense::details;

TEST(ThermalMessage, GroupHeaderRoundTrip)
{
    wire::ThermalGroupHeader input;
    input.magic = wire::THERMAL_GROUP_MAGIC;
    input.version = wire::THERMAL_GROUP_VERSION;
    input.payloadType = wire::THERMAL_PAYLOAD_FRAME_GROUP;
    input.headerLength = wire::ThermalGroupHeader::WIRE_SIZE +
                         wire::ThermalImageDescriptor::WIRE_SIZE;
    input.frameId = 1234;
    input.timeSeconds = 10;
    input.timeMicroseconds = 20;
    input.ptpLocked = 1;
    input.numImages = 1;
    input.imagerEnableMask = 0x5;

    utility::BufferStreamWriter writer(wire::ThermalGroupHeader::WIRE_SIZE);
    input.serialize(writer);
    EXPECT_EQ(writer.tell(), wire::ThermalGroupHeader::WIRE_SIZE);

    utility::BufferStreamReader reader(
        static_cast<const uint8_t*>(writer.data()), writer.tell());
    wire::ThermalGroupHeader output(reader);

    EXPECT_EQ(output.magic, input.magic);
    EXPECT_EQ(output.headerLength, input.headerLength);
    EXPECT_EQ(output.frameId, input.frameId);
    EXPECT_EQ(output.numImages, input.numImages);
    EXPECT_EQ(output.imagerEnableMask, input.imagerEnableMask);
}

TEST(ThermalMessage, PayloadTypesHaveStableWireSizes)
{
    wire::ThermalImageDescriptor image;
    utility::BufferStreamWriter imageWriter(wire::ThermalImageDescriptor::WIRE_SIZE);
    image.serialize(imageWriter);
    EXPECT_EQ(imageWriter.tell(), wire::ThermalImageDescriptor::WIRE_SIZE);

    wire::ThermalControl control;
    utility::BufferStreamWriter controlWriter(wire::ThermalControl::WIRE_SIZE);
    control.serialize(controlWriter);
    EXPECT_EQ(controlWriter.tell(), wire::ThermalControl::WIRE_SIZE);

    wire::ThermalConfig config;
    utility::BufferStreamWriter configWriter(wire::ThermalConfig::WIRE_SIZE);
    config.serialize(configWriter);
    EXPECT_EQ(configWriter.tell(), wire::ThermalConfig::WIRE_SIZE);
}

} // namespace
