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

TEST(ThermalMessage, FrameGroupValidatesAndExposesImages)
{
    constexpr uint16_t width = 2;
    constexpr uint16_t height = 2;
    constexpr uint8_t imageCount = 2;
    constexpr uint32_t imageLength = width * height * sizeof(uint16_t);
    constexpr uint32_t headerLength = wire::ThermalGroupHeader::WIRE_SIZE +
                                      imageCount * wire::ThermalImageDescriptor::WIRE_SIZE;
    constexpr uint32_t payloadLength = headerLength + imageCount * imageLength;

    wire::ThermalGroupHeader header;
    header.magic = wire::THERMAL_GROUP_MAGIC;
    header.version = wire::THERMAL_GROUP_VERSION;
    header.payloadType = wire::THERMAL_PAYLOAD_FRAME_GROUP;
    header.headerLength = headerLength;
    header.frameId = 42;
    header.numImages = imageCount;

    utility::BufferStreamWriter writer(payloadLength);
    header.serialize(writer);
    for (uint8_t index = 0; index < imageCount; ++index) {
        wire::ThermalImageDescriptor descriptor;
        descriptor.offset = headerLength + index * imageLength;
        descriptor.length = imageLength;
        descriptor.width = width;
        descriptor.height = height;
        descriptor.bitsPerPixel = 16;
        descriptor.imagerId = index;
        descriptor.serialize(writer);
    }
    const uint16_t pixels[width * height * imageCount] = {};
    writer.write(pixels, sizeof(pixels));

    utility::BufferStreamReader reader(
        static_cast<const uint8_t *>(writer.data()), writer.tell());
    wire::ThermalFrameGroup group(reader);
    ASSERT_EQ(group.size(), imageCount);
    EXPECT_EQ(group.header().frameId, 42);
    EXPECT_EQ(group.at(1).descriptor().imagerId, 1);
    EXPECT_EQ(group.at(1).size(), imageLength);
    EXPECT_EQ(group.at(1).data(),
              static_cast<const uint8_t *>(writer.data()) + headerLength + imageLength);
}

TEST(ThermalMessage, FrameGroupRejectsInvalidImageGeometry)
{
    constexpr uint32_t headerLength = wire::ThermalGroupHeader::WIRE_SIZE +
                                      wire::ThermalImageDescriptor::WIRE_SIZE;
    wire::ThermalGroupHeader header;
    header.magic = wire::THERMAL_GROUP_MAGIC;
    header.version = wire::THERMAL_GROUP_VERSION;
    header.payloadType = wire::THERMAL_PAYLOAD_FRAME_GROUP;
    header.headerLength = headerLength;
    header.numImages = 1;

    wire::ThermalImageDescriptor descriptor;
    descriptor.offset = headerLength;
    descriptor.length = 1;
    descriptor.width = 2;
    descriptor.height = 2;
    descriptor.bitsPerPixel = 16;

    utility::BufferStreamWriter writer(headerLength + descriptor.length);
    header.serialize(writer);
    descriptor.serialize(writer);
    const uint8_t pixel = 0;
    writer.write(&pixel, sizeof(pixel));

    utility::BufferStreamReader reader(
        static_cast<const uint8_t *>(writer.data()), writer.tell());
    EXPECT_THROW(wire::ThermalFrameGroup{reader}, std::runtime_error);
}

} // namespace
