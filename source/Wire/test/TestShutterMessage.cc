#include <gtest/gtest.h>
#include <MultiSense/wire/ShutterRequestMessage.hh>

using namespace crl::multisense::details;

TEST(ShutterRequest, RoundTripAndValidation)
{
    wire::ShutterRequest input;
    input.requestId = wire::ShutterRequest::nextRequestId();
    input.action = wire::ShutterRequest::DELAY;
    input.delayMs = 5000;
    utility::BufferStreamWriter writer(64);
    input.serialize(writer, 1);
    EXPECT_EQ(writer.tell(), 13u);
    utility::BufferStreamReader reader(static_cast<const uint8_t*>(writer.data()), writer.tell());
    wire::ShutterRequest output(reader, 1);
    EXPECT_EQ(output.requestId, input.requestId);
    EXPECT_EQ(output.action, input.action);
    EXPECT_EQ(output.delayMs, input.delayMs);
    EXPECT_TRUE(output.valid());
    output.delayMs = 0; EXPECT_FALSE(output.valid());
    output.delayMs = 60001; EXPECT_FALSE(output.valid());
    output.action = wire::ShutterRequest::CALIBRATE; EXPECT_FALSE(output.valid());
    output.delayMs = 0; EXPECT_TRUE(output.valid());
    output.requestId = 0; EXPECT_FALSE(output.valid());
    EXPECT_NE(wire::ShutterRequest::nextRequestId(), input.requestId);
}
