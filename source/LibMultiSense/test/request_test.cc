#include <gtest/gtest.h>
#include <limits>
#include "details/legacy/requests.hh"
#include "details/legacy/channel.hh"
#ifdef MULTISENSE_TEST_JSON
#include "MultiSense/MultiSenseSerialization.hh"
#endif

using namespace multisense;
using multisense::legacy::convert;
using Request = MultiSenseRequest::ShutterRequest;
using Action = Request::Action;
namespace wire = crl::multisense::details::wire;

TEST(requests, default_has_no_actions)
{
    EXPECT_FALSE(MultiSenseRequest{}.shutter);
}

TEST(requests, calibrate_and_resume_ignore_delay)
{
    for (const auto action : {Action::CALIBRATE, Action::RESUME}) {
        Request input{action, std::chrono::milliseconds{-1}};
        const auto output = convert(input);
        ASSERT_TRUE(output);
        EXPECT_EQ(output->action, action == Action::CALIBRATE ?
                                  wire::ShutterRequest::CALIBRATE : wire::ShutterRequest::RESUME);
        EXPECT_EQ(output->delayMs, 0u);
        EXPECT_TRUE(output->valid());
    }
}

TEST(requests, delay_defaults_and_bounds)
{
    Request input;
    input.action = Action::DELAY;
    const auto default_delay = convert(input);
    ASSERT_TRUE(default_delay);
    EXPECT_EQ(default_delay->delayMs, 5000u);
    for (const int64_t ms : {1, 5000, 60000}) {
        input.delay = std::chrono::milliseconds{ms};
        const auto output = convert(input);
        ASSERT_TRUE(output);
        EXPECT_EQ(output->action, wire::ShutterRequest::DELAY);
        EXPECT_EQ(output->delayMs, static_cast<uint32_t>(ms));
        EXPECT_TRUE(output->valid());
    }
    for (const int64_t ms : {int64_t{-1}, int64_t{0}, int64_t{60001},
                            std::numeric_limits<int64_t>::max()}) {
        input.delay = std::chrono::milliseconds{ms};
        EXPECT_FALSE(convert(input));
    }
}

TEST(requests, invalid_action_is_rejected)
{
    Request input;
    input.action = static_cast<Action>(255);
    EXPECT_FALSE(convert(input));
}

TEST(requests, repeated_submission_is_a_new_action)
{
    const Request input;
    const auto first = convert(input);
    const auto second = convert(input);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first->requestId, second->requestId);
    EXPECT_EQ(input.action, Action::CALIBRATE);
}

TEST(requests, disconnected_channel_returns_uninitialized)
{
    Channel::Config config;
    config.connect_on_initialization = false;
    config.receive_buffer_configuration.num_small_buffers = 1;
    config.receive_buffer_configuration.num_large_buffers = 1;
    auto channel = Channel::create(config);
    EXPECT_EQ(channel->send_request(MultiSenseRequest{}), Status::UNINITIALIZED);
    MultiSenseRequest request;
    request.shutter = Request{};
    EXPECT_EQ(channel->send_request(request), Status::UNINITIALIZED);
}

#ifdef MULTISENSE_TEST_JSON
TEST(requests, json_round_trip_preserves_action_and_optional_group)
{
    const MultiSenseRequest empty;
    EXPECT_EQ(nlohmann::json(empty).get<MultiSenseRequest>(), empty);
    for (const auto action : {Action::CALIBRATE, Action::DELAY, Action::RESUME}) {
        MultiSenseRequest input;
        input.shutter = Request{action, std::chrono::milliseconds{1200}};
        const nlohmann::json json = input;
        EXPECT_EQ(json.get<MultiSenseRequest>(), input);
        EXPECT_EQ(json.at("shutter").at("delay"), 1200);
    }
}

TEST(requests, json_unknown_actions_cannot_trigger_calibration)
{
    const auto json = nlohmann::json::parse(
        R"({"shutter":{"action":"DEALY","delay":5000}})");
    EXPECT_THROW(json.get<MultiSenseRequest>(), std::invalid_argument);
}
#endif
