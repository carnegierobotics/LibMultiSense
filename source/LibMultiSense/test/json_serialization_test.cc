#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <MultiSense/MultiSenseSerialization.hh>
#include <MultiSense/MultiSenseTypes.hh>
#include <chrono>

using namespace multisense;
using json = nlohmann::json;

TEST(JsonSerializationTest, TimeTSerialization)
{
    // Create a TimeT object
    auto now = std::chrono::system_clock::now();
    TimeT t = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    // Serialize to JSON
    json j = t;

    // Verify it's an integer and matches the count
    EXPECT_TRUE(j.is_number_integer());
    EXPECT_EQ(j.get<int64_t>(), t.time_since_epoch().count());

    // Deserialize from JSON
    TimeT t_deserialized = j.get<TimeT>();

    // Verify they match
    EXPECT_EQ(t, t_deserialized);
}

TEST(JsonSerializationTest, TimeTZeroSerialization)
{
    TimeT t{std::chrono::nanoseconds{0}};
    json j = t;
    EXPECT_EQ(j.get<int64_t>(), 0);

    TimeT t_deserialized = j.get<TimeT>();
    EXPECT_EQ(t, t_deserialized);
}
