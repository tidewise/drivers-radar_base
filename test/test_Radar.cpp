#include <gtest/gtest.h>
#include <math.h>
#include <radar_base/Radar.hpp>

using namespace radar_base;

struct RadarTest : public ::testing::Test {
    Radar radar;
    uint8_t echo[10] = {0, 0, 0, 0, 255, 255, 0, 0, 0, 0};
    float range = 2;
    uint16_t sweep_length = 10;
    base::Angle step_size = base::Angle::fromRad((2.0 / 8192.0) * 2 * M_PI);
    base::Angle heading_0 = base::Angle::fromRad(2.75196);
    base::Angle heading_1 = base::Angle::fromRad(2.7535);
    base::Angle heading_2 = base::Angle::fromRad(2.75503);
};

TEST_F(RadarTest, it_creates_an_object_correctly)
{
    radar.addEcho(range, sweep_length, step_size, heading_0, echo);
    radar.addEcho(range, sweep_length, step_size, heading_1, echo);
    radar.addEcho(range, sweep_length, step_size, heading_2, echo);
    uint8_t echo_result[3 * sizeof(echo)];
    memcpy(echo_result, echo, sizeof(echo));
    memcpy(echo_result + sizeof(echo), echo, sizeof(echo));
    memcpy(echo_result + 2 * sizeof(echo), echo, sizeof(echo));

    ASSERT_NO_THROW();
    ASSERT_EQ(radar.sweep_timestamps.size(), 3);
    ASSERT_EQ(radar.step_angle.rad, step_size.rad);
    ASSERT_EQ(radar.sweep_data.size(), sizeof(echo_result));
    for (int i = 0; i < static_cast<int>(sizeof(echo_result)); i++) {
        ASSERT_EQ(radar.sweep_data.at(i), echo_result[i]);
    }
}

TEST_F(RadarTest, it_crashes_with_different_step_size)
{
    base::Angle step_size2 = base::Angle::fromRad((1.0 / 8192.0) * 2 * M_PI);
    radar.addEcho(range, sweep_length, step_size, heading_0, echo);
    ASSERT_THROW(radar.addEcho(range, sweep_length, step_size2, heading_1, echo),
        std::runtime_error);
}

TEST_F(RadarTest, it_crashes_with_different_sweep_length)
{
    radar.addEcho(range, sweep_length, step_size, heading_0, echo);
    ASSERT_THROW(radar.addEcho(range, sweep_length + 2, step_size, heading_1, echo),
        std::runtime_error);
}

TEST_F(RadarTest, it_crashes_with_different_range)
{
    radar.addEcho(range, sweep_length, step_size, heading_0, echo);
    ASSERT_THROW(radar.addEcho(range + 2.0, sweep_length, step_size, heading_1, echo),
        std::runtime_error);
}