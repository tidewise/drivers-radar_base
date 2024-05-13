#include <gtest/gtest.h>
#include <radar_base/EchoToImageLUT.hpp>

using namespace radar_base;
using namespace std;

struct EchoToImageTest : public ::testing::Test {
    int num_angles = 4;
    int sweep_size = 2;
    float beam_width = M_PI / 2;
    int window_size = 3;
    EchoToImageLUT lut = EchoToImageLUT(num_angles, sweep_size, beam_width, window_size);
};

TEST_F(EchoToImageTest, it_returns_the_initial_and_final_pixel_iterators_for_a_radar_dot)
{
    auto iterators = lut.getPixels(0, 1);
    ASSERT_EQ(iterators.first->x, 1);
    ASSERT_EQ(iterators.first->y, 0);
    ASSERT_EQ(iterators.second->x, 1);
    ASSERT_EQ(iterators.second->y, 1);
}
