#include "EchoToImageLUT.hpp"
#include <algorithm>
#include <utility>

using namespace radar_base;
using namespace cv;
using namespace std;

static int discretizeAngle(double theta_rad, int num_angles)
{
    double angle_step = 2 * M_PI / num_angles;
    return round(theta_rad / angle_step);
}

EchoToImageLUT::EchoToImageLUT(int num_angles,
    int sweep_size,
    float beam_width,
    int window_size)
    : m_sweep_size(sweep_size)
    , m_num_angles(num_angles)
    , m_beam_width(beam_width)
    , m_window_size(window_size)
{
    auto raw_data = computeRawLUTTable(sweep_size, window_size, num_angles, beam_width);
    linearizeRawTable(raw_data);
}

vector<vector<Point>> EchoToImageLUT::computeRawLUTTable(int sweep_size,
    int window_size,
    int num_angles,
    float beam_width)
{
    RawTable data;
    data.resize(num_angles * sweep_size);

    float const pixel_per_cell = (window_size / 2.0) / sweep_size;
    float const half_beam_width = beam_width / 2;

    Point img_center(window_size / 2, window_size / 2);
    for (auto i = 0; i < window_size; i++) {
        for (auto j = 0; j < window_size; j++) {
            Point2i current_p(j, i);
            Point2i relative_p = img_center - current_p;
            double distance = norm(relative_p);
            int echo_index = round(distance / pixel_per_cell);
            if (echo_index >= sweep_size) {
                continue;
            }

            double theta_rad = atan2(relative_p.x, relative_p.y);
            int theta0 = discretizeAngle(theta_rad - half_beam_width, num_angles);
            int theta1 = discretizeAngle(theta_rad + half_beam_width, num_angles);
            for (int t = theta0; t <= theta1; ++t) {
                addRawLUTEntry(data, t, echo_index, sweep_size, num_angles, current_p);
            }
        }
    }
    return data;
}

void EchoToImageLUT::linearizeRawTable(RawTable const& table)
{
    m_data_index.resize(table.size() + 1);
    m_data.clear();
    for (size_t global_echo_id = 0; global_echo_id < table.size(); global_echo_id++) {
        m_data_index[global_echo_id] = m_data.size();
        for (size_t point_id = 0; point_id < table[global_echo_id].size(); point_id++) {
            m_data.push_back(table[global_echo_id][point_id]);
        }
    }
    m_data_index[table.size()] = m_data.size();
}

int EchoToImageLUT::normalizeAngle(int angle, int num_angles)
{
    angle = angle % num_angles;
    if (angle < 0) {
        return angle + num_angles;
    }
    return angle;
}

void EchoToImageLUT::addRawLUTEntry(RawTable& table,
    int angle,
    int echo_index,
    int sweep_size,
    int num_angles,
    Point2i const& p)
{
    angle = normalizeAngle(angle, num_angles);
    table[angle * sweep_size + echo_index].push_back(p);
}

void EchoToImageLUT::updateImage(Mat& image, int angle, int echo_index, int echo) const
{
    if (echo < 0) {
        echo = 0;
    }
    int data_i = angle * m_sweep_size + echo_index;
    auto begin = m_data_index[data_i];
    auto end = m_data_index[data_i + 1];
    for (auto id = begin; id < end; id++) {
        auto p = m_data[id];

        auto& current = image.at<Vec3b>(p);
        auto v = std::max<int>(current[0], echo);
        current = Vec3b(v, v, v);
    }
}

Point EchoToImageLUT::fetchEntry(int angle, int echo_index_in_sweep) const
{
    auto data_i = angle * m_sweep_size + echo_index_in_sweep;
    return m_data[m_data_index[data_i]];
}

bool EchoToImageLUT::hasMatchingConfiguration(int num_angles,
    int sweep_size,
    float beam_width,
    int window_size)
{
    return num_angles == m_num_angles && sweep_size == m_sweep_size &&
           beam_width == m_beam_width && window_size == m_window_size;
}

void EchoToImageLUT::drawImageFromEchoes(std::vector<uint8_t> const& world_echoes,
    cv::Mat& radar_frame)
{
    for (long i = 0; i < static_cast<long>(world_echoes.size()); i++) {
        updateImage(radar_frame, i / m_sweep_size, i % m_sweep_size, world_echoes.at(i));
    }
}

pair<vector<Point>::const_iterator, vector<Point>::const_iterator> EchoToImageLUT::
    getPixels(int angle_idx, int sweep_idx) const
{
    int initial_point_idx = m_data_index[angle_idx * m_sweep_size + sweep_idx];
    int final_point_idx = m_data_index[angle_idx * m_sweep_size + sweep_idx + 1] - 1;
    return std::make_pair(m_data.cbegin() + initial_point_idx,
        m_data.cbegin() + final_point_idx);
}
