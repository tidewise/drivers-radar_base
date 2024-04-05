#include "Radar.hpp"

using namespace base;
using namespace std;
using namespace radar_base;

Radar::Radar()
{
}

Radar::Radar(float range,
    uint16_t sweep_length,
    Angle step_angle,
    Angle start_heading,
    Time timestamp)
    : range(range)
    , sweep_length(sweep_length)
    , step_angle(step_angle)
    , start_heading(start_heading)
    , time(timestamp)
{
}

Radar::~Radar()
{
}

void Radar::addEcho(float range,
    uint16_t sweep_length,
    Angle step_angle,
    Angle heading,
    uint8_t* echo_data)
{
    if (time.isNull()) {
        time = Time::now();
        this->range = range;
        this->sweep_length = sweep_length;
        this->step_angle = step_angle;
        this->start_heading = heading;
    }
    else if (this->range != range) {
        throw runtime_error("Current range differs from expected value! Expected: " +
                            to_string(this->range) + " Current:" + to_string(range));
    }
    else if (this->sweep_length != sweep_length) {
        throw runtime_error(
            "Current sweep length differs from expected value! Expected: " +
            to_string(this->sweep_length) + " Current:" + to_string(sweep_length));
    }
    else if (!this->step_angle.isApprox(step_angle)) {
        throw runtime_error("Current step angle differs from expected value! Expected: " +
                            to_string(this->step_angle.getRad()) +
                            " Current:" + to_string(step_angle.getRad()));
    }
    else {
        if (!verifyNextAngle(heading)) {
            throw runtime_error(
                "Current angle differs from expected! Expected: " +
                to_string(
                    (start_heading + step_angle * sweep_timestamps.size()).getRad()) +
                " Current:" + to_string(heading.getRad()));
        }
    }
    sweep_timestamps.push_back(Time::now());
    sweep_data.insert(sweep_data.end(), echo_data, echo_data + sweep_length);
}

bool Radar::verifyNextAngle(Angle angle)
{
    if (time.isNull()) {
        return true;
    }
    Angle expected = start_heading + step_angle * (sweep_timestamps.size());
    return expected.isApprox(angle);
}

std::size_t Radar::size()
{
    return sweep_timestamps.size();
}

void Radar::updateEchoes(Radar const& radar_echo,
    base::Angle const& yaw_correction,
    std::vector<uint8_t>& world_echoes)
{
    double angle = (radar_echo.start_heading).getRad() + yaw_correction.getRad();
    if (angle < 0) {
        angle = 2 * M_PI + angle;
    }

    int start_angle_unit = round(angle / abs(radar_echo.step_angle.getRad()));
    int angles_in_a_frame = round(2 * M_PI / abs(radar_echo.step_angle.getRad()));
    int signal = copysign(1, radar_echo.step_angle.getRad());
    for (size_t current_angle = 0; current_angle < radar_echo.sweep_timestamps.size();
         current_angle++) {

        int sweep_global_angular_position =
            (start_angle_unit + current_angle * signal) % angles_in_a_frame;
        if (sweep_global_angular_position < 0) {
            sweep_global_angular_position += angles_in_a_frame;
        }
        std::copy(radar_echo.sweep_data.begin() + current_angle * radar_echo.sweep_length,
            radar_echo.sweep_data.begin() + (current_angle + 1) * radar_echo.sweep_length,
            world_echoes.begin() +
                radar_echo.sweep_length * sweep_global_angular_position);
    }
}