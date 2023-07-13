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
    , timestamp(timestamp)
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
    if (timestamp.isNull()) {
        timestamp = Time::now();
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
            Angle expected = start_heading + step_angle * sweep_timestamps.size();
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
    if (timestamp.isNull()) {
        return true;
    }
    Angle expected = start_heading + step_angle * (sweep_timestamps.size());
    return expected.isApprox(angle);
}

std::size_t Radar::size()
{
    return sweep_timestamps.size();
}