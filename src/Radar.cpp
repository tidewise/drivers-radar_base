#include "Radar.hpp"

using namespace std;
using namespace radar_base;

Radar::Radar()
{
}

Radar::Radar(float range,
    uint16_t sweep_length,
    base::Angle step_angle,
    base::Angle start_heading,
    base::Time timestamp)
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
    base::Angle step_angle,
    base::Angle heading,
    uint8_t* echo_data)
{
    if (timestamp.isNull()) {
        timestamp = base::Time::now();
        this->range = range;
        this->sweep_length = sweep_length;
        this->step_angle = step_angle;
        this->start_heading = heading;
    }
    else if (this->range != range) {
        throw std::runtime_error(
            "Current range differs from configured values! Configured: " +
            to_string(this->range) + " Current:" + to_string(range));
    }
    else if (this->sweep_length != sweep_length) {
        throw std::runtime_error(
            "Current sweep length differs from configured values! Configured: " +
            to_string(this->sweep_length) + " Current:" + to_string(sweep_length));
    }
    else if (!this->step_angle.isApprox(step_angle)) {
        throw std::runtime_error(
            "Current step angle differs from configured values! Configured: " +
            to_string(this->step_angle.getRad()) +
            " Current:" + to_string(step_angle.getRad()));
    }
    else {
        base::Angle start = heading - step_angle * sweep_timestamps.size();
        if (!this->start_heading.isApprox(start)) {
            throw std::runtime_error(
                "Current Start heading differs from configured values! Configured: " +
                to_string(this->start_heading.getRad()) +
                " Current:" + to_string(start.getRad()));
        }
    }
    sweep_timestamps.push_back(base::Time::now());
    sweep_data.insert(sweep_data.end(), echo_data, echo_data + sweep_length);
}
