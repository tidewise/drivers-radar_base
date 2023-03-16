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
    else if (this->range != range || this->sweep_length != sweep_length ||
             !this->step_angle.isApprox(step_angle) ||
             !this->start_heading.isApprox(heading)) {
        throw std::runtime_error("Current Echo differs from configured values!");
    }
    sweep_timestamps.push_back(base::Time::now());
    sweep_data.insert(sweep_data.end(), echo_data, echo_data + sweep_length);
}
