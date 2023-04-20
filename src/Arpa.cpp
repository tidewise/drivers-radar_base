#include "Arpa.hpp"

using namespace std;
using namespace radar_base;

Arpa::Arpa()
{
}

Arpa::Arpa(base::samples::BodyState& target2antenna,
    float closest_point_of_approach,
    base::Time time_to_closest_approach,
    float bow_crossing_range,
    base::Time bow_crossing_time)
{
    timestamp = base::Time::now();
    this->target2antenna = target2antenna;
    this->closest_point_of_approach = closest_point_of_approach;
    this->time_to_closest_approach = time_to_closest_approach;
    this->bow_crossing_range = bow_crossing_range;
    this->bow_crossing_time = bow_crossing_time;
}

Arpa::~Arpa()
{
}
