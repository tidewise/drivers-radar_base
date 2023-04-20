#ifndef __RADAR_BASE_ARPA_HPP__
#define __RADAR_BASE_ARPA_HPP__

#include <base/samples/BodyState.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <vector>
namespace radar_base {
    struct Arpa {
    public:
        base::Time timestamp;
        int id;
        base::samples::BodyState target2antenna;
        float closest_point_of_approach; // ???
        base::Time time_to_closest_approach;
        float bow_crossing_range;
        base::Time bow_crossing_time;

        Arpa();

        Arpa(base::samples::BodyState& target2antenna,
            float closest_point_of_approach,
            base::Time time_to_closest_approach,
            float bow_crossing_range,
            base::Time bow_crossing_time);

        ~Arpa();
    };
} // namespaces

#endif
