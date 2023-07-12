#ifndef __RADAR_BASE_RADAR_HPP__
#define __RADAR_BASE_RADAR_HPP__

#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <vector>
namespace radar_base {
    struct Radar {
    public:
        base::Time timestamp;
        float range = base::unknown<float>();
        base::Angle step_angle = base::Angle::unknown();
        base::Angle start_heading = base::Angle::unknown();
        uint16_t sweep_length = base::unknown<uint16_t>();
        std::vector<base::Time> sweep_timestamps;
        std::vector<uint8_t> sweep_data;

        Radar();

        Radar(float range,
            uint16_t sweep_length,
            base::Angle step_angle,
            base::Angle start_heading,
            base::Time timestamp);

        ~Radar();
        bool verifyNextAngle(base::Angle angle);
        void addEcho(float range,
            uint16_t sweep_length,
            base::Angle step_angle,
            base::Angle heading,
            uint8_t* echo_data);
    };
} // namespaces

#endif
