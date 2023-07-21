#ifndef __RADAR_BASE_RADAR_HPP__
#define __RADAR_BASE_RADAR_HPP__

#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <vector>
namespace radar_base {
    struct Radar {
    public:
        float range = base::unknown<float>();
        uint16_t sweep_length = base::unknown<uint16_t>();
        base::Angle step_angle = base::Angle::unknown();
        base::Angle start_heading = base::Angle::unknown();
        base::Time timestamp;
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
        std::size_t size();
        void addEcho(float range,
            uint16_t sweep_length,
            base::Angle step_angle,
            base::Angle heading,
            uint8_t* echo_data);
    };
} // namespaces

#endif
