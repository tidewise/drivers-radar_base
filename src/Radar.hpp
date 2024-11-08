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
            base::Angle echo_angle2radar,
            uint8_t* echo_data);

        /**
         * Iterates over the sweeps of a radar echo saving them into the world echoes
         * vector.
         *
         * @param radar_echo the current radar echo to be saved
         * @param yaw_correction the yaw correction to be applied into this radar echo
         * @param world_echoes the serialized full radar rotation to be updated
         */
        static void updateEchoes(Radar const& radar_echo,
            base::Angle const& yaw_correction,
            std::vector<uint8_t>& world_echoes);
        /**
         * Checks if at least one echo dot (elements of the \see sweep_data vector) is not
         * zero
         */
        bool allZero() const;
    };
} // namespaces

#endif
