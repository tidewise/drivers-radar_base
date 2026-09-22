#ifndef __RADAR_BASE_RADAR_HPP__
#define __RADAR_BASE_RADAR_HPP__

#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <vector>
namespace radar_base {
    struct Radar {
    public:
        static constexpr double METERS_PER_SECOND2KNOT = 1.94384;

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

        /**
         * @brief Normalize the sweep data of the radar in the speed map mode
         *
         * The normalized value is interpreted following these rules:
         * 1 - Value 0 indicates that there is no object
         * 2 - Value 127 indicates null speed
         * 3 - Values above 127 indicates positive speeds, with increment of +0.25 m/s per
         * unit. Eg. 130 represents +0.75 m/s
         * 4 - Values below 127 indicates negative speeds, with increment of -0.25 m/s per
         * unit. Eg. 125 represents -0.5 m/s
         *
         * @param speed_map_sweep_data The radar sweep data in speed map mode
         *
         * 1 - Sweep datum within the range ]0, 82] indicates that the object has speed.
         * 2 - Value outside the range indicates that there is no object
         * 3 - Value 32 indicates a objeject with null speed
         * 4 - Value above 32 indicates a object with positive speed
         * 5 - Value below 32 indicates a object with negative speed
         */
        static void normalizeSpeedMap(std::vector<uint8_t>& speed_map_sweep_data);
    };
} // namespaces

#endif
