#ifndef RADAR_BASE_BASETYPES_HPP
#define RADAR_BASE_BASETYPES_HPP

#include <base/Angle.hpp>
#include <base/Time.hpp>
#include <cstdint>
#include <vector>

namespace radar_base {
    struct ArpaTarget {
        /** Timestamp */
        base::Time time;
        /** Target unique identifier */
        uint64_t target_id = 0;
        /** Status 1 */
        uint16_t status1 = 0;
        /** Status 2 (GD-700 only) */
        uint16_t status2 = 0;
        /** Target distance from conning position (Consistent Common Reference Point) */
        float ccrp_distance = 0;
        /** Target bearing from conning position (Consistent Common Reference Point) */
        base::Angle ccrp_bearing;
        /** Target distance from antenna position */
        float antenna_distance = 0;
        /** Target bearing from antenna position */
        base::Angle antenna_bearing;
        /** Target speed (True) */
        float true_speed = 0;
        /** Target course (True) */
        base::Angle true_course;
        /** Target speed (Relative) */
        float relative_speed = 0;
        /** Target course (Relative) */
        base::Angle relative_course;
        /** Closest Point of Approach */
        float cpa = 0;
        /** Time to Closest Point of Approach */
        base::Time tcpa;
        /** Bow Crossing Range */
        float bcr = 0;
        /** Bow Crossing Time */
        base::Time bct;
        /** Antenna Number */
        int16_t antenna_number = 0;
        /** Target latitude */
        base::Angle latitude;
        /** Target longitude */
        base::Angle longitude;
    };

    struct ArpaTargets {
        base::Time time;
        std::vector<ArpaTarget> targets;
    };
}

#endif
