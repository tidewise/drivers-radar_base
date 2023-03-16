#include <boost/test/unit_test.hpp>
#include <radar_base/Radar.hpp>

using namespace radar_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_creating_an_object)
{
    Radar radar;
}
