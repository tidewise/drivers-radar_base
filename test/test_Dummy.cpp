#include <boost/test/unit_test.hpp>
#include <radar_base/Dummy.hpp>

using namespace radar_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    radar_base::DummyClass dummy;
    dummy.welcome();
}
