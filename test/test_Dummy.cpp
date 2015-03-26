#include <boost/test/unit_test.hpp>
#include <vizkit3d_normal_depth_map/Dummy.hpp>

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    vizkit3d_normal_depth_map::DummyClass dummy;
    dummy.welcome();
}
