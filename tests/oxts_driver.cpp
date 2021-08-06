#include <boost/test/unit_test.hpp>

#include <thread>
#include <utility>

#include "tests.h"
#include "oxts_driver/driver.hpp"

using namespace oxts_driver;

namespace tests::oxts_driver {

std::shared_ptr<OxtsDriver> driver;

struct Fixture {
    Fixture() {
        rclcpp::init(0, nullptr);
        driver = tests::newDriver();
        std::thread{[]() { rclcpp::spin(driver); }}.detach();
    }

    ~Fixture() {
        driver.reset();
        rclcpp::shutdown();
    }
};

BOOST_AUTO_TEST_SUITE(oxts_driver, * boost::unit_test::fixture<Fixture>())

BOOST_AUTO_TEST_CASE(checkRate) {
    BOOST_CHECK(driver->checkRate(-1, 0) == false);
    BOOST_CHECK(driver->checkRate(10, 0));
    BOOST_CHECK(driver->checkRate(0, 1) == false);
    BOOST_CHECK(driver->checkRate(1, 1));
}

BOOST_AUTO_TEST_CASE(getTimestamp) {
    auto timestamp = driver->getTimestamp();
    BOOST_CHECK(timestamp.nanoseconds() >= 0);
    BOOST_CHECK(timestamp < driver->getTimestamp());
}

BOOST_AUTO_TEST_CASE(getNcomTime) {
    auto ncomTime = driver->getNcomTime(driver->nrx);
    BOOST_CHECK(ncomTime.nanoseconds() >= 0);

    std::chrono::milliseconds packet_duration{
        long(1000 / driver->get_parameter("ncom_rate").get_value<long>()) + 100
    };

    std::this_thread::sleep_for(packet_duration);
    BOOST_CHECK(ncomTime <= driver->getNcomTime(driver->nrx));
}

BOOST_AUTO_TEST_CASE(getUnitIp) {
    BOOST_CHECK(driver->getUnitIp() == driver->get_parameter("unit_ip").get_value<std::string>());
}

BOOST_AUTO_TEST_CASE(getUnitPort) {
    BOOST_CHECK(driver->getUnitPort() == driver->get_parameter("unit_port").get_value<long>());
}

BOOST_AUTO_TEST_SUITE_END()

}