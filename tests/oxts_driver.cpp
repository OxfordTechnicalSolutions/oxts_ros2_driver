// Copyright 2021 Oxford Technical Solutions Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/test/unit_test.hpp>

#include <thread>
#include <utility>

#include "oxts_driver/driver.hpp"
#include "tests.h"

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

BOOST_AUTO_TEST_SUITE(oxts_driver, *boost::unit_test::fixture<Fixture>())

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
      long(1000 / driver->get_parameter("ncom_rate").get_value<long>()) + 100};

  std::this_thread::sleep_for(packet_duration);
  BOOST_CHECK(ncomTime <= driver->getNcomTime(driver->nrx));
}

BOOST_AUTO_TEST_CASE(getUnitIp) {
  BOOST_CHECK(driver->getUnitIp() ==
              driver->get_parameter("unit_ip").get_value<std::string>());
}

BOOST_AUTO_TEST_CASE(getUnitPort) {
  BOOST_CHECK(driver->getUnitPort() ==
              driver->get_parameter("unit_port").get_value<long>());
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests::oxts_driver
