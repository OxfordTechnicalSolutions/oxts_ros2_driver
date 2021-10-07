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
#include <chrono>
#include <future>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "oxts_ins/convert.hpp"
#include "tests.h"

using namespace oxts_ins;
using namespace oxts_driver;
using namespace std::literals;

namespace tests::convert {

std::shared_ptr<OxtsDriver> driver;
std::shared_ptr<OxtsIns> ins;
std::string topicPrefix;

struct Fixture {
  Fixture() {
    rclcpp::init(0, nullptr);

    driver = tests::newDriver();
    std::thread{[]() { rclcpp::spin(driver); }}.detach();

    rclcpp::NodeOptions insOptions;
    insOptions.append_parameter_override("lrf_source", 1);
    insOptions.append_parameter_override("pub_nav_sat_fix_rate", 10);
    insOptions.append_parameter_override("pub_velocity_rate", 10);
    insOptions.append_parameter_override("pub_odometry_rate", 10);
    insOptions.append_parameter_override("pub_path_rate", 10);
    insOptions.append_parameter_override("pub_time_reference_rate", 10);
    insOptions.append_parameter_override("pub_ecef_pos_rate", 10);
    insOptions.append_parameter_override("pub_nav_sat_ref_rate", 10);
    insOptions.append_parameter_override("pub_lever_arm_rate", 10);
    insOptions.append_parameter_override("pub_imu_bias_rate", 10);
    ins = std::make_shared<OxtsIns>(insOptions);
    std::thread{[]() { rclcpp::spin(ins); }}.detach();

    topicPrefix =
        driver->get_parameter("topic_prefix").get_value<std::string>();
  }

  ~Fixture() {
    ins.reset();
    driver.reset();
    rclcpp::shutdown();
  }
};

template <typename Message>
void waitForMessage(std::string topic,
                    std::chrono::duration<long> timeout = 1s) {
  topic.insert(0, topicPrefix + '/');

  std::timed_mutex gotMsg;
  gotMsg.lock();

  auto _ = ins->create_subscription<Message>(
      topic, 1, [&gotMsg](typename Message::SharedPtr) { gotMsg.unlock(); });

  BOOST_CHECK_MESSAGE(gotMsg.try_lock_for(timeout),
                      std::string{"receiving a message on the topic \""} +
                          topic + '"');
}

BOOST_AUTO_TEST_SUITE(convert, *boost::unit_test::fixture<Fixture>())

BOOST_AUTO_TEST_CASE(ecef_pos) {
  waitForMessage<geometry_msgs::msg::PointStamped>("ecef_pos");
}

BOOST_AUTO_TEST_CASE(nav_sat_fix) {
  waitForMessage<sensor_msgs::msg::NavSatFix>("nav_sat_fix");
}

BOOST_AUTO_TEST_CASE(nav_sat_ref) {
  waitForMessage<oxts_msgs::msg::NavSatRef>("nav_sat_ref");
}

BOOST_AUTO_TEST_CASE(imu) { waitForMessage<sensor_msgs::msg::Imu>("imu"); }

BOOST_AUTO_TEST_CASE(velocity) {
  waitForMessage<geometry_msgs::msg::TwistStamped>("velocity");
}

BOOST_AUTO_TEST_CASE(odometry) {
  waitForMessage<nav_msgs::msg::Odometry>("odometry");
}

BOOST_AUTO_TEST_CASE(path) { waitForMessage<nav_msgs::msg::Path>("path"); }

BOOST_AUTO_TEST_CASE(time_reference) {
  waitForMessage<sensor_msgs::msg::TimeReference>("time_reference");
}

BOOST_AUTO_TEST_CASE(lever_arm) {
  waitForMessage<oxts_msgs::msg::LeverArm>("lever_arm");
}

BOOST_AUTO_TEST_CASE(imu_bias) {
  waitForMessage<oxts_msgs::msg::ImuBias>("imu_bias");
}

BOOST_AUTO_TEST_CASE(ncom) { waitForMessage<oxts_msgs::msg::Ncom>("ncom"); }

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests::convert
