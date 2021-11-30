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

#include "oxts_ins/wrapper.hpp"

#include "tests.h"

using namespace RosNComWrapper;

namespace tests::wrapper
{

BOOST_AUTO_TEST_SUITE(wrapper)

BOOST_AUTO_TEST_CASE(getVat) {
  NComRxC nrx;
  nrx.mImu2VehRoll = 90;
  nrx.mImu2VehPitch = 180;
  nrx.mImu2VehHeading = 270;
  BOOST_CHECK(
    approxEqual(::getVat(&nrx), (tf2::Quaternion{-0.5, -0.5, 0.5, 0.5})));
}

BOOST_AUTO_TEST_CASE(getNsp) {
  NComRxC nrx;
  nrx.mNoSlipLeverArmX = 1;
  nrx.mNoSlipLeverArmY = 2;
  nrx.mNoSlipLeverArmZ = 3;
  BOOST_CHECK(::getNsp(&nrx) == (tf2::Vector3{1, 2, 3}));
}

BOOST_AUTO_TEST_CASE(getVehRPY) {
  NComRxC nrx;
  nrx.mRoll = 90;
  nrx.mPitch = 180;
  nrx.mHeading = 270;
  BOOST_CHECK(
    approxEqual(
      ::getVehRPY(&nrx),
      (tf2::Quaternion{0.707107, 0, 0, 0.707107})));
}

BOOST_AUTO_TEST_CASE(getBodyRPY) {
  NComRxC nrx;
  nrx.mRoll = 90;
  nrx.mPitch = 180;
  nrx.mHeading = 270;
  nrx.mImu2VehRoll = 200;
  nrx.mImu2VehRoll = 150;
  nrx.mImu2VehRoll = 100;
  BOOST_CHECK(
    approxEqual(
      ::getBodyRPY(&nrx),
      (tf2::Quaternion{0.7044, 0.7044, -0.0616, 0.0616})));
}

BOOST_AUTO_TEST_CASE(getNcomLrf) {
  NComRxC nrx;
  nrx.mRefLat = 1;
  nrx.mRefLon = 2;
  nrx.mRefAlt = 3;
  nrx.mRefHeading = 180;
  BOOST_CHECK(approxEqual(::getNcomLrf(&nrx), (Lrf{1, 2, 3, 1.570796})));
}

BOOST_AUTO_TEST_CASE(nav_sat_fix) {
  NComRxC nrx;
  nrx.mLat = 1;
  nrx.mLon = 2;
  nrx.mAlt = 3;
  nrx.mEastAcc = 4;
  nrx.mNorthAcc = 5;
  nrx.mAltAcc = 6;

  auto msg = ::nav_sat_fix(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(msg.latitude == 1);
  BOOST_CHECK(msg.longitude == 2);
  BOOST_CHECK(msg.altitude == 3);
  BOOST_CHECK(msg.position_covariance[0] == 16);
  BOOST_CHECK(msg.position_covariance[4] == 25);
  BOOST_CHECK(msg.position_covariance[8] == 36);
}

BOOST_AUTO_TEST_CASE(nav_sat_ref) {
  auto msg = ::nav_sat_ref(Lrf{1, 2, 3, 4}, std_msgs::msg::Header{});
  BOOST_CHECK(msg.latitude == 1);
  BOOST_CHECK(msg.longitude == 2);
  BOOST_CHECK(msg.altitude == 3);
  BOOST_CHECK(msg.heading == 4);
}

BOOST_AUTO_TEST_CASE(lever_arm_gap) {
  NComRxC nrx;
  nrx.mGAPx = 1;
  nrx.mGAPy = 2;
  nrx.mGAPz = 3;

  auto msg = ::lever_arm_gap(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(msg.lever_arm_id == "gap");
  BOOST_CHECK(msg.offset_x == 1);
  BOOST_CHECK(msg.offset_y == 2);
  BOOST_CHECK(msg.offset_z == 3);
}

BOOST_AUTO_TEST_CASE(imu_bias) {
  NComRxC nrx;
  nrx.mAxBias = 1;
  nrx.mAyBias = 2;
  nrx.mAzBias = 3;
  nrx.mWxBias = 4;
  nrx.mWyBias = 5;
  nrx.mWzBias = 6;

  geometry_msgs::msg::Vector3 expectedAccel{};
  expectedAccel.x = 1;
  expectedAccel.y = 2;
  expectedAccel.z = 3;

  geometry_msgs::msg::Vector3 expectedGyro{};
  expectedGyro.x = 4;
  expectedGyro.y = 5;
  expectedGyro.z = 6;

  auto msg = ::imu_bias(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(msg.accel == expectedAccel);
  BOOST_CHECK(msg.gyro == expectedGyro);
}

BOOST_AUTO_TEST_CASE(ecef_pos) {
  NComRxC nrx;
  nrx.mLat = 30;
  nrx.mLon = 50;
  nrx.mAlt = 150;

  geometry_msgs::msg::Point expectedPoint{};
  expectedPoint.x = 3553578.3715;
  expectedPoint.y = 4234989.7908;
  expectedPoint.z = 3170448.7354;

  auto msg = ::ecef_pos(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(approxEqual(msg.point, expectedPoint));
}

BOOST_AUTO_TEST_CASE(imu) {
  NComRxC nrx;
  nrx.mImu2VehRoll = 90;
  nrx.mImu2VehPitch = 180;
  nrx.mImu2VehHeading = 270;
  nrx.mRoll = 50;
  nrx.mPitch = 100;
  nrx.mHeading = 150;
  nrx.mWx = 35;
  nrx.mWy = 70;
  nrx.mWz = 105;
  nrx.mAx = 300;
  nrx.mAy = 290;
  nrx.mAz = 280;

  geometry_msgs::msg::Vector3 expectedVelocity{};
  expectedVelocity.x = -1.8326;
  expectedVelocity.y = 0.6109;
  expectedVelocity.z = -1.2217;

  geometry_msgs::msg::Vector3 expectedAcceleration{};
  expectedAcceleration.x = -280;
  expectedAcceleration.y = 300;
  expectedAcceleration.z = -290;

  auto msg = ::imu(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(approxEqual(msg.angular_velocity, expectedVelocity));
  BOOST_CHECK(approxEqual(msg.linear_acceleration, expectedAcceleration));
}

BOOST_AUTO_TEST_CASE(velocity) {
  NComRxC nrx;
  nrx.mImu2VehRoll = 180;
  nrx.mImu2VehPitch = 170;
  nrx.mImu2VehHeading = 160;
  nrx.mIsoVoX = 50;
  nrx.mIsoVoY = 100;
  nrx.mIsoVoZ = 150;
  nrx.mWx = 35;
  nrx.mWy = 70;
  nrx.mWz = 105;

  geometry_msgs::msg::Vector3 expectedLinearTwist{};
  expectedLinearTwist.x = -12.4076;
  expectedLinearTwist.y = -101.902;
  expectedLinearTwist.z = -156.404;

  geometry_msgs::msg::Vector3 expectedAngularTwist{};
  expectedAngularTwist.x = 73.4645;
  expectedAngularTwist.y = 47.7536;
  expectedAngularTwist.z = 97.3271;

  auto msg = ::velocity(&nrx, std_msgs::msg::Header{});

  BOOST_CHECK(approxEqual(msg.twist.linear, expectedLinearTwist));
  BOOST_CHECK(approxEqual(msg.twist.angular, expectedAngularTwist));
}

BOOST_AUTO_TEST_CASE(odometry) {
  NComRxC nrx;
  nrx.mLat = 30;
  nrx.mLon = 60;
  nrx.mAlt = 100;
  nrx.mRoll = 90;
  nrx.mPitch = 180;
  nrx.mHeading = 270;
  nrx.mImu2VehRoll = 200;
  nrx.mImu2VehRoll = 150;
  nrx.mImu2VehRoll = 100;
  nrx.mEastAcc = 10;
  nrx.mNorthAcc = 20;
  nrx.mAltAcc = 30;
  nrx.mIsoVoX = 50;
  nrx.mIsoVoY = 100;
  nrx.mIsoVoZ = 150;
  nrx.mWx = 35;
  nrx.mWy = 70;
  nrx.mWz = 105;

  auto msg = ::odometry(&nrx, std_msgs::msg::Header{}, Lrf{25, 45, 65, 123});

  geometry_msgs::msg::Point expectedPosition{};
  expectedPosition.x = -979434.4658;
  expectedPosition.y = -1220110.6938;
  expectedPosition.z = -194852.9041;

  geometry_msgs::msg::Quaternion expectedOrientation{};
  expectedOrientation.x = -0.0206;
  expectedOrientation.y = 0.0847;
  expectedOrientation.z = -0.9679;
  expectedOrientation.w = 0.2358;

  geometry_msgs::msg::Vector3 expectedAngularTwist{};
  expectedAngularTwist.x = 35;
  expectedAngularTwist.y = -115.56;
  expectedAngularTwist.z = 50.7035;

  geometry_msgs::msg::Vector3 expectedLinearTwist{};
  expectedLinearTwist.x = 50;
  expectedLinearTwist.y = 165.086;
  expectedLinearTwist.z = -72.4335;

  BOOST_CHECK(approxEqual(msg.pose.pose.position, expectedPosition));
  BOOST_CHECK(approxEqual(msg.pose.pose.orientation, expectedOrientation));
  BOOST_CHECK(approxEqual(msg.twist.twist.angular, expectedAngularTwist));
  BOOST_CHECK(approxEqual(msg.twist.twist.linear, expectedLinearTwist));

  BOOST_REQUIRE(msg.pose.covariance.size() >= 15);

  BOOST_CHECK(
    approxEqual(
      std::vector<double>{&msg.pose.covariance[0], &msg.pose.covariance[3]},
      std::vector<double>{216.165, -116.455, -131.085}));

  BOOST_CHECK(
    approxEqual(
      std::vector<double>{&msg.pose.covariance[6], &msg.pose.covariance[9]},
      std::vector<double>{-116.455, 330.424, -130.945}));

  BOOST_CHECK(
    approxEqual(
      std::vector<double>{&msg.pose.covariance[12], &msg.pose.covariance[15]},
      std::vector<double>{-131.085, -130.945, 853.41}));
}

geometry_msgs::msg::PoseStamped makePose(
  int ox, int oy, int oz, int px, int py,
  int pz)
{
  geometry_msgs::msg::PoseStamped pose{};
  pose.pose.orientation.x = ox;
  pose.pose.orientation.y = oy;
  pose.pose.orientation.z = oz;
  pose.pose.position.x = px;
  pose.pose.position.y = py;
  pose.pose.position.z = pz;
  return pose;
}

BOOST_AUTO_TEST_CASE(path) {
  std::vector<geometry_msgs::msg::PoseStamped> poses{
    makePose(1, 2, 3, 4, 5, 6), makePose(7, 8, 9, 10, 11, 12),
    makePose(13, 14, 15, 16, 17, 18), makePose(19, 20, 21, 22, 23, 24),
    makePose(25, 26, 27, 28, 29, 30), makePose(31, 32, 33, 34, 35, 36),
    makePose(37, 38, 39, 40, 41, 42)};

  auto msg = ::path(poses, std_msgs::msg::Header{});

  BOOST_CHECK(msg.poses == poses);
}

BOOST_AUTO_TEST_CASE(time_reference) {
  NComRxC nrx;
  nrx.mTimeWeekSecond = 12345.6789;
  auto msg = ::time_reference(&nrx, std_msgs::msg::Header{});
  BOOST_CHECK(msg.time_ref.sec == 12345);
  BOOST_CHECK(msg.time_ref.nanosec == 678900000);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests::wrapper
