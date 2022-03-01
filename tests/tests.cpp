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

#define BOOST_TEST_MAIN

#include <boost/test/unit_test.hpp>
#include <memory>
#include <filesystem>

#include "oxts_driver/driver.hpp"
#include "tests.h"

namespace tests {

std::unique_ptr<oxts_driver::OxtsDriver> newDriver() {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
      "ncom", std::filesystem::canonical(std::filesystem::path{"./test.ncom"}));
  options.append_parameter_override("ncom_rate", 250);
  return std::make_unique<oxts_driver::OxtsDriver>(options);
}

bool approxEqual(double l, double r, double delta) {
  return std::abs(l - r) <= delta;
}

bool approxEqual(const std::vector<double> &l, const std::vector<double> &r,
                 double delta) {
  if (l.size() != r.size())
    return false;

  for (decltype(l.size()) i = 0; i < l.size(); ++i)
    if (!approxEqual(l[i], r[i], delta))
      return false;

  return true;
}

bool approxEqual(const Point::Cart &l, const Point::Cart &r, double delta) {
  return approxEqual(l.x(), r.x(), delta) && approxEqual(l.y(), r.y(), delta) &&
         approxEqual(l.z(), r.z(), delta);
}

bool approxEqual(const Point::Geodetic &l, const Point::Geodetic &r,
                 double delta) {
  return approxEqual(l.lat(), r.lat(), delta) &&
         approxEqual(l.lon(), r.lon(), delta) &&
         approxEqual(l.alt(), r.alt(), delta);
}

bool approxEqual(const tf2::Quaternion &l, const tf2::Quaternion &r,
                 double delta) {
  return approxEqual(l.x(), r.x(), delta) && approxEqual(l.y(), r.y(), delta) &&
         approxEqual(l.z(), r.z(), delta);
}

bool approxEqual(const Lrf &l, const Lrf &r, double delta) {
  return approxEqual(l.lat(), r.lat(), delta) &&
         approxEqual(l.lon(), r.lon(), delta) &&
         approxEqual(l.alt(), r.alt(), delta) &&
         approxEqual(l.heading(), r.heading(), delta);
}

bool approxEqual(const geometry_msgs::msg::Point &l,
                 const geometry_msgs::msg::Point &r, double delta) {
  return approxEqual(l.x, r.x, delta) && approxEqual(l.y, r.y, delta) &&
         approxEqual(l.z, r.z, delta);
}

bool approxEqual(const geometry_msgs::msg::Vector3 &l,
                 const geometry_msgs::msg::Vector3 &r, double delta) {
  return approxEqual(l.x, r.x, delta) && approxEqual(l.y, r.y, delta) &&
         approxEqual(l.z, r.z, delta);
}

bool approxEqual(const geometry_msgs::msg::Quaternion &l,
                 const geometry_msgs::msg::Quaternion &r, double delta) {
  return approxEqual(l.x, r.x, delta) && approxEqual(l.y, r.y, delta) &&
         approxEqual(l.z, r.z, delta) && approxEqual(l.w, r.w, delta);
}

} // namespace tests
