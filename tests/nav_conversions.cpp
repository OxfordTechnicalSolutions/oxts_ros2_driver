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

#include "oxts_ins/nav_conversions.hpp"
#include "tests.h"

using namespace NavConversions;

namespace tests::nav_conversions {

BOOST_AUTO_TEST_SUITE(nav_conversions)

BOOST_AUTO_TEST_CASE(hprToQuaternion) {
  BOOST_CHECK(approxEqual(::hprToQuaternion(0, 0, 0),
                          (std::vector<double>{0, 0, 0, 1})));
  BOOST_CHECK(approxEqual(::hprToQuaternion(360, 360, 360),
                          (std::vector<double>{0, 0, 0, -1})));
  BOOST_CHECK(approxEqual(::hprToQuaternion(-90, -180, -270),
                          (std::vector<double>{0.5, 0.5, -0.5, -0.5})));
  BOOST_CHECK(approxEqual(
      ::hprToQuaternion(123, 321, 234),
      (std::vector<double>{-0.267585, -0.8104293, 0.2341715, 0.4655816})));
  BOOST_CHECK(approxEqual(
      ::hprToQuaternion(1000, 1000, 1000),
      (std::vector<double>{0.6937144, 0.0606921, 0.6937144, -0.183949})));
}

BOOST_AUTO_TEST_CASE(llaToEcef) {
  BOOST_CHECK(
      approxEqual(::llaToEcef(0, 0, 0), (std::vector<double>{6378137, 0, 0})));
  BOOST_CHECK(approxEqual(
      ::llaToEcef(12, 35, 72),
      (std::vector<double>{5111289.8066, 3578963.6526, 1317417.5009})));
  BOOST_CHECK(approxEqual(
      ::llaToEcef(-5.55, -5.55, -5.55),
      (std::vector<double>{6318670.3573, -613984.428, -612746.6264})));
  BOOST_CHECK(approxEqual(
      ::llaToEcef(9999, 9999, 9999),
      (std::vector<double>{156840.9832, -990254.9949, -6287848.3576})));
}

BOOST_AUTO_TEST_CASE(geodeticToEcef) {
  BOOST_CHECK(
      approxEqual(::geodeticToEcef(0, 0, 0), Point::Cart{6378137, 0, 0}));
  BOOST_CHECK(
      approxEqual(::geodeticToEcef(44, 11, 9999),
                  Point::Cart{4518105.0405, 878230.6545, 4415037.5015}));
  BOOST_CHECK(
      approxEqual(::geodeticToEcef(-3.14, -1.23, 55),
                  Point::Cart{6367212.7422, -136709.4584, -347035.9514}));
  BOOST_CHECK(
      approxEqual(::geodeticToEcef(9999, 9999, 9999),
                  Point::Cart{156840.9832, -990254.9949, -6287848.3576}));
}

BOOST_AUTO_TEST_CASE(ecefToGeodetic) {
  BOOST_CHECK(
      approxEqual(::ecefToGeodetic(0, 0, 0), Point::Geodetic{0, 0, -6378137}));
  BOOST_CHECK(approxEqual(::ecefToGeodetic(44, 11, 9999),
                          Point::Geodetic{-89.528, 14.0362, -6394086.5905}));
  BOOST_CHECK(approxEqual(::ecefToGeodetic(-3.14, -1.23, 55),
                          Point::Geodetic{-2.7554, -158.6087, -6378182.9612}));
  BOOST_CHECK(approxEqual(::ecefToGeodetic(9999, 9999, 9999),
                          Point::Geodetic{9.9773, 45, -6364420.0874}));
}

BOOST_AUTO_TEST_CASE(ecefToEnu) {
  BOOST_CHECK(approxEqual(::ecefToEnu(Point::Cart{1, 2, 3}, 20, 50, 10),
                          Point::Cart{0.5195, 13730.219, -6375646.1017}));
  BOOST_CHECK(approxEqual(::ecefToEnu(Point::Cart{999, 999, 999}, 20, 50, 10),
                          Point::Cart{-123.1336, 14185.5296, -6373984.9481}));
  BOOST_CHECK(
      approxEqual(::ecefToEnu(Point::Cart{45, 45, 30}, -3.14, -1.23, 55),
                  Point::Cart{45.9556, -2302.9496, -6378085.6303}));
  BOOST_CHECK(
      approxEqual(::ecefToEnu(Point::Cart{45, 45, 30}, 9999, 9999, 9999),
                  Point::Cart{51.4855, -6651.0537, -6367310.9757}));
}

BOOST_AUTO_TEST_CASE(enuToEcef) {
  BOOST_CHECK(
      approxEqual(::enuToEcef(1, 2, 3, 20, 50, 10),
                  Point::Cart{3854055.9838, 4593086.6205, 2167703.1135}));
  BOOST_CHECK(
      approxEqual(::enuToEcef(999, 999, 999, 20, 50, 10),
                  Point::Cart{3853673.8914, 4594183.8729, 2168980.6391}));
  BOOST_CHECK(
      approxEqual(::enuToEcef(45, 45, 30, -3.14, -1.23, 55),
                  Point::Cart{6367246.1205, -136665.1647, -346992.6622}));
  BOOST_CHECK(
      approxEqual(::enuToEcef(45, 45, 30, 9999, 9999, 9999),
                  Point::Cart{156893.1162, -990296.4894, -6287870.9487}));
}

BOOST_AUTO_TEST_CASE(enuToLrf) {
  BOOST_CHECK(approxEqual(::enuToLrf(0, 0, 0, 0), Point::Cart{0, 0, 0}));
  BOOST_CHECK(approxEqual(::enuToLrf(100, 100, 100, 180),
                          Point::Cart{20.2696, -139.9613, 100}));
  BOOST_CHECK(approxEqual(::enuToLrf(450, 340, 230, -90),
                          Point::Cart{102.3257, -554.6435, 230}));
}

BOOST_AUTO_TEST_CASE(geodeticToEnu) {
  BOOST_CHECK(
      approxEqual(::geodeticToEnu(0, 0, 0, 0, 0, 0), Point::Cart{0, 0, 0}));
  BOOST_CHECK(
      approxEqual(::geodeticToEnu(40, 50, 60, 44, 11, 9999),
                  Point::Cart{3079109.5854, 313497.8013, -809786.2658}));
  BOOST_CHECK(
      approxEqual(::geodeticToEnu(-100, -100, -100, -3.14, -1.23, 55),
                  Point::Cart{1098156.3573, -6243102.4693, -5866100.6648}));
  BOOST_CHECK(approxEqual(::geodeticToEnu(9999, 9999, 9999, 9999, 9999, 9999),
                          Point::Cart{0, 0, 0}));
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests::nav_conversions
