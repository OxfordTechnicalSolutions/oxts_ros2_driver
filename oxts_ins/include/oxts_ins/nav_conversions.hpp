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

/**
 * @file convert.hpp
 * @brief General purpose conversion functions between coordinate frames.
 */

#ifndef NAV_CONVERSIONS_HPP
#define NAV_CONVERSIONS_HPP

#include <math.h>
#include <vector>

#include "oxts_ins/nav_const.hpp"

namespace Point
{

class Point
{
private:
  double x0_ = 0;
  double x1_ = 0;
  double x2_ = 0;

public:
  [[nodiscard]] double x0() const;
  [[nodiscard]] double x1() const;
  [[nodiscard]] double x2() const;
  void x0(double x0_);
  void x1(double x1_);
  void x2(double x2_);

  Point()
  : x0_(0), x1_(0), x2_(0) {}
  Point(double x_0, double x_1, double x_2)
  : x0_(x_0), x1_(x_1), x2_(x_2) {}

  Point operator+(const Point & p) const;
  Point operator-(const Point & p) const;
};

class Cart : protected Point
{
public:
  Cart()
  : Point() {}
  Cart(double x, double y, double z)
  : Point(x, y, z) {}
  [[nodiscard]] double x() const;
  [[nodiscard]] double y() const;
  [[nodiscard]] double z() const;
  void x(double x);
  void y(double y);
  void z(double z);

  Cart operator+(const Cart & p);
  Cart operator-(const Cart & p);
};

class Geodetic : protected Point
{
public:
  Geodetic()
  : Point() {}
  Geodetic(double lat, double lon, double alt)
  : Point(lat, lon, alt) {}
  [[nodiscard]] double lat() const;
  [[nodiscard]] double lon() const;
  [[nodiscard]] double alt() const;
  void lat(double x);
  void lon(double y);
  void alt(double z);

  Geodetic operator+(const Geodetic & p);
  Geodetic operator-(const Geodetic & p);
};

} // namespace Point

class Lrf
{
private:
  Point::Geodetic origin_;
  double heading_;

public:
  Lrf()
  : origin_(0.0, 0.0, 0.0), heading_(0.0) {}
  Lrf(double lat, double lon, double alt, double heading)
  : origin_(lat, lon, alt), heading_(heading) {}

  Lrf(Point::Geodetic origin, double heading)
  : origin_(origin), heading_(heading) {}

  void origin(Point::Geodetic origin) {this->origin_ = origin;}
  void origin(double lat, double lon, double alt)
  {
    this->origin_.lat(lat);
    this->origin_.lon(lon);
    this->origin_.alt(alt);
  }
  void heading(double heading) {this->heading_ = heading;}

  Point::Geodetic origin() {return this->origin_;}
  [[nodiscard]] double lat() const {return this->origin_.lat();}
  [[nodiscard]] double lon() const {return this->origin_.lon();}
  [[nodiscard]] double alt() const {return this->origin_.alt();}
  [[nodiscard]] double heading() const {return this->heading_;}
};

/**
 * @namespace NavConversions
 * @brief Functions to convert measurements between frames etc.
 */
namespace NavConversions
{
/**
 * @fn hprToQuaternion
 * @brief Convert Euler angles (Heading, Pitch, Roll) to Quaternions.
 *
 * @param  h Heading (deg)
 * @param  p Pitch   (deg)
 * @param  r Roll    (deg)
 * @return Quaternion (x, y, z, w)
 */
std::vector<double> hprToQuaternion(double h, double p, double r);
/**
 * @fn llaToEcef
 * @brief Convert Lat, Long, Alt to ECEF x,y,z
 *
 * @param  lat Latitude in WGS84
 * @param  lon Longitude in WGS84
 * @param  alt Altitude in WGS84
 * @return ECEF (x, y, z)
 */
std::vector<double> llaToEcef(double lat, double lon, double alt);
/** Converts WGS-84 Geodetic point (lat, lon, alt) to the
 * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)
 */
Point::Cart geodeticToEcef(double lat, double lon, double alt);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * (WGS-84) Geodetic point (lat, lon, h).
 */
Point::Geodetic ecefToGeodetic(double x, double y, double z);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt).
 */
Point::Cart ecefToEnu(
  double x, double y, double z, double lat0, double lon0,
  double alt);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt).
 */
Point::Cart ecefToEnu(Point::Cart p, double lat0, double lon0, double alt0);
/** Inverse of ecefToEnu. Converts East-North-Up coordinates
 * (xEast, yNorth, zUp) in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt0) to the
 * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
 */
Point::Cart enuToEcef(
  double xEast, double yNorth, double zUp, double lat0,
  double lon0, double alt0);
/** Converts the East-North-Up coordinates into a local coordinate frame
 *  defined at the same origin, but rotated around the negative z-axis by
 *  ref_heading degrees. Designed for use with the LRF from NCom.
 */
Point::Cart enuToLrf(
  double xEast, double yNorth, double zUp,
  double ref_heading);

/** Converts the geodetic WGS-84 coordinated (lat, lon, h) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt0).
 */
Point::Cart geodeticToEnu(
  double lat, double lon, double h, double lat0,
  double lon0, double alt0);
} // namespace NavConversions

#endif // NAV_CONVERSIONS_HPP
