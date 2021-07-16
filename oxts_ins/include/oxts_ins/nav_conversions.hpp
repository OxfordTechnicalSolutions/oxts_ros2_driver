/**
 * @file convert.hpp
 * @brief General purpose conversion functions between coordinate frames.
 */

#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <math.h>
#include <vector>

#include "oxts_ins/nav_const.hpp"

namespace Point {

class Point {
private:
  double x0_ = 0;
  double x1_ = 0;
  double x2_ = 0;

public:
  double x0() const;
  double x1() const;
  double x2() const;
  void x0(double x0_);
  void x1(double x1_);
  void x2(double x2_);

  Point() : x0_(0), x1_(0), x2_(0) {}
  Point(double x_0, double x_1, double x_2) : x0_(x_0), x1_(x_1), x2_(x_2) {}

  Point operator+(const Point &p);
  Point operator-(const Point &p);
};

class Cart : protected Point {
public:
  Cart() : Point() {}
  Cart(double x, double y, double z) : Point(x, y, z) {}
  double x() const;
  double y() const;
  double z() const;
  void x(double x);
  void y(double y);
  void z(double z);

  Cart operator+(const Cart &p);
  Cart operator-(const Cart &p);
};

class Geodetic : protected Point {
public:
  Geodetic() : Point() {}
  Geodetic(double lat, double lon, double alt) : Point(lat, lon, alt) {}
  double lat() const;
  double lon() const;
  double alt() const;
  void lat(double x);
  void lon(double y);
  void alt(double z);

  Geodetic operator+(const Geodetic &p);
  Geodetic operator-(const Geodetic &p);
};

} // namespace Point

class Lrf {
private:
  Point::Geodetic origin_;
  double heading_;

public:
  Lrf() : origin_(0.0, 0.0, 0.0), heading_(0.0) {}
  Lrf(double lat, double lon, double alt, double heading)
      : origin_(lat, lon, alt), heading_(heading) {}

  Lrf(Point::Geodetic origin, double heading)
      : origin_(origin), heading_(heading) {}

  void origin(Point::Geodetic origin) { this->origin_ = origin; }
  void origin(double lat, double lon, double alt) {
    this->origin_.lat(lat);
    this->origin_.lon(lon);
    this->origin_.alt(alt);
  }
  void heading(double heading) { this->heading_ = heading; }

  Point::Geodetic origin() { return this->origin_; }
  double lat() { return this->origin_.lat(); }
  double lon() { return this->origin_.lon(); }
  double alt() { return this->origin_.alt(); }
  double heading() { return this->heading_; }
};

/**
 * @namespace NavConversions
 * @brief Functions to convert measurements between frames etc.
 */
namespace NavConversions {
/**
 * @fn hpr_to_quaternion
 * @brief Convert Euler angles (Heading, Pitch, Roll) to Quaternions.
 *
 * @param  h Heading (deg)
 * @param  p Pitch   (deg)
 * @param  r Roll    (deg)
 * @return Quaternion (x, y, z, w)
 */
std::vector<double> hpr_to_quaternion(double h, double p, double r);
/**
 * @fn lla_to_ecef
 * @brief Convert Lat, Long, Alt to ECEF x,y,z
 *
 * @param  lat Latitude in WGS84
 * @param  lon Longitude in WGS84
 * @param  alt Altitude in WGS84
 * @return ECEF (x, y, z)
 */
std::vector<double> lla_to_ecef(double lat, double lon, double alt);
/** Converts WGS-84 Geodetic point (lat, lon, alt) to the
 * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)
 */
Point::Cart GeodeticToEcef(double lat, double lon, double alt);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * (WGS-84) Geodetic point (lat, lon, h).
 */
Point::Geodetic EcefToGeodetic(double x, double y, double z);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt).
 */
Point::Cart EcefToEnu(double x, double y, double z, double lat0, double lon0,
                      double alt);
/** Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt).
 */
Point::Cart EcefToEnu(Point::Cart p, double lat0, double lon0, double alt0);
/** Inverse of EcefToEnu. Converts East-North-Up coordinates
 * (xEast, yNorth, zUp) in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt0) to the
 * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
 */
Point::Cart EnuToEcef(double xEast, double yNorth, double zUp, double lat0,
                      double lon0, double alt0);
/** Converts the East-North-Up coordinates into a local coordinate frame
 *  defined at the same origin, but rotated around the negative z-axis by
 *  ref_heading degrees. Designed for use with the LRF from NCom.
 */
Point::Cart EnuToLrf(double xEast, double yNorth, double zUp,
                     double ref_heading);

/** Converts the geodetic WGS-84 coordinated (lat, lon, h) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, alt0).
 */
Point::Cart GeodeticToEnu(double lat, double lon, double h, double lat0,
                          double lon0, double alt0);
} // namespace NavConversions

#endif // CONVERSIONS_HPP