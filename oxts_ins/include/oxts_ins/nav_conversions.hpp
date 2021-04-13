/**
 * @file convert.hpp
 * @brief General purpose conversion functions between coordinate frames.
 */

#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <vector>
#include <math.h>

#include "oxts_ins/nav_const.hpp"


namespace Point
{

struct Cart
{
  double x = 0;
  double y = 0;
  double z = 0;
};

struct LLA
{
  double lat = 0;
  double lon = 0;
  double alt = 0;
};

}



/**
 * @namespace NavConversions
 * @brief Functions to convert measurements between frames etc.
 */
namespace NavConversions
{
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


  // Converts WGS-84 Geodetic point (lat, lon, h) to the 
  // Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
  Point::Cart GeodeticToEcef(double lat, double lon, double alt);

  // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
  // (WGS-84) Geodetic point (lat, lon, h).
  Point::LLA EcefToGeodetic(double x, double y, double z);

  // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
  // East-North-Up coordinates in a Local Tangent Plane that is centered at the 
  // (WGS-84) Geodetic point (lat0, lon0, alt).
  Point::Cart EcefToEnu(double x, double y, double z,
                        double lat0, double lon0, double alt
                        );

  // Inverse of EcefToEnu. Converts East-North-Up coordinates (xEast, yNorth, zUp) in a
  // Local Tangent Plane that is centered at the (WGS-84) Geodetic point (lat0, lon0, alt)
  // to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
  Point::Cart EnuToEcef(double xEast, double yNorth, double zUp,
                        double lat0, double lon0, double alt
                        );


  Point::Cart EnuToLrf(double xEast, double yNorth, double zUp, double ref_heading);


  // Converts the geodetic WGS-84 coordinated (lat, lon, h) to 
  // East-North-Up coordinates in a Local Tangent Plane that is centered at the 
  // (WGS-84) Geodetic point (lat0, lon0, alt0).
  Point::Cart GeodeticToEnu(double lat, double lon, double h,
                            double lat0, double lon0, double alt0
                            );
}

#endif // CONVERSIONS_HPP