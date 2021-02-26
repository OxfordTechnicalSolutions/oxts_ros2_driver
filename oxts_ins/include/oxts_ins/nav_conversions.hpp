/**
 * @file convert.hpp
 * @brief General purpose conversion functions between coordinate frames.
 */

#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <vector>
#include <math.h>

#include "oxts_ins/nav_const.hpp"


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
}




#endif // CONVERSIONS_HPP