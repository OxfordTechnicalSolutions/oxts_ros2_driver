/**
 * @file convert.hpp
 * @brief General purpose conversion functions between coordinate frames.
 */

#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <vector>
#include <math.h>

#include "ros-driver/nav_const.hpp"


/**
 * @namespace Convert
 * @brief Functions to convert measurements between frames etc.
 */
namespace Convert
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






}




#endif // CONVERSIONS_HPP