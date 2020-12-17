/**
 * \file conversions.hpp
 * General purpose conversion functions between coordinate frames.
 */

#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <vector>
#include <math.h>

#include "nav_const.hpp"

namespace Conversions
{

  /**
   * Convert Euler angles (Heading, Pitch, Roll) to Quaternions. 
   * 
   * @param h Heading (deg)
   * @param p Pitch   (deg)
   * @param r Roll    (deg)
   */
  std::vector<double> hpr_to_quaternion(double h, double p, double r);






}




#endif // CONVERSIONS_HPP