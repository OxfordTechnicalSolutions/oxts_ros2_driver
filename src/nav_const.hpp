/**
 * \file nav_const.hpp
 * Basic definitions of constants to be used across codebases
 */

#ifndef NAV_CONST_HPP
#define NAV_CONST_HPP

#include <math.h>


namespace NAV_CONST
{
    const int SECS2NANOSECS =  1e9;
    const double DEG2RADS   =  M_PI / 180.0;
    const double RADS2DEG   =  180.0 / M_PI;
}




#endif // NAV_CONST_HPP