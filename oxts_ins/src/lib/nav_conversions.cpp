#include "oxts_ins/nav_conversions.hpp"



std::vector<double> NavConversions::hpr_to_quaternion(double h, double p, double r)
{
  std::vector<double> q(4);  // q.x, q.y, q.z, q.w
  double h_rads, p_rads, r_rads;
  h_rads = h * NAV_CONST::DEG2RADS;
  p_rads = p * NAV_CONST::DEG2RADS;
  r_rads = r * NAV_CONST::DEG2RADS;

  q[0] = (std::sin(r_rads/2) * std::cos(p_rads/2) * std::cos(h_rads/2)) 
       - (std::cos(r_rads/2) * std::sin(p_rads/2) * std::sin(h_rads/2));
  q[1] = (std::cos(r_rads/2) * std::sin(p_rads/2) * std::cos(h_rads/2)) 
       + (std::sin(r_rads/2) * std::cos(p_rads/2) * std::sin(h_rads/2));
  q[2] = (std::cos(r_rads/2) * std::cos(p_rads/2) * std::sin(h_rads/2)) 
       - (std::sin(r_rads/2) * std::sin(p_rads/2) * std::cos(h_rads/2));
  q[3] = (std::cos(r_rads/2) * std::cos(p_rads/2) * std::cos(h_rads/2)) 
       + (std::sin(r_rads/2) * std::sin(p_rads/2) * std::sin(h_rads/2));

  return q;
}


std::vector<double> NavConversions::lla_to_ecef(double lat, double lon, double alt)
{
  std::vector<double> posEcef(3);

  double cosLat = std::cos(lat * NAV_CONST::DEG2RADS);
  double sinLat = std::sin(lat * NAV_CONST::DEG2RADS);
  double c = 1/std::sqrt(std::pow(cosLat,2) 
                              + (NAV_CONST::FLAT_FACTOR2 * std::pow(sinLat,2)));
  double s = c * NAV_CONST::FLAT_FACTOR2;

  posEcef[0] = ((NAV_CONST::EARTH_RADIUS * c) + alt) 
                                   * cosLat * std::cos(NAV_CONST::DEG2RADS*lon);
  posEcef[1] = ((NAV_CONST::EARTH_RADIUS * c) + alt) 
                                   * cosLat * std::sin(NAV_CONST::DEG2RADS*lon);
  posEcef[2] = ((NAV_CONST::EARTH_RADIUS * s) + alt) * sinLat;

  return posEcef;
}