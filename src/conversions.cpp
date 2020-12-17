#include "ros-driver/conversions.hpp"



std::vector<double> Conversions::hpr_to_quaternion(double h, double p, double r)
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