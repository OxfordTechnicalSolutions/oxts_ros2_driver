#include "oxts_ins/nav_conversions.hpp"


namespace Point
{

double Point::x0() const { return x0_; }
double Point::x1() const { return x1_; }
double Point::x2() const { return x2_; }
void   Point::x0(double x_0){this->x0_ = x_0;}
void   Point::x1(double x_1){this->x1_ = x_1;}
void   Point::x2(double x_2){this->x2_ = x_2;}
Point Point::operator+ (const Point& p)
{
  return Point(x0() + p.x0(), x1() + p.x1(), x2() + p.x2());
}

Point Point::operator- (const Point& p)
{
  return Point(x0() - p.x0(), x1() - p.x1(), x2() - p.x2());
}

double Cart::x() const { return x0(); }
double Cart::y() const { return x1(); }
double Cart::z() const { return x2(); }
void   Cart::x(double x){ x0(x); }
void   Cart::y(double y){ x1(y); }
void   Cart::z(double z){ x2(z); }
Cart Cart::operator+ (const Cart& p)
{
  return Cart(x0() + p.x0(), x1() + p.x1(), x2() + p.x2());
}

Cart Cart::operator- (const Cart& p)
{
  return Cart(x0() - p.x0(), x1() - p.x1(), x2() - p.x2());
}

double Geodetic::lat() const { return x0(); }
double Geodetic::lon() const { return x1(); }
double Geodetic::alt() const { return x2(); }
void   Geodetic::lat(double x){ x0(x); }
void   Geodetic::lon(double y){ x1(y); }
void   Geodetic::alt(double z){ x2(z); }
Geodetic Geodetic::operator+ (const Geodetic& p)
{
  return Geodetic(x0() + p.x0(), x1() + p.x1(), x2() + p.x2());
}
Geodetic Geodetic::operator- (const Geodetic& p)
{
  return Geodetic(x0() - p.x0(), x1() - p.x1(), x2() - p.x2());
}

} // namespace Point


std::vector<double> NavConversions::hpr_to_quaternion(double h, double p, double r)
{
  std::vector<double> q(4);  // q.x, q.y, q.z, q.w
  double h_rads, p_rads, r_rads;
  h_rads = h * NAV_CONST::DEG2RADS ;
  p_rads = p * NAV_CONST::DEG2RADS ;
  r_rads = r * NAV_CONST::DEG2RADS ;

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

  double cosLat = std::cos(lat * NAV_CONST::DEG2RADS );
  double sinLat = std::sin(lat * NAV_CONST::DEG2RADS );
  double c = 1/std::sqrt(std::pow(cosLat,2) 
                              + (NAV_CONST::FLAT_FACTOR2 * std::pow(sinLat,2)));
  double s = c * NAV_CONST::FLAT_FACTOR2;

  posEcef[0] = ((NAV_CONST::EARTH_RADIUS * c) + alt) 
                                   * cosLat * std::cos(NAV_CONST::DEG2RADS *lon);
  posEcef[1] = ((NAV_CONST::EARTH_RADIUS * c) + alt) 
                                   * cosLat * std::sin(NAV_CONST::DEG2RADS *lon);
  posEcef[2] = ((NAV_CONST::EARTH_RADIUS * s) + alt) * sinLat;

  return posEcef;
}


Point::Cart NavConversions::GeodeticToEcef(double lat, double lon, double alt)
{
  Point::Cart p_ecef;
  // Calculate parameters
  double lambda = NAV_CONST::DEG2RADS * (lat);
  double phi = NAV_CONST::DEG2RADS * (lon);
  double sin_lambda = std::sin(lambda);
  double cos_lambda = std::cos(lambda);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);
  double N = NAV_CONST::EARTH_RADIUS / 
             std::sqrt(1 - NAV_CONST::ECC2 * sin_lambda * sin_lambda);
  // Calculate the position in ECEF
  p_ecef.x((alt + N) * cos_lambda * cos_phi);
  p_ecef.y((alt + N) * cos_lambda * sin_phi);
  p_ecef.z((alt + (1 - NAV_CONST::ECC2) * N) * sin_lambda);

  return p_ecef;
}


Point::Geodetic NavConversions::EcefToGeodetic(double x, double y, double z)
{
  Point::Geodetic p_geo;

  double eps = NAV_CONST::ECC2 / (1.0 - NAV_CONST::ECC2);
  double p = std::sqrt(x * x + y * y);
  double q = std::atan2((z * NAV_CONST::EARTH_RADIUS), 
                        (p * NAV_CONST::EARTH_SEMI_MINOR));
  double sin_q = std::sin(q);
  double cos_q = std::cos(q);
  double sin_q_3 = sin_q * sin_q * sin_q;
  double cos_q_3 = cos_q * cos_q * cos_q;
  double phi = std::atan2((z + eps * NAV_CONST::EARTH_SEMI_MINOR * sin_q_3), 
                      (p - NAV_CONST::ECC2 *NAV_CONST::EARTH_RADIUS* cos_q_3));
  double lambda = std::atan2(y, x);
  double v = NAV_CONST::EARTH_RADIUS / 
              std::sqrt(1.0 - NAV_CONST::ECC2 * std::sin(phi) * std::sin(phi));

  p_geo.alt((p / std::cos(phi)) - v);
  p_geo.lat(NAV_CONST::RADS2DEG * (phi));
  p_geo.lon(NAV_CONST::RADS2DEG * (lambda));

  return p_geo;
}



Point::Cart NavConversions::EcefToEnu(Point::Cart p, double lat0, double lon0, double alt0)
{
  Point::Cart p_enu;
  // Convert to radians in notation consistent with the paper:
  double lambda = NAV_CONST::DEG2RADS * lat0;
  double phi = NAV_CONST::DEG2RADS * lon0;
  double sin_lambda = std::sin(lambda);
  double cos_lambda = std::cos(lambda);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  // Calculate the position of the reference point in ECEF
  Point::Cart ecef0 = NavConversions::GeodeticToEcef(lat0, lon0, alt0);
  // Translate the position by the reference point to align the ECEF data to 
  // the ENU frame
  Point::Cart p_d;
  p_d = p - ecef0;

  // Matrix multiplication (x = East, y = North, z = Up) to rotate the data 
  // into the ENU frame.
  p_enu.x(-sin_phi * p_d.x() + cos_phi * p_d.y());
  p_enu.y(-cos_phi * sin_lambda * p_d.x() - sin_lambda * sin_phi * p_d.y() + cos_lambda * p_d.z());
  p_enu.z(cos_lambda * cos_phi * p_d.x() + cos_lambda * sin_phi * p_d.y() + sin_lambda * p_d.z());

  return p_enu;
}

Point::Cart NavConversions::EcefToEnu(double x, double y, double z,
                                      double lat0, double lon0, double alt0
                                      )
{
  Point::Cart p(x,y,z);
  return NavConversions::EcefToEnu(p, lat0, lon0, alt0);
}

Point::Cart NavConversions::EnuToEcef(double xEast, double yNorth, double zUp,
                                      double lat0, double lon0, double alt0
                                      )
{
  Point::Cart p_ecef;
  // Calculate parameters
  double lambda = NAV_CONST::DEG2RADS * lat0;
  double phi = NAV_CONST::DEG2RADS * lon0;
  double sin_lambda = std::sin(lambda);
  double cos_lambda = std::cos(lambda);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);
  // Calculate the position of the reference point in ECEF
  Point::Cart ecef0 = NavConversions::GeodeticToEcef(lat0, lon0, alt0);
  // Rotate the ENU position into the orientation of the ECEF frame
  double xd = -sin_phi * xEast - cos_phi * sin_lambda * yNorth + cos_lambda * cos_phi * zUp;
  double yd = cos_phi * xEast - sin_lambda * sin_phi * yNorth + cos_lambda * sin_phi * zUp;
  double zd = cos_lambda * yNorth + sin_lambda * zUp;
  // Add the translation to the rotated ENU position to get the position in ECEF
  p_ecef.x(xd + ecef0.x());
  p_ecef.y(yd + ecef0.y());
  p_ecef.z(zd + ecef0.z());

  return p_ecef;
}

Point::Cart NavConversions::EnuToLrf(double xEast, double yNorth, double zUp, 
                                     double ref_heading)
{
  Point::Cart p_lrf;
  // theta is the ref_heading angle in the enu frame
  double theta = (90.0 - ref_heading) * NAV_CONST::DEG2RADS;

  p_lrf.x(xEast * std::cos(theta) - yNorth * std::sin(theta));
  p_lrf.y(xEast * std::sin(theta) + yNorth * std::cos(theta));
  p_lrf.z(zUp);

  return p_lrf;
}


Point::Cart NavConversions::GeodeticToEnu(double lat, double lon, double alt,
                                          double lat0, double lon0, double alt0
                                          )
{
  Point::Cart p_ecef = NavConversions::GeodeticToEcef(lat, lon, alt);
  Point::Cart p_enu = NavConversions::EcefToEnu(p_ecef.x(), 
                                                p_ecef.y(), 
                                                p_ecef.z(), 
                                                lat0, 
                                                lon0, 
                                                alt0
                                                );

  return p_enu;
}