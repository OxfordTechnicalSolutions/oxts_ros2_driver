/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */

#ifndef TESTS_H
#define TESTS_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "oxts_driver/driver.hpp"
#include "oxts_ins/nav_conversions.hpp"

namespace tests {

/**
 * @return A new OxtsDriver for use in the unit tests (i.e. configured to use an
 * .ncom file).
 */
std::unique_ptr<oxts_driver::OxtsDriver> newDriver();

constexpr double DEFAULT_DELTA = 0.001;

/** ≅ function because of floating point imprecision (and to make the check
 * messages more readable when they're printed via ./test_program -l all).
 */
bool approxEqual(double l, double r, double delta);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const std::vector<double> &l, const std::vector<double> &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const Point::Cart &l, const Point::Cart &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const Point::Geodetic &l, const Point::Geodetic &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const tf2::Quaternion &l, const tf2::Quaternion &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const Lrf &l, const Lrf &r, double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const geometry_msgs::msg::Point &l,
                 const geometry_msgs::msg::Point &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const geometry_msgs::msg::Vector3 &l,
                 const geometry_msgs::msg::Vector3 &r,
                 double delta = DEFAULT_DELTA);

//! @copydoc approxEqual(double, double, double)
bool approxEqual(const geometry_msgs::msg::Quaternion &l,
                 const geometry_msgs::msg::Quaternion &r,
                 double delta = DEFAULT_DELTA);

} // namespace tests

#endif // TESTS_H