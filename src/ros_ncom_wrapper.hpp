/**
 * \file ros_ncom_wrapper.hpp
 * Functions to wrap NCom data in ROS messages
 */

#ifndef ROS_NCOM_WRAPPER
#define ROS_NCOM_WRAPPER

// Standard includes
#include <string>
#include <vector>


// ROS message types includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

// OxTS includes
#include "NComRxC.h"
#include "nav_const.hpp"
#include "conversions.hpp"

/**
 * Functions to convert data from the NCom decoder to ROS messages
 */
namespace RosNComWrapper
{
  /**
   * \todo Add helper function to create headers 
   */

  /**
   * Wrap data from NCom decoder to sensor_msgs/msg/NavSatFix
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  sensor_msgs::msg::NavSatFix wrap_nav_sat_fix(const NComRxC *nrx);
  /**
   * Wrap data from NCom decoder to nav_msgs/msg/Odometry
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  nav_msgs::msg::Odometry     wrap_odometry   (const NComRxC *nrx);
  /**
   * Wrap position data from NCom decoder to std_msgs/msg/String
   * Not really a permanent function, more for easy testing.
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  std_msgs::msg::String       wrap_string     (const NComRxC *nrx); 
  /**
   * Wrap IMU data from NCom decoder to sensor_msgs/msg/Imu
   * 
   * @param nrx Pointer to the decoded NCom data
   * 
   * \todo Covariances
   * \todo Validate Quaternion conversions
   */
  sensor_msgs::msg::Imu       wrap_imu        (const NComRxC *nrx);
}




#endif //ROS_NCOM_WRAPPER