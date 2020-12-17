#ifndef ROS_NCOM_WRAPPER
#define ROS_NCOM_WRAPPER

// Standard includes
#include <string>


// ROS message types includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// OxTS includes
#include "NComRxC.h"
#include "nav_const.hpp"

/*
 * Functions to convert data from the NCom decoder to ROS messages
 */
namespace RosNComWrapper
{
  /*
   * @TODO Add helper function to create headers 
   */

  /*
   * Wrap data from NCom decoder to sensor_msgs/msg/NavSatFix
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  sensor_msgs::msg::NavSatFix wrap_nav_sat_fix(const NComRxC *nrx);
  /*
   * Wrap data from NCom decoder to nav_msgs/msg/Odometry
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  nav_msgs::msg::Odometry     wrap_odometry   (const NComRxC *nrx);
  /*
   * Wrap position data from NCom decoder to std_msgs/msg/String
   * Not really a permanent function, more for easy testing.
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  std_msgs::msg::String       wrap_string     (const NComRxC *nrx);

}




#endif //ROS_NCOM_WRAPPER