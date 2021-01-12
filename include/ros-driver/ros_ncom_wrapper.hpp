/**
 * \file ros_ncom_wrapper.hpp
 * Functions to wrap NCom data in ROS messages
 */

#ifndef ROS_NCOM_WRAPPER
#define ROS_NCOM_WRAPPER

// Standard includes
#include <string>
#include <vector>
#include <cmath>

// ROS message types includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// OxTS includes
#include "nav/NComRxC.h"
#include "ros-driver/nav_const.hpp"
#include "ros-driver/convert.hpp"

/**
 * Functions to convert data from the NCom decoder to ROS messages
 */
namespace RosNComWrapper
{
  /**
   * Convert NCom time to a ROS friendly time format
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  rclcpp::Time       ncom_time_to_time(const NComRxC *nrx);
  /**
   * Wrap data from NCom decoder to std_msgs/msg/Header (for use in other msgs)
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  std_msgs::msg::Header              wrap_header_ncom_time(const NComRxC *nrx);
  /**
   * Wrap data into ROS header format.
   * 
   * Does not strictly belong here since it is not encoding NCom data. Move when
   * appropriate alternative location makes itself known.
   * 
   * @param time Timestamp to be added to the packet
   * @param frame frame_id of the message
   */
  std_msgs::msg::Header              wrap_header     (rclcpp::Time time, 
                                                      std::string frame);
  /** 
   * Wrap data from NCom decoder to std_msgs/msg/NavSatStatus
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  sensor_msgs::msg::NavSatStatus     wrap_nav_sat_status(const NComRxC *nrx);
  /**
   * Wrap data from NCom decoder to sensor_msgs/msg/NavSatFix
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   */
  sensor_msgs::msg::NavSatFix        wrap_nav_sat_fix(const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap data from NCom decoder to nav_msgs/msg/Odometry
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   */
  nav_msgs::msg::Odometry            wrap_odometry (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap position data from NCom decoder to std_msgs/msg/String
   * Not really a permanent function, more for easy testing.
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  std_msgs::msg::String              wrap_string   (const NComRxC *nrx); 
  /**
   * Wrap IMU data from NCom decoder to sensor_msgs/msg/Imu
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   * \todo Covariances
   * \todo Validate Quaternion conversions
   */
  sensor_msgs::msg::Imu              wrap_imu      (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap velocity data from NCom decoder to sensor_msgs/msg/Imu
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   * @returns Linear velocity forward, lateral, down. 
   *          Angular velocity forward, lateral, down.
   */
  geometry_msgs::msg::TwistStamped   wrap_velocity (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap time data from NCom decoder to sensor_msgs/msg/TimeReference
   * 
   * The time reference depends on the presence of GNSS signal.
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   */
  sensor_msgs::msg::TimeReference wrap_time_reference (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap tf data from NCom decoder to sensor_msgs/msg/TimeReference
   * 
   * Transform from global ECEF to IMU
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   */
  geometry_msgs::msg::TransformStamped wrap_tf2    (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
}




#endif //ROS_NCOM_WRAPPER