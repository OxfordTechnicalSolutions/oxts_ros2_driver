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
#include <iostream>

// ROS message types includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2_kdl/tf2_kdl.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
   * Calculate the rotational component of the transform from frame1 to frame2
   * based on Euler angles
   * 
   * @param nrx Pointer to the decoded NCom data
   * @return A transform which can be applied to data in the vehicle frame to 
   *         convert it to the imu frame. Timestamp is not used.
   * 
   * @todo Translation component?
   */
  tf2::Quaternion wrap_vat_to_quaternion(const NComRxC *nrx);
  /**
   * Convert NCom time to a ROS friendly time format. Does not convert to ROS
   * time, only the format.
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  rclcpp::Time       ncom_time_to_time(const NComRxC *nrx);
  /**
   * Wrap data into ROS header format.
   * 
   * Does not strictly belong here since it is not encoding NCom data. Move when
   * appropriate alternative location makes itself known.
   * 
   * @param time Timestamp to be added to the packet
   * @param frame frame_id of the message
   */
  std_msgs::msg::Header wrap_header(rclcpp::Time time, std::string frame);
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
   * Wrap data from the NCom decoder to 
   * geometry_msgs/msg/PoseWithCovarianceStamped. 
   * 
   * @param nrx Pointer to the decoded NCom data.
   * @param head Header to be added to the published message.
   * @return Pose of the IMU in the ECEF coordinate frame.
   */
  geometry_msgs::msg::PoseWithCovarianceStamped wrap_pose_ecef
                                                (
                                                const NComRxC *nrx,
                                                std_msgs::msg::Header head
                                                );
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