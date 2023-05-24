// Copyright 2021 Oxford Technical Solutions Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file ros_ncom_wrapper.hpp
 * Functions to wrap NCom data in ROS messages
 */

#ifndef ROS_NCOM_WRAPPER
#define ROS_NCOM_WRAPPER

// Standard includes
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// ROS message types includes
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <oxts_msgs/msg/imu_bias.hpp>
#include <oxts_msgs/msg/lever_arm.hpp>
#include <oxts_msgs/msg/nav_sat_ref.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

// OxTS includes
#include "oxts_ins/NComRxC.h"
#include "oxts_ins/nav_const.hpp"
#include "oxts_ins/nav_conversions.hpp"

/**
 * Functions to convert data from the NCom decoder to ROS messages
 */
namespace RosNComWrapper {

/**
 * Calculate the rotational component of the transform from frame1 to frame2
 * based on Euler angles
 *
 * @param nrx Pointer to the decoded NCom data
 * @return A quaternion representing the rotation between vehicle frame and imu
 * frame
 */
tf2::Quaternion getVat(const NComRxC *nrx);
/**
 * Calculate the lateral no slip component of the transform from frame1 to
 * frame2
 *
 * @param nrx Pointer to the decoded NCom data
 * @return A transform representing the translation between the vehicle rear
 * axle and the imu frame
 */
tf2::Vector3 getNsp(const NComRxC *nrx);
/**
 * Get the NCOM orientation in body (IMU) frame
 *
 * @param nrx Pointer to the decoded NCom data
 * @return A quaterntion representing the rotation between the vehicle frame and
 * NED
 */
tf2::Quaternion getBodyRPY(const NComRxC *nrx);
/**
 * Get the NCOM orientation in vehicle frame
 *
 * @param nrx Pointer to the decoded NCom data
 * @return A quaterntion representing the rotation between the vehicle frame and
 * NED
 */
tf2::Quaternion getVehRPY(const NComRxC *nrx);
/**
 * Get the LRF from the NCOM decoder
 */
Lrf getNcomLrf(const NComRxC *nrx);
/**
 * Wrap data from NCom decoder to std_msgs/msg/NavSatStatus
 *
 * @param nrx Pointer to the decoded NCom data
 */
sensor_msgs::msg::NavSatStatus nav_sat_status(const NComRxC *nrx);
/**
 * Wrap data from NCom decoder to sensor_msgs/msg/NavSatFix
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 */
sensor_msgs::msg::NavSatFix nav_sat_fix(const NComRxC *nrx,
                                        std_msgs::msg::Header head);
/**
 * Wrap data for local reference point using oxts_msgs/msg/NavSatRef
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 */
oxts_msgs::msg::NavSatRef nav_sat_ref(const Lrf lrf,
                                      std_msgs::msg::Header head);

/**
 * @brief Wrap data for GAP (primary GPS to IMU offset) using
 * oxts_msgs/msg/LeverArm
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 * @return oxts_msgs::msgs::LeverArm
 */
oxts_msgs::msg::LeverArm lever_arm_gap(const NComRxC *nrx,
                                       std_msgs::msg::Header head);

/**
 * @brief Wrap data for IMU biases (accelerometer and gyro)
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 * @return oxts_msgs::msg::ImuBias
 */
oxts_msgs::msg::ImuBias imu_bias(const NComRxC *nrx,
                                 std_msgs::msg::Header head);

/**
 * Wrap data from the NCom decoder to
 * geometry_msgs/msg/PointStamped.
 *
 * @param nrx Pointer to the decoded NCom data.
 * @param head Header to be added to the published message.
 * @return Position of the IMU in the ECEF coordinate frame.
 */
geometry_msgs::msg::PointStamped ecef_pos(const NComRxC *nrx,
                                          std_msgs::msg::Header head);
/**
 * Wrap position data from NCom decoder to std_msgs/msg/String
 * Not really a permanent function, more for easy testing.
 *
 * @param nrx Pointer to the decoded NCom data
 */
std_msgs::msg::String string(const NComRxC *nrx);
/**
 * Wrap IMU data from NCom decoder to sensor_msgs/msg/Imu
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 *
 * \todo Covariances
 */
sensor_msgs::msg::Imu imu(const NComRxC *nrx, std_msgs::msg::Header head);
/**
 * Wrap velocity data from NCom decoder to sensor_msgs/msg/Imu
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 *
 * @returns Linear velocity forward, lateral, down.
 *          Angular velocity forward, lateral, down.
 */
geometry_msgs::msg::TwistStamped velocity(const NComRxC *nrx,
                                          std_msgs::msg::Header head);
/**
 * Wrap navigation data from NCom decoder to nav_msgs/msg/Odometry
 *
 * @param nrx Pointer to the decoded NCom data
 * @param head Header to be added to the published message
 * @returns
 */
nav_msgs::msg::Odometry odometry(const NComRxC *nrx,
                                 const std_msgs::msg::Header &head, Lrf lrf);
/**
 * Wrap navigation data from NCom decoder to nav_msgs/msg/Path
 *
 * @param poses Reference to the vector of poses
 * @param head Header to be added to the published message
 * @returns
 */
nav_msgs::msg::Path
path(const std::vector<geometry_msgs::msg::PoseStamped> &poses,
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
sensor_msgs::msg::TimeReference time_reference(const NComRxC *nrx,
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
geometry_msgs::msg::TransformStamped tf2(const NComRxC *nrx,
                                         std_msgs::msg::Header head);
} // namespace RosNComWrapper

#endif // ROS_NCOM_WRAPPER
