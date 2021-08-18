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
 * \file convert.hpp
 * Defines node to take NCom data and publish it in ROS messages.
 */

#ifndef OXTS_INS__INS_HPP_
#define OXTS_INS__INS_HPP_

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

// ROS includes
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose_with_covariance.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <oxts_msgs/msg/ncom.hpp>
#include <tf2_ros/transform_broadcaster.h>
// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "oxts_ins/NComRxC.h"
#include "oxts_ins/nav_const.hpp"
#include "oxts_ins/nav_conversions.hpp"
#include "oxts_ins/wrapper.hpp"

using std::placeholders::_1;

namespace oxts_ins {

/**
 * Enumeration of LRF sources
 */
enum LRF_SOURCE {
  /** Use the LRF from NCom. */
  NCOM_LRF = 0,
  /** Use the position and heading of first NCom packet as LRF. */
  NCOM_FIRST = 1,
  /** Use the position of first NCom packet as origin of LRF. Align to ENU */
  NCOM_FIRST_ENU = 2
};

/**
 * This class creates a subclass of Node designed to take NCom data from the
 * NCom decoder and publish it to pre-configured ROS topics.
 *
 * @todo Add config struct to hold data which will hold config parsed from the
 *       .yaml file.
 */
class OxtsIns : public rclcpp::Node {
private:
  /*! Rate at which to sample NCom. Expected that this will typically match
    the rate of NCom itself, though can be set lower to save computation. */
  uint8_t ncom_rate;
  /*! Local reference frame source
    {0 : From NCom LRF, 1 : First NCom position} */
  uint8_t lrf_source;
  /*! Frame ID of outgoing packets. @todo Having a general frame ID may not
    make sense. This isn't implemented. */
  std::string frame_id;
  /*! Publishing rate for debug String message. */
  uint8_t pub_string_rate;
  /*! Publishing rate for NavSatFix message. */
  uint8_t pub_nav_sat_fix_rate;
  /*! Publishing rate for Velocity message. */
  uint8_t pub_velocity_rate;
  /*! Publishing rate for Odometry message. */
  uint8_t pub_odometry_rate;
  /*! Frame ID for Odometry message. */
  std::string pub_odometry_frame_id;
  /*! Publishing rate for Path message. */
  uint8_t pub_path_rate;
  /*! Historical data for Path message. */
  std::vector<geometry_msgs::msg::PoseStamped> past_poses;
  /*! Publishing rate for TimeReference message.*/
  uint8_t pub_time_reference_rate;
  /*! Publishing rate for PointStamped message. */
  uint8_t pub_ecef_pos_rate;
  /*! Publishing rate for PointStamped message. */
  uint8_t pub_nav_sat_ref_rate;
  /*! Flag to enable publishing of Imu message. */
  bool pub_imu_flag;
  /*! Flag to enable publishing of Tf messages. */
  bool pub_tf_flag;
  /*! Publishing rate for lever arm message. */
  uint8_t pub_lever_arm_rate;
  /*! Publishing rate for IMU Bias message. */
  uint8_t pub_imu_bias_rate;

  uint8_t pubStringInterval;
  uint8_t pubNavSatFixInterval;
  uint8_t pubVelocityInterval;
  uint8_t pubOdometryInterval;
  uint8_t pubPathInterval;
  uint8_t pubTimeReferenceInterval;
  uint8_t pubEcefPosInterval;
  uint8_t pubNavSatRefInterval;
  uint8_t pubLeverArmInterval;
  uint8_t pubIMUBiasInterval;

  std::string topic_prefix;
  std::string ncom_topic;
  std::string string_topic;
  std::string nav_sat_fix_topic;
  std::string velocity_topic;
  std::string odometry_topic;
  std::string path_topic;
  std::string time_reference_topic;
  std::string ecef_pos_topic;
  std::string nav_sat_ref_topic;
  std::string lever_arm_topic;
  std::string imu_bias_topic;
  std::string imu_topic;
  // ...

  void ncomCallbackRegular(const oxts_msgs::msg::Ncom::SharedPtr msg);
  /** Callback function for debug String message. Wraps message, publishes, and
   *  prints some information to the console.*/
  void string();
  /** Callback function for NavSatFix message. Wraps message and publishes. */
  void nav_sat_fix(std_msgs::msg::Header header);
  /** Callback function for Imu message. Wraps message and publishes. */
  void imu(std_msgs::msg::Header header);
  /** Callback function for Tf messages. Wraps messages and broadcasts. */
  void tf(const std_msgs::msg::Header &header);
  /** Callback function for TimeReference message. Wraps message and publishes.
   */
  void time_reference(std_msgs::msg::Header header);
  /** Callback function for Velocity message. Wraps message and publishes. */
  void velocity(std_msgs::msg::Header header);
  /** Callback function for Odometry message. Wraps message and publishes. */
  void odometry(std_msgs::msg::Header header);
  /** Callback function for Path message. Wraps message and publishes. */
  void path(std_msgs::msg::Header header);
  /** Callback function for PointStamped message. Wraps message and
   *  publishes.*/
  void ecef_pos(std_msgs::msg::Header header);
  /** Callback function for OxTS NavSatRef message. Wraps message and
   *  publishes.*/
  void nav_sat_ref(std_msgs::msg::Header header);
  /** Callback function for OxTS GAP lever arm message. Wraps message and
   *  publishes.*/
  void lever_arm_gap(std_msgs::msg::Header header);
  /** Callback function for OxTS IMU Bias message. Wraps message and publishes.
   */
  void imu_bias(std_msgs::msg::Header header);
  /** Get the LRF from configured source */
  void getLrf();

  /**  Subscriber for oxts_msgs/msg/NCom. */
  rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr subNCom_;
  /** Publisher for std_msgs/msg/string. Only used for debugging, currently
   *  outputs lat, long, alt in string form. */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubString_;
  /** Publisher for /sensor_msgs/msg/NavSatFix */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNavSatFix_;
  /** Publisher for /sensor_msgs/msg/Imu */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_;
  /** Publisher for /sensor_msgs/msg/TwistStamped */
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubVelocity_;
  /** Publisher for /nav_msgs/msg/Odometry */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry_;
  /** Publisher for /nav_msgs/msg/Path */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
  /** Publisher for /sensor_msgs/msg/TimeReference */
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr
      pubTimeReference_;
  /** Publisher for /geometry_msgs/msg/PointStamped */
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubEcefPos_;
  /** Publisher for /oxts_msgs/msg/NavSatRef */
  rclcpp::Publisher<oxts_msgs::msg::NavSatRef>::SharedPtr pubNavSatRef_;
  /** Publsuher for /oxts_msgs/msg/LeverArm */
  rclcpp::Publisher<oxts_msgs::msg::LeverArm>::SharedPtr pubLeverArm_;
  /** Publisher for /oxts_msgs/msg/ImuBias */
  rclcpp::Publisher<oxts_msgs::msg::ImuBias>::SharedPtr pubIMUBias_;
  /** TF broadcaster */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  /** Error strings */
  std::string notFactorError = " publish rate is not a factor of NCom rate, "
                               "true output rate would not be %dHz.";

public:
  /**
   * Default constructor for the OxtsIns. Parses options from the
   * .yaml params/config file, sets up UDP connection to unit.
   */
  explicit OxtsIns(const rclcpp::NodeOptions &options)
      : Node("oxts_ins", options) {
    // Get parameters (from config, command line, or from default)
    // Initialise configurable parameters (all params should have defaults)
    ncom_rate = this->declare_parameter("ncom_rate", 100);
    frame_id = this->declare_parameter("frame_id", "oxts_link");
    pub_string_rate = this->declare_parameter("pub_string_rate", 0);
    pub_nav_sat_fix_rate = this->declare_parameter("pub_nav_sat_fix_rate", 0);
    pub_imu_flag = this->declare_parameter("pub_imu_flag", true);
    pub_velocity_rate = this->declare_parameter("pub_velocity_rate", 0);
    pub_odometry_rate = this->declare_parameter("pub_odometry_rate", 0);
    pub_odometry_frame_id =
        this->declare_parameter("pub_odometry_frame_id", "map");
    pub_path_rate = this->declare_parameter("pub_path_rate", 0);
    pub_time_reference_rate =
        this->declare_parameter("pub_time_reference_rate", 0);
    pub_ecef_pos_rate = this->declare_parameter("pub_ecef_pos_rate", 0);
    pub_nav_sat_ref_rate = this->declare_parameter("pub_nav_sat_ref_rate", 0);
    pub_tf_flag = this->declare_parameter("pub_tf_flag", true);
    lrf_source = this->declare_parameter("lrf_source", 0);
    pub_lever_arm_rate = this->declare_parameter("pub_lever_arm_rate", 0);
    pub_imu_bias_rate = this->declare_parameter("pub_imu_bias_rate", 0);
    topic_prefix = this->declare_parameter("topic_prefix", "ins");
    ncom_topic = this->declare_parameter("ncom_topic", "ncom");
    string_topic = this->declare_parameter("string_topic", "debug_string_pos");
    nav_sat_fix_topic =
        this->declare_parameter("nav_sat_fix_topic", "nav_sat_fix");
    velocity_topic = this->declare_parameter("velocity_topic", "velocity");
    odometry_topic = this->declare_parameter("odometry_topic", "odometry");
    path_topic = this->declare_parameter("path_topic", "path");
    time_reference_topic =
        this->declare_parameter("time_reference_topic", "time_reference");
    ecef_pos_topic = this->declare_parameter("ecef_pos_topic", "ecef_pos");
    nav_sat_ref_topic =
        this->declare_parameter("nav_sat_ref_topic", "nav_sat_ref");
    lever_arm_topic = this->declare_parameter("lever_arm_topic", "lever_arm");
    imu_bias_topic = this->declare_parameter("imu_bias_topic", "imu_bias");
    imu_topic = this->declare_parameter("imu_topic", "imu");

    /** @todo Improve error handling */
    if (ncom_rate == 0) {
      RCLCPP_ERROR(
          this->get_logger(),
          "NCom publish rate is set to zero. No messages will be output.");
      return;
    }

    pubStringInterval =
        (pub_string_rate == 0) ? 0 : ncom_rate / pub_string_rate;
    pubNavSatFixInterval =
        (pub_nav_sat_fix_rate == 0) ? 0 : ncom_rate / pub_nav_sat_fix_rate;
    pubVelocityInterval =
        (pub_velocity_rate == 0) ? 0 : ncom_rate / pub_velocity_rate;
    pubOdometryInterval =
        (pub_odometry_rate == 0) ? 0 : ncom_rate / pub_odometry_rate;
    pubPathInterval = (pub_path_rate == 0) ? 0 : ncom_rate / pub_path_rate;
    pubTimeReferenceInterval = (pub_time_reference_rate == 0)
                                   ? 0
                                   : ncom_rate / pub_time_reference_rate;
    pubEcefPosInterval =
        (pub_ecef_pos_rate == 0) ? 0 : ncom_rate / pub_ecef_pos_rate;
    pubNavSatRefInterval =
        (pub_nav_sat_ref_rate == 0) ? 0 : ncom_rate / pub_nav_sat_ref_rate;
    pubLeverArmInterval =
        (pub_lever_arm_rate == 0) ? 0 : ncom_rate / pub_lever_arm_rate;
    pubIMUBiasInterval =
        (pub_imu_bias_rate == 0) ? 0 : ncom_rate / pub_imu_bias_rate;

    // Initilize tf broadcaster if configured to broadcast
    if (pub_tf_flag) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    // Initialise publishers for each message if configured to publish
    if (pub_imu_flag)
      pubImu_ = this->create_publisher<sensor_msgs::msg::Imu>(
          topic_prefix + "/" + imu_topic, 10);

    if (pubStringInterval) {
      // Throw an error if ncom_rate / String_rate is not an integer
      if (ncom_rate % pubStringInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "String" + notFactorError,
                     pub_string_rate);
        return;
      }
      // Create publisher
      pubString_ = this->create_publisher<std_msgs::msg::String>(
          topic_prefix + "/" + string_topic, 10);
    }
    if (pubNavSatFixInterval) {
      // Throw an error if ncom_rate / NavSatFix_rate is not an integer
      if (ncom_rate % pubNavSatFixInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "NavSatFix" + notFactorError,
                     pub_nav_sat_fix_rate);
        return;
      }
      // Create publisher
      pubNavSatFix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
          topic_prefix + "/" + nav_sat_fix_topic, 10);
    }
    if (pubVelocityInterval) {
      // Throw an error if ncom_rate / Velocity_rate is not an integer
      if (ncom_rate % pubVelocityInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "Velocity" + notFactorError,
                     pub_velocity_rate);
        return;
      }
      // Create publisher
      pubVelocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          topic_prefix + "/" + velocity_topic, 10);
    }
    if (pubOdometryInterval) {
      // Throw an error if ncom_rate / Odometry_rate is not an integer
      if (ncom_rate % pubOdometryInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "Odometry" + notFactorError,
                     pub_odometry_rate);
        return;
      }
      // Create publisher
      pubOdometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
          topic_prefix + "/" + odometry_topic, 10);
    }
    if (pubPathInterval) {
      // Throw an error if ncom_rate / Path_rate is not an integer
      if (ncom_rate % pubPathInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "Path" + notFactorError,
                     pub_path_rate);
        return;
      }
      // Create publisher
      pubPath_ = this->create_publisher<nav_msgs::msg::Path>(
          topic_prefix + "/" + path_topic, 10);
    }
    if (pubTimeReferenceInterval) {
      // Throw an error if ncom_rate / TimeReference_rate is not an integer
      if (ncom_rate % pubTimeReferenceInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "TimeReference" + notFactorError,
                     pub_time_reference_rate);
        return;
      }
      // Create publisher
      pubTimeReference_ =
          this->create_publisher<sensor_msgs::msg::TimeReference>(
              topic_prefix + "/" + time_reference_topic, 10);
    }
    if (pubEcefPosInterval) {
      // Throw an error if ncom_rate / EcefPos_rate is not an integer
      if (ncom_rate % pubEcefPosInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "EcefPos" + notFactorError,
                     pub_ecef_pos_rate);
        return;
      }
      // Create publisher
      pubEcefPos_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
          topic_prefix + "/" + ecef_pos_topic, 10);
    }
    if (pubNavSatRefInterval) {
      // Throw an error if ncom_rate / NavSatRef_rate is not an integer
      if (ncom_rate % pubNavSatRefInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "NavSatRef" + notFactorError,
                     pub_nav_sat_ref_rate);
        return;
      }
      // Create publisher
      pubNavSatRef_ = this->create_publisher<oxts_msgs::msg::NavSatRef>(
          topic_prefix + "/" + nav_sat_ref_topic, 10);
    }
    if (pubLeverArmInterval) {
      // Throw an error if ncom_rate / LeverArm_rate is not an integer
      if (ncom_rate % pubLeverArmInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "LeverArm" + notFactorError,
                     pub_lever_arm_rate);
        return;
      }
      // Create publisher
      pubLeverArm_ = this->create_publisher<oxts_msgs::msg::LeverArm>(
          topic_prefix + "/" + lever_arm_topic, 10);
    }
    if (pubIMUBiasInterval) {
      // Throw an error if ncom_rate / LeverArm_rate is not an integer
      if (ncom_rate % pubIMUBiasInterval != 0) {
        RCLCPP_ERROR(this->get_logger(), "IMUBias" + notFactorError,
                     pub_imu_bias_rate);
        return;
      }
      // Create publisher
      pubIMUBias_ = this->create_publisher<oxts_msgs::msg::ImuBias>(
          topic_prefix + "/" + imu_bias_topic, 10);
    }

    // Initialise subscriber for regular ncom packet message
    subNCom_ = this->create_subscription<oxts_msgs::msg::Ncom>(
        topic_prefix + "/" + ncom_topic, 10,
        std::bind(&OxtsIns::ncomCallbackRegular, this, _1));

    nrx = NComCreateNComRxC();

    lrf_valid = false;
  }

  /** NCom decoder instance */
  NComRxC *nrx;
  /** Local reference frame validity flag */
  bool lrf_valid;
  /** Local reference frame */
  Lrf lrf;
};

} // namespace oxts_ins

#endif // OXTS_INS__INS_HPP_