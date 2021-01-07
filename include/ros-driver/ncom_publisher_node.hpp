/**
 * \file ncom_publisher_node.hpp
 * Defines node to take NCom data and publish it in ROS messages.
 */

#ifndef NCOM_PUBLISHER_NODE_HPP
#define NCOM_PUBLISHER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "nav/NComRxC.h"
#include "ros-driver/nav_const.hpp"
#include "ros-driver/ros_ncom_wrapper.hpp"


/**
 * This class creates a subclass of Node designed to take NCom data from the 
 * NCom decoder and publish it to pre-configured ROS topics.
 * 
 * @todo Add config struct to hold data which will hold config parsed from the 
 *       .yaml file.
 * @todo Change topic names to a standard form /imu/.. 
 */
class NComPublisherNode : public rclcpp::Node
{
private:

  rclcpp::Parameter param_ncom_rate;
  rclcpp::Parameter param_unit_ip;
  rclcpp::Parameter param_unit_port;
  rclcpp::Parameter param_timestamp_mode;
  rclcpp::Parameter param_frame_id;
  rclcpp::Parameter param_pub_string_rate;
  rclcpp::Parameter param_pub_odometry_rate;
  rclcpp::Parameter param_pub_nav_sat_fix_rate;
  rclcpp::Parameter param_pub_imu_rate;
  rclcpp::Parameter param_pub_velocity_rate;
  rclcpp::Parameter param_pub_tf2_rate;

  int ncomRate;
  std::string unitIp;
  short unitPort;
  short timestampMode;
  std::string frameId;
  int pubStringRate;
  int pubOdometryRate;
  int pubNavSatFixRate;
  int pubImuRate;
  int pubVelocityRate;
  int pubTf2Rate;
  // ...



  /**
   * Publisher for std_msgs/msg/string. Only used for debugging, currently 
   * outputs lat, long, alt in string form.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       pubString_;
  /**
   * Publisher for /sensor_msgs/msg/Odometry
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     pubOdometry_;
  /**
   * Publisher for /sensor_msgs/msg/NavSatFix
   */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNavSatFix_;
  /**
   * Publisher for /sensor_msgs/msg/Imu
   */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       pubImu_;
  /**
   * Publisher for /sensor_msgs/msg/TwistStamped
   */
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  pubVelocity_;
  /**
   * Publisher for /geometry_msgs/msg/TransformStamped
   */
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr  pubTf2_;

public:
  NComPublisherNode() : Node("ncom_publisher"), count_(0)
  {
    // Initialise configurable parameters (all params should have defaults)
    this->declare_parameter("ncom_rate", 100);
    this->declare_parameter("unit_ip", "0.0.0.0");
    this->declare_parameter("unit_port", 3000);
    this->declare_parameter("timestamp_mode", 1);
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("pub_string_rate", 1);
    this->declare_parameter("pub_odometry_rate", 1);
    this->declare_parameter("pub_nav_sat_fix_rate", 1);
    this->declare_parameter("pub_imu_rate", 1);
    this->declare_parameter("pub_velocity_rate", 1);
    this->declare_parameter("pub_tf2_rate", 1);

    // Get parameters (from config, command line, or from default)
    param_ncom_rate                 = this->get_parameter("ncom_rate");
    param_unit_ip                   = this->get_parameter("unit_ip");
    param_unit_port                 = this->get_parameter("unit_port");
    param_timestamp_mode            = this->get_parameter("timestamp_mode");
    param_frame_id                  = this->get_parameter("frame_id");
    param_pub_string_rate           = this->get_parameter("pub_string_rate");
    param_pub_odometry_rate         = this->get_parameter("pub_odometry_rate");
    param_pub_nav_sat_fix_rate      = this->get_parameter("pub_nav_sat_fix_rate");
    param_pub_imu_rate              = this->get_parameter("pub_imu_rate");
    param_pub_velocity_rate         = this->get_parameter("pub_velocity_rate");
    param_pub_tf2_rate              = this->get_parameter("pub_tf2_rate");
    // Convert parameters to useful variable types
    ncomRate              = param_ncom_rate.as_int();
    unitIp                = param_unit_ip.as_string();
    unitPort              = param_unit_port.as_int();
    timestampMode         = param_timestamp_mode.as_int();
    frameId               = param_frame_id.as_string();
    pubStringRate         = param_pub_string_rate.as_int();
    pubOdometryRate       = param_pub_odometry_rate.as_int();
    pubNavSatFixRate      = param_pub_nav_sat_fix_rate.as_int();
    pubImuRate            = param_pub_imu_rate.as_int();
    pubVelocityRate       = param_pub_velocity_rate.as_int();
    pubTf2Rate            = param_pub_tf2_rate.as_int();

    // Derive number of NCom packets received per message published by each 
    // publisher from the NCom and publisher rates. 0 => message configured off
    ncomPerStringPublished    = pubStringRate    ? (ncomRate / pubStringRate   ) : 0;
    ncomPerOdometryPublished  = pubOdometryRate  ? (ncomRate / pubOdometryRate ) : 0;
    ncomPerNavSatFixPublished = pubNavSatFixRate ? (ncomRate / pubNavSatFixRate) : 0;
    ncomPerImuPublished       = pubImuRate       ? (ncomRate / pubImuRate      ) : 0;
    ncomPerVelocityPublished  = pubVelocityRate  ? (ncomRate / pubVelocityRate ) : 0;
    ncomPerTf2Published       = pubTf2Rate       ? (ncomRate / pubTf2Rate      ) : 0;

    // Initialise publishers for each message - all are initialised, even if not
    // configured
    pubString_    = this->create_publisher<std_msgs::msg::String>               ("ins/debug_string_pos", 10); 
    pubOdometry_  = this->create_publisher<nav_msgs::msg::Odometry>             ("ins/odom",             10); 
    pubNavSatFix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>         ("ins/nav_sat_fix",      10); 
    pubImu_       = this->create_publisher<sensor_msgs::msg::Imu>               ("imu/imu_data",         10); 
    pubVelocity_  = this->create_publisher<geometry_msgs::msg::TwistStamped>    ("ins/velocity",         10); 
    // pubTimeReference_
    pubTf2_       = this->create_publisher<geometry_msgs::msg::TransformStamped>("ins/tf2",              10); 

    // Initialise uptime to 0
    upTime = 0;
  }

  /**
   * Count of callbacks run by the node (will correspond to number of ncom 
   * packets received.)
   */
  int count_;
  /**
   * Time that the node has been active (s).
   * 
   * This is derived from number of NCom packets received so not suitable for 
   * critical applications. More useful as an indicator.
   */
  unsigned long int upTime;
  /**
   * Expected number of NCom packets received for every one String message to be 
   * published. This is derived from the ncomRate and pubStringRate
   */
  int ncomPerStringPublished;
  /**
   * Expected number of NCom packets received for every one Odometry message to be 
   * published. This is derived from the ncomRate and pubOdometryRate
   */
  int ncomPerOdometryPublished;
  /**
   * Expected number of NCom packets received for every one NavSatFix message to be 
   * published. This is derived from the ncomRate and pubNavSatFixRate
   */
  int ncomPerNavSatFixPublished;
  /**
   * Expected number of NCom packets received for every one Imu message to be 
   * published. This is derived from the ncomRate and pubImuRate
   */
  int ncomPerImuPublished;
  /**
   * Expected number of NCom packets received for every one Velocity message to be 
   * published. This is derived from the ncomRate and pubVelocityRate
   */
  int ncomPerVelocityPublished;
  /**
   * Expected number of NCom packets received for every one tf2 message to be 
   * published. This is derived from the ncomRate and pubTf2Rate
   */
  int ncomPerTf2Published;

  /**
   * Constructs and publishes all configured ROS topics using data from NCom
   * decoder.
   */
  int ncom_callback(const NComRxC* nrx);

  /**
   * Get the IP address of the OxTS unit, as set in the .yaml params file
   * 
   * @returns IP address as a string
   */
  std::string get_unit_ip();
  /**
   * Get the endpoint port of the OxTS unit, as set in the .yaml params file
   * 
   * @returns Port as a short
   */
  short get_unit_port();

};






#endif //NCOM_PUBLISHER_NODE_HPP