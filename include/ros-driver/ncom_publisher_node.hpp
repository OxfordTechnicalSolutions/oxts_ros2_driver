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
 * @todo Add config struct to hold data which will eventually be parsed from a 
 *       .yaml file.
 * @todo Change topic names to a standard form /imu/.. 
 */
class NComPublisherNode : public rclcpp::Node
{
public:
  rclcpp::Parameter param_imu_rate;

  int imu_rate;


  NComPublisherNode()
  : Node("ncom_publisher"), count_(0)
  {
    // Initialise parameters
    this->declare_parameter("imu_rate");//, 100);

    param_imu_rate = this->get_parameter("imu_rate");

    imu_rate = param_imu_rate.as_int();

    // Initialise publishers for each message - all are initialised, even if not
    // configured
    pubString_    = this->create_publisher<std_msgs::msg::String>      ("imu/std_msgs/msg/string", 10); 
    pubOdometry_  = this->create_publisher<nav_msgs::msg::Odometry>    ("imu/nav_msgs/msg/odometry", 10); 
    pubNavSatFix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("imu/sensor_msgs/msg/nav_sat_fix", 10); 
    pubImu_       = this->create_publisher<sensor_msgs::msg::Imu>      ("imu/sensor_msgs/msg/imu", 10); 

  }

  /**
   * Constructs and publishes all configured ROS topics using data from NCom
   * decoder.
   */
  int ncom_callback(const NComRxC* nrx);

private:

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
   * Count of messages sent by the node
   */
  int count_;
};






#endif //NCOM_PUBLISHER_NODE_HPP