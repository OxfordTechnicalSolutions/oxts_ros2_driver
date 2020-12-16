#ifndef NCOM_PUBLISHER_NODE_HPP
#define NCOM_PUBLISHER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "nav/NComRxC.h"


/* 
 * This class creates a subclass of Node designed to take NCom data from the 
 * NCom decoder and publish it to pre-configured ROS topics.
 */

class NComPublisherNode : public rclcpp::Node
{
public:
  NComPublisherNode()
  : Node("ncom_publisher"), count_(0)
  {
    // Initialise the publisher with the string message type, topic name 
    // "topic", and queue size limit
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); 
    //publisher2_ = this->create_publisher<nav_msgs::msg::Odometry>("topic2", 10); 

  }
  int ncom_callback(NComRxC* nrx);

private:
  nav_msgs::msg::Odometry odo;
  sensor_msgs::msg::NavSatFix nav;
  // TODO: Config
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher2_;
  // TODO: Restructure with different publishers for different messages

  size_t count_;
};






#endif //NCOM_PUBLISHER_NODE_HPP