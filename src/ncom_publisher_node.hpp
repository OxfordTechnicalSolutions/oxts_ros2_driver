#ifndef NCOM_PUBLISHER_NODE_HPP
#define NCOM_PUBLISHER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "nav/NComRxC.h"


using namespace std::chrono_literals;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class NComPublisherNode : public rclcpp::Node
{
public:
  NComPublisherNode()
  : Node("ncom_publisher"), count_(0)
  {
    // Initialise the publisher with the string message type, topic name 
    // "topic", and queue size limit
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); 
    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic2", 10); 

  }
  int ncom_callback(NComRxC* nrx);

private:

  // TODO: Config
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  // TODO: Restructure with different publishers for different messages

  size_t count_;
};






#endif //NCOM_PUBLISHER_NODE_HPP