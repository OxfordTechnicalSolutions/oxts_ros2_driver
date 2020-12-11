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
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // Initialise the publisher with the string message type, topic name "topic", and queue size limit
    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic2", 10); //
    timer_ = this->create_wall_timer(
      500ms, std::bind(&NComPublisherNode::timer_callback, this)); // Initialise timer, which causes timer_callback to be called twice a second
  }
  int ncom_callback(NComRxC* nrx);

private:
  void timer_callback() // Sets data in message and does the publishing
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    publisher2_->publish(message);
  }
  

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  size_t count_;
};






#endif //NCOM_PUBLISHER_NODE_HPP