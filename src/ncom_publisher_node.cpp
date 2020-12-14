#include "ncom_publisher_node.hpp"


int NComPublisherNode::ncom_callback(NComRxC* nrx)
{


    // Code to publish string - reuse to output some ncom data in string
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    return 0;
}