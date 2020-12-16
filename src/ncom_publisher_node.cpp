#include "ncom_publisher_node.hpp"


int NComPublisherNode::ncom_callback(NComRxC* nrx)
{
    // Code to publish string - reuse to output some ncom data in string
    auto message = std_msgs::msg::String();
    message.data = "Lat, Long, Alt : " + std::to_string(nrx->mLat) + ", "
                                + std::to_string(nrx->mLon) + ", "
                                + std::to_string(nrx->mAlt);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // TODO: Add switch statement on different messages to be output

    return 0;
}