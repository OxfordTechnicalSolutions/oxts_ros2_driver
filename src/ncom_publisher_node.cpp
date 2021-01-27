#include "ros-driver/ncom_publisher_node.hpp"


void NComPublisherNode::timer_ncom_callback()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpointNCom);
  // Add data to decoder
  NComNewChars(this->nrx, this->buff, size); 
}

void NComPublisherNode::timer_ncom_file_callback()
{
  char c;

  while(this->inFileNCom.get(c))
  {
    // Decode the data
    if(NComNewChar(this->nrx, (unsigned char) c) == COM_NEW_UPDATE)
    {
      break;
    }
  }

}

void NComPublisherNode::timer_string_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    auto msgString = RosNComWrapper::wrap_string(this->nrx);
    pubString_->publish(msgString);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", 
                                    msgString.data.c_str());
  }
}


void NComPublisherNode::timer_nav_sat_fix_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "ins");
    auto msg    = RosNComWrapper::wrap_nav_sat_fix(this->nrx, header);
    pubNavSatFix_->publish(msg);
  }
}

void NComPublisherNode::timer_imu_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "imu");
    auto msg    = RosNComWrapper::wrap_imu(this->nrx, header);
    pubImu_->publish(msg);
  }
}

void NComPublisherNode::timer_velocity_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "ins");
    auto msg    = RosNComWrapper::wrap_velocity(this->nrx, header);
    pubVelocity_->publish(msg);
  }
}

void NComPublisherNode::timer_time_reference_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "ins");
    auto msg    = RosNComWrapper::wrap_time_reference(this->nrx, header);
    pubTimeReference_->publish(msg);
  }
}

void NComPublisherNode::timer_tf2_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "earth");
    auto msg    = RosNComWrapper::wrap_tf2(this->nrx, header);
    pubTf2_->publish(msg);
  }
}


rclcpp::Time NComPublisherNode::get_timestamp()
{
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return clock_.now();
  else
    return RosNComWrapper::ncom_time_to_time(nrx);
}

std::string NComPublisherNode::get_unit_ip()
{
  return this->unit_ip;
}

short       NComPublisherNode::get_unit_port()
{
  return this->unit_port;
}