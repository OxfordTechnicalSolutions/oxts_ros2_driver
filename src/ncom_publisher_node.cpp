#include "ros-driver/ncom_publisher_node.hpp"


void NComPublisherNode::timer_ncom_callback()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpointNCom);
  // Add data to decoder
  NComNewChars(nrx, buff, size); 
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
  auto msgString = RosNComWrapper::wrap_string (nrx);
  pubString_->publish(msgString);

  RCLCPP_INFO(this->get_logger(), "'%d' Publishing: '%s'", 
                                  this->count_, msgString.data.c_str());
}

void NComPublisherNode::timer_odometry_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "ins");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "ins");

  auto msg    = RosNComWrapper::wrap_odometry (this->nrx, header);
  pubOdometry_->publish(msg);
}

void NComPublisherNode::timer_nav_sat_fix_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "ins");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "ins");

  auto msg    = RosNComWrapper::wrap_nav_sat_fix(this->nrx, header);
  pubNavSatFix_->publish(msg);
}

void NComPublisherNode::timer_imu_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "ins");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "ins");

  auto msg    = RosNComWrapper::wrap_imu(this->nrx, header);
  pubImu_->publish(msg);
}

void NComPublisherNode::timer_velocity_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "ins");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "ins");

  auto msg    = RosNComWrapper::wrap_velocity(this->nrx, header);
  pubVelocity_->publish(msg);
}

void NComPublisherNode::timer_time_reference_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "ins");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "ins");

  auto msg    = RosNComWrapper::wrap_time_reference(this->nrx, header);
  pubTimeReference_->publish(msg);
}

void NComPublisherNode::timer_tf2_callback()
{
  std_msgs::msg::Header header;
  if (this->timestampMode == PUB_TIMESTAMP_MODE::DRIVER)
    header = RosNComWrapper::wrap_header(clock_.now(), "earth");
  else
    header = RosNComWrapper::wrap_header(RosNComWrapper::ncom_time_to_time(nrx), "earth");

  auto msg    = RosNComWrapper::wrap_tf2(this->nrx, header);
  pubTf2_->publish(msg);
}


std::string NComPublisherNode::get_unit_ip()
{
  return this->unitIp;
}

short       NComPublisherNode::get_unit_port()
{
  return this->unitPort;
}