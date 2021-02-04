#include "oxts_driver/driver.hpp"

namespace oxts_driver
{

void OxtsDriver::timer_ncom_callback()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpointNCom);
  // Add data to decoder
  NComNewChars(this->nrx, this->buff, size); 
}

void OxtsDriver::timer_ncom_file_callback()
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

void OxtsDriver::timer_string_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    auto msgString = RosNComWrapper::wrap_string(this->nrx);
    pubString_->publish(msgString);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", 
                                    msgString.data.c_str());
  }
}

void OxtsDriver::timer_nav_sat_fix_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "navsat_link");
    auto msg    = RosNComWrapper::wrap_nav_sat_fix(this->nrx, header);
    pubNavSatFix_->publish(msg);
  }
}

void OxtsDriver::timer_ecef_pos_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::wrap_ecef_pos(this->nrx, header);
    pubEcefPos_->publish(msg);
  }
}

void OxtsDriver::timer_imu_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME
     && this->nrx->mIsImu2VehHeadingValid)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "imu_link");
    auto msg    = RosNComWrapper::wrap_imu(this->nrx, header);
    pubImu_->publish(msg);
 
    geometry_msgs::msg::TransformStamped tf_oxts;
    tf_oxts.header = header;
    tf_oxts.header.frame_id = "base_link";
    tf_oxts.child_frame_id = "oxts_link";
    tf_oxts.transform.rotation.x = msg.orientation.x;
    tf_oxts.transform.rotation.y = msg.orientation.y;
    tf_oxts.transform.rotation.z = msg.orientation.z;
    tf_oxts.transform.rotation.w = msg.orientation.w;
    tf_broadcaster_->sendTransform(tf_oxts);
  }
}

void OxtsDriver::timer_velocity_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME
     && this->nrx->mIsImu2VehHeadingValid)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::wrap_velocity(this->nrx, header);
    pubVelocity_->publish(msg);
  }
}

void OxtsDriver::timer_time_reference_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::wrap_header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::wrap_time_reference(this->nrx, header);
    pubTimeReference_->publish(msg);
  }
}

rclcpp::Time OxtsDriver::get_timestamp()
{
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return clock_.now();
  else
    return RosNComWrapper::ncom_time_to_time(nrx);
}

std::string OxtsDriver::get_unit_ip()
{
  return this->unit_ip;
}

short       OxtsDriver::get_unit_port()
{
  return this->unit_port;
}

} // namespace oxts_driver