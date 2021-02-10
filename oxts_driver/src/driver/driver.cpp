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
    auto msgString = RosNComWrapper::string(this->nrx);
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
    header = RosNComWrapper::header(this->get_timestamp(), "navsat_link");
    auto msg    = RosNComWrapper::nav_sat_fix(this->nrx, header);
    pubNavSatFix_->publish(msg);
  }
}

void OxtsDriver::timer_nav_sat_ref_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::nav_sat_ref(this->nrx, header);
    pubNavSatRef_->publish(msg);
  }
}
void OxtsDriver::timer_ecef_pos_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::ecef_pos(this->nrx, header);
    pubEcefPos_->publish(msg);
  }
}

void OxtsDriver::timer_imu_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME
     && this->nrx->mIsImu2VehHeadingValid)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::header(this->get_timestamp(), "imu_link");
    auto msg    = RosNComWrapper::imu(this->nrx, header);
    pubImu_->publish(msg);
 
    auto rpyENU    = RosNComWrapper::getRPY(this->nrx);
    geometry_msgs::msg::TransformStamped tf_oxts;
    tf_oxts.header = header;
    tf_oxts.header.frame_id = "map";
    tf_oxts.child_frame_id = "vehicle_link";
    tf_oxts.transform.rotation.x = rpyENU.x();
    tf_oxts.transform.rotation.y = rpyENU.y();
    tf_oxts.transform.rotation.z = rpyENU.z();
    tf_oxts.transform.rotation.w = rpyENU.w();
    tf_broadcaster_->sendTransform(tf_oxts);

    auto vat    = RosNComWrapper::getVat(this->nrx);
    auto nsp    = RosNComWrapper::getNsp(this->nrx);
    // convert nsp from imu->axle to axle->imu
    nsp = tf2::quatRotate(vat.inverse(), -nsp);
    geometry_msgs::msg::TransformStamped tf_vat;
    tf_vat.header = header;
    tf_vat.header.frame_id = "vehicle_link";
    tf_vat.child_frame_id = "oxts_link";
    tf_vat.transform.translation.x = nsp.x();
    tf_vat.transform.translation.y = nsp.y();
    tf_vat.transform.translation.z = nsp.z();
    tf_vat.transform.rotation.x = vat.inverse().x();
    tf_vat.transform.rotation.y = vat.inverse().y();
    tf_vat.transform.rotation.z = vat.inverse().z();
    tf_vat.transform.rotation.w = vat.inverse().w();
    tf_broadcaster_->sendTransform(tf_vat);
  }
}

void OxtsDriver::timer_velocity_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME
     && this->nrx->mIsImu2VehHeadingValid)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::velocity(this->nrx, header);
    pubVelocity_->publish(msg);
  }
}

void OxtsDriver::timer_time_reference_callback()
{
  if(this->nrx->mInsNavMode == NAV_CONST::NAV_MODE::REAL_TIME)
  {
    std_msgs::msg::Header header;
    header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
    auto msg    = RosNComWrapper::time_reference(this->nrx, header);
    pubTimeReference_->publish(msg);
  }
}

rclcpp::Time OxtsDriver::get_timestamp()
{
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return clock_.now();
  else
    return RosNComWrapper::ncomTime(nrx);
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