#include "oxts_ins/convert.hpp"


namespace oxts_ins
{

void OxtsIns::NCom_callback(const oxts_msgs::msg::Ncom::SharedPtr msg)
{
  // Add data to decoder
  if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) == COM_NEW_UPDATE)
  {
    double current_time = this->get_timestamp().seconds();
    int sec_idx = round((current_time - floor(current_time)) * this->ncom_rate);
    
    switch (nrx->mOutputPacketType)
    {
      case OUTPUT_PACKET_REGULAR:
      {
        // Publish IMU message if being subscribed to and enabled in config
        if (this->pub_imu_flag) 
          this->imu();
        if (this->pub_tf_flag)
          this->tf();
        if (this->pubStringInterval && (sec_idx % this->pubStringInterval == 0))
          this->string();
        if (this->pubNavSatRefInterval && (sec_idx % this->pubNavSatRefInterval == 0))
          this->nav_sat_ref();
        if (this->pubEcefPosInterval && (sec_idx % this->pubEcefPosInterval == 0))
          this->ecef_pos();
        if (this->pubNavSatFixInterval && (sec_idx % this->pubNavSatFixInterval == 0))
          this->nav_sat_fix();
        if (this->pubVelocityInterval && (sec_idx % this->pubVelocityInterval == 0))
          this->velocity();
        if (this->pubTimeReferenceInterval && (sec_idx % this->pubTimeReferenceInterval == 0))
          this->time_reference();
        break;
      }
      case OUTPUT_PACKET_STATUS:
      {
        break;
      }
      default : break;

    };
  }
}

void OxtsIns::string()
{
  auto msgString = RosNComWrapper::string(this->nrx);
  pubString_->publish(msgString);

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", 
                                  msgString.data.c_str());
}

void OxtsIns::nav_sat_fix()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "navsat_link");
  auto msg    = RosNComWrapper::nav_sat_fix(this->nrx, header);
  pubNavSatFix_->publish(msg);
}

void OxtsIns::nav_sat_ref()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "navsat_link");
  auto msg    = RosNComWrapper::nav_sat_ref(this->nrx, header);
  pubNavSatRef_->publish(msg);
}
void OxtsIns::ecef_pos()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
  auto msg    = RosNComWrapper::ecef_pos(this->nrx, header);
  pubEcefPos_->publish(msg);
}

void OxtsIns::imu()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "imu_link");
  auto msg    = RosNComWrapper::imu(this->nrx, header);
  pubImu_->publish(msg);
}

void OxtsIns::tf()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "imu_link");

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

void OxtsIns::velocity()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
  auto msg    = RosNComWrapper::velocity(this->nrx, header);
  pubVelocity_->publish(msg);
}

void OxtsIns::time_reference()
{
  std_msgs::msg::Header header;
  header = RosNComWrapper::header(this->get_timestamp(), "oxts_link");
  auto msg    = RosNComWrapper::time_reference(this->nrx, header);
  pubTimeReference_->publish(msg);
}

rclcpp::Time OxtsIns::get_timestamp()
{
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return this->get_clock()->now();
  else
    return RosNComWrapper::ncomTime(nrx);
}

} // namespace oxts_ins