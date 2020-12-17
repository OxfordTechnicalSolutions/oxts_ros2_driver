#include "ncom_publisher_node.hpp"


#define SECS2NANOSECS 1e9

int NComPublisherNode::ncom_callback(NComRxC* nrx)
{
  //////////////////////////////////////////////////////////////////////////////
  // Construct std_msgs/msg/String
  //////////////////////////////////////////////////////////////////////////////
  auto msgString = std_msgs::msg::String();
  msgString.data = "Lat, Long, Alt : " + std::to_string(nrx->mLat) + ", "
                                       + std::to_string(nrx->mLon) + ", "
                                       + std::to_string(nrx->mAlt);
  pubString_->publish(msgString);      
  if ((this->count_ % 100) == 0)
    RCLCPP_INFO(this->get_logger(), "'%d' Publishing: '%s'", this->count_, msgString.data.c_str());
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/NavSatFix
  //////////////////////////////////////////////////////////////////////////////
  auto msgNavSatFix = sensor_msgs::msg::NavSatFix();
  msgNavSatFix.header.stamp.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msgNavSatFix.header.stamp.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))*SECS2NANOSECS);
  msgNavSatFix.header.frame_id = "WGS84"; // @TODO Change this

  msgNavSatFix.status.status = nrx->mGpsPosMode; // @TODO Change to ROS code
  msgNavSatFix.status.service = 8; // @TODO: 1 GPS, 2 GLO, 4 Bei, 8 GAL

  msgNavSatFix.latitude  = nrx->mLat;
  msgNavSatFix.longitude = nrx->mLon;
  msgNavSatFix.altitude  = nrx->mAlt;

  // @TODO: This accuracy is not actually a covariance. Also should be in ENU
  msgNavSatFix.position_covariance[0] = nrx->mNorthAcc;
  msgNavSatFix.position_covariance[4] = nrx->mEastAcc;
  msgNavSatFix.position_covariance[8] = nrx->mAltAcc;

  msgNavSatFix.position_covariance_type = 2; // @TODO Change to ROS code
  
  pubNavSatFix_->publish(msgNavSatFix);
  
  /*
   * @TODO: Add switch statement on different messages to be output
   */    

  this->count_++;  
  return 0;
}