#include "oxts_driver/ros_ncom_wrapper.hpp"


tf2::Quaternion RosNComWrapper::getVat(const NComRxC *nrx)
{
  tf2::Quaternion vat; 
  vat.setRPY(
    NAV_CONST::DEG2RADS * nrx->mImu2VehRoll,
    NAV_CONST::DEG2RADS * nrx->mImu2VehPitch,
    NAV_CONST::DEG2RADS * nrx->mImu2VehHeading
  );
  return vat;
}

tf2::Vector3 RosNComWrapper::getVaa(const NComRxC *nrx)
{
  tf2::Vector3 vaa; // Translation of rear axle in imu frame
  vaa.setX(nrx->mNoSlipLeverArmX);
  vaa.setY(nrx->mNoSlipLeverArmY);
  vaa.setZ(nrx->mNoSlipLeverArmZ);
  return vaa;
}

tf2::Quaternion RosNComWrapper::getRPY(const NComRxC *nrx)
{
  auto rpyNED = tf2::Quaternion(); // Orientation of the vehicle (NED frame)
  auto rpyENU = tf2::Quaternion(); // Orientation of the vehicle (ENU frame)
  auto ned2enu = tf2::Quaternion(); // NED to ENU rotation

  rpyNED.setRPY(
    NAV_CONST::DEG2RADS * nrx->mRoll,
    NAV_CONST::DEG2RADS * nrx->mPitch,
    NAV_CONST::DEG2RADS * nrx->mHeading
  );
  ned2enu.setRPY(180.0*NAV_CONST::DEG2RADS,0,90.0*NAV_CONST::DEG2RADS);
  rpyENU = ned2enu * rpyNED;
  return rpyENU;
}


rclcpp::Time      RosNComWrapper::ncomTime(const NComRxC *nrx)
{
  auto time = rclcpp::Time(static_cast<int32_t>(nrx->mTimeWeekSecond) + 
                           (nrx->mTimeWeekCount * NAV_CONST::WEEK_SECS) + 
                           nrx->mTimeUtcOffset + NAV_CONST::GPS2UNIX_EPOCH,
  static_cast<uint32_t>((nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS ));

  return time;
}

std_msgs::msg::Header       RosNComWrapper::header(rclcpp::Time time,
                                                        std::string frame)
{
  auto header = std_msgs::msg::Header();

  header.stamp = time;
  header.frame_id = frame; 

  return header;
}

sensor_msgs::msg::NavSatStatus RosNComWrapper::nav_sat_status(
                                                    const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::NavSatStatus();

  switch(nrx->mGpsPosMode)
  {
    // No Fix
    case NAV_CONST::GNSS_POS_MODE::NONE          :
    case NAV_CONST::GNSS_POS_MODE::SEARCH        :
    case NAV_CONST::GNSS_POS_MODE::DOPPLER       :
    case NAV_CONST::GNSS_POS_MODE::NODATA        :
    case NAV_CONST::GNSS_POS_MODE::BLANKED       :
    case NAV_CONST::GNSS_POS_MODE::PP_DOPPLER    :
    case NAV_CONST::GNSS_POS_MODE::NOT_KNOWN     :
    case NAV_CONST::GNSS_POS_MODE::GX_DOPPLER    :
    case NAV_CONST::GNSS_POS_MODE::IX_DOPPLER    :
    case NAV_CONST::GNSS_POS_MODE::UNKNOWN       :
      msg.status = msg.STATUS_NO_FIX;
      break;
    // Fix
    case NAV_CONST::GNSS_POS_MODE::SPS           :
    case NAV_CONST::GNSS_POS_MODE::PP_SPS        :
    case NAV_CONST::GNSS_POS_MODE::GX_SPS        :
    case NAV_CONST::GNSS_POS_MODE::IX_SPS        :
    case NAV_CONST::GNSS_POS_MODE::GENAID        :
    case NAV_CONST::GNSS_POS_MODE::SEGMENT       :
      msg.status = msg.STATUS_FIX;
      break;
    // Ground-based augmentation
    case NAV_CONST::GNSS_POS_MODE::DIFF          :
    case NAV_CONST::GNSS_POS_MODE::FLOAT         :
    case NAV_CONST::GNSS_POS_MODE::INTEGER       :
    case NAV_CONST::GNSS_POS_MODE::PP_DIFF       :
    case NAV_CONST::GNSS_POS_MODE::PP_FLOAT      :
    case NAV_CONST::GNSS_POS_MODE::PP_INTEGER    :
    case NAV_CONST::GNSS_POS_MODE::GX_DIFF       :
    case NAV_CONST::GNSS_POS_MODE::GX_FLOAT      :
    case NAV_CONST::GNSS_POS_MODE::GX_INTEGER    :
    case NAV_CONST::GNSS_POS_MODE::IX_DIFF       :
    case NAV_CONST::GNSS_POS_MODE::IX_FLOAT      :
    case NAV_CONST::GNSS_POS_MODE::IX_INTEGER    :
      msg.status = msg.STATUS_GBAS_FIX;
      break;
    // Satellite-based augmentation
    case NAV_CONST::GNSS_POS_MODE::WAAS          :
    case NAV_CONST::GNSS_POS_MODE::OMNISTAR      :
    case NAV_CONST::GNSS_POS_MODE::OMNISTARHP    :
    case NAV_CONST::GNSS_POS_MODE::OMNISTARXP    :
    case NAV_CONST::GNSS_POS_MODE::CDGPS         :
    case NAV_CONST::GNSS_POS_MODE::PPP_CONVERGING:
    case NAV_CONST::GNSS_POS_MODE::PPP           :
    case NAV_CONST::GNSS_POS_MODE::GX_SBAS       :
    case NAV_CONST::GNSS_POS_MODE::IX_SBAS       :
      msg.status = msg.STATUS_SBAS_FIX;
      break;
  }

  msg.service = 0; /*! @todo Output value based on use of constellations */

  return msg;
}

sensor_msgs::msg::NavSatFix RosNComWrapper::nav_sat_fix(
                            const NComRxC *nrx,
                            std_msgs::msg::Header head)
{
  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header = head;

  msg.status = RosNComWrapper::nav_sat_status(nrx);

  msg.latitude  = nrx->mLat;
  msg.longitude = nrx->mLon;
  msg.altitude  = nrx->mAlt;

  //!< @todo This accuracy is not actually a covariance. Also should be in ENU
  msg.position_covariance[0] = nrx->mEastAcc;
  msg.position_covariance[4] = nrx->mNorthAcc;
  msg.position_covariance[8] = nrx->mAltAcc;

  msg.position_covariance_type = 2; /*! @todo Change to ROS code */
  
  return msg;
}


geometry_msgs::msg::PointStamped RosNComWrapper::ecef_pos
                                              (
                                              const NComRxC *nrx,
                                              std_msgs::msg::Header head
                                              )
{
  auto msg = geometry_msgs::msg::PointStamped();
  msg.header = head;

  std::vector<double> ecef = Convert::lla_to_ecef(nrx->mLat, nrx->mLon, nrx->mAlt);
  msg.point.x    = ecef[0]; 
  msg.point.y    = ecef[1];
  msg.point.z    = ecef[2];

  return msg;
}

std_msgs::msg::String RosNComWrapper::string (const NComRxC *nrx)
{
  auto msg = std_msgs::msg::String();
  msg.data = "Time, Lat, Long, Alt : "
                                 + std::to_string(nrx->mTimeWeekSecond) + ", " 
                                 + std::to_string(nrx->mLat) + ", "
                                 + std::to_string(nrx->mLon) + ", "
                                 + std::to_string(nrx->mAlt);

  return msg;
}


sensor_msgs::msg::Imu RosNComWrapper::imu (
                      const NComRxC *nrx,
                      std_msgs::msg::Header head)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header = head;

  auto q_vat = tf2::Quaternion(); // Quaternion representation of the vehicle-imu alignment
  auto veh_o = tf2::Quaternion(); // Orientation of the vehicle (NED frame)
  auto imu_o = tf2::Quaternion(); // Orientation of the IMU (ENU frame)
  auto veh_w = tf2::Vector3();    // Angular rate in vehicle frame (rads)
  auto imu_w = tf2::Vector3();    // Angular rate in imu frame (rads)
  auto veh_a = tf2::Vector3();    // Linear Acceleration in the vehicle frame (rads)
  auto imu_a = tf2::Vector3();    // Linear Acceleration in the imu frame (rads)

  // Construct vehicle-imu frame transformation --------------------------------
  q_vat = RosNComWrapper::getVat(nrx);
  // Get vehicle orientation from HPR ------------------------------------------
  veh_o = RosNComWrapper::getRPY(nrx); // ENU frame
  // Find imu orientation
  imu_o = veh_o * q_vat.inverse(); // vehicle to body
  tf2::convert(imu_o, msg.orientation);

  // Covariance = 0 => unknown. -1 => invalid
  msg.orientation_covariance[0] = 0.0;
  // ...
  msg.orientation_covariance[8] = 0.0;

  // Rotate angular rate data before copying into message ----------------------
  veh_w = tf2::Vector3(nrx->mWx, nrx->mWy, nrx->mWz);
  veh_w *= NAV_CONST::DEG2RADS;
  imu_w = tf2::quatRotate(q_vat, veh_w);
  msg.angular_velocity.x = imu_w.x();
  msg.angular_velocity.y = imu_w.y();
  msg.angular_velocity.z = imu_w.z();
  msg.angular_velocity_covariance[0] = 0.0; //Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // Rotate linear acceleration data -------------------------------------------
  veh_a = tf2::Vector3(nrx->mAx, nrx->mAy, nrx->mAz);
  imu_a = tf2::quatRotate(q_vat, veh_a);
  msg.linear_acceleration.x = imu_a.x();
  msg.linear_acceleration.y = imu_a.y();
  msg.linear_acceleration.z = imu_a.z();

  msg.linear_acceleration_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.linear_acceleration_covariance[8] = 0.0;

  return msg;
}

geometry_msgs::msg::TwistStamped   RosNComWrapper::velocity   (
                                   const NComRxC *nrx,
                                   std_msgs::msg::Header head)
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header = head;

  // Construct vehicle-imu frame transformation --------------------------------
  auto q_vat      = RosNComWrapper::getVat(nrx);
  auto r_vat      = tf2::Matrix3x3(q_vat);
  auto veh_v      = tf2::Vector3(nrx->mIsoVoX,nrx->mIsoVoY,nrx->mIsoVoZ);
  auto veh_w      = tf2::Vector3(nrx->mWx,nrx->mWy,nrx->mWz);
  auto imu_w      = tf2::Vector3();
  auto imu_v      = tf2::Vector3();
  auto q_iso_oxts = tf2::Quaternion();

  q_iso_oxts.setRPY(180.0*NAV_CONST::DEG2RADS,0.0,0.0);
  auto r_iso_oxts = tf2::Matrix3x3(q_iso_oxts);

  imu_v = r_vat * r_iso_oxts * veh_v;
  imu_w = r_vat * veh_w;

  msg.twist.linear.x  = imu_v.getX();
  msg.twist.linear.y  = imu_v.getY();
  msg.twist.linear.z  = imu_v.getZ();
  msg.twist.angular.x = imu_w.getX();
  msg.twist.angular.y = imu_w.getY();
  msg.twist.angular.z = imu_w.getZ();

  return msg;
}


  sensor_msgs::msg::TimeReference   RosNComWrapper::time_reference  (
                                    const NComRxC *nrx,
                                    std_msgs::msg::Header head)
{
  auto msg = sensor_msgs::msg::TimeReference();
  
  msg.header = head;

  msg.time_ref.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msg.time_ref.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS );
  msg.source = "ins"; 

  return msg;
}


