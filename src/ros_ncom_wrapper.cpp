#include "ros-driver/ros_ncom_wrapper.hpp"


tf2::Quaternion RosNComWrapper::wrap_vat_to_quaternion(
                                     const NComRxC *nrx)
{
  tf2::Quaternion q_align; 
  q_align.setRPY(
    NAV_CONST::DEG2RADS * nrx->mImu2VehRoll,
    NAV_CONST::DEG2RADS * nrx->mImu2VehPitch,
    NAV_CONST::DEG2RADS * nrx->mImu2VehHeading
  );
  return q_align;
}


rclcpp::Time      RosNComWrapper::ncom_time_to_time(const NComRxC *nrx)
{
  auto time = rclcpp::Time(static_cast<int32_t>(nrx->mTimeWeekSecond) + 
                           (nrx->mTimeWeekCount * NAV_CONST::WEEK_SECS) + 
                           nrx->mTimeUtcOffset + NAV_CONST::GPS2UNIX_EPOCH,
  static_cast<uint32_t>((nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS ));

  return time;
}

std_msgs::msg::Header       RosNComWrapper::wrap_header(rclcpp::Time time,
                                                        std::string frame)
{
  auto header = std_msgs::msg::Header();

  header.stamp = time;
  header.frame_id = frame; 

  return header;
}

sensor_msgs::msg::NavSatStatus RosNComWrapper::wrap_nav_sat_status(
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

sensor_msgs::msg::NavSatFix RosNComWrapper::wrap_nav_sat_fix(
                            const NComRxC *nrx,
                            std_msgs::msg::Header head)
{
  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header = head;

  msg.status = RosNComWrapper::wrap_nav_sat_status(nrx);

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


geometry_msgs::msg::PoseWithCovarianceStamped RosNComWrapper::wrap_pose_ecef
                                              (
                                              const NComRxC *nrx,
                                              std_msgs::msg::Header head
                                              )
{
  auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  msg.header = head;

  std::vector<double> ecef = Convert::lla_to_ecef(nrx->mLat, nrx->mLon, nrx->mAlt);

  msg.pose.pose.position.x    = ecef[0]; 
  msg.pose.pose.position.y    = ecef[1];
  msg.pose.pose.position.z    = ecef[2];

  auto q_vat = tf2::Quaternion();
  auto veh_o = tf2::Quaternion();
  auto imu_o = tf2::Quaternion();

  // Construct vehicle-imu frame transformation --------------------------------
  q_vat = RosNComWrapper::wrap_vat_to_quaternion(nrx);
  // Get vehicle orientation from HPR ------------------------------------------
  veh_o.setRPY(
               NAV_CONST::DEG2RADS * nrx->mRoll,
               NAV_CONST::DEG2RADS * nrx->mPitch,
               NAV_CONST::DEG2RADS * nrx->mHeading
               );

  // NED to ENU rotation
  auto q_ned_enu = tf2::Quaternion();
  q_ned_enu.setRPY(180*NAV_CONST::DEG2RADS,0,90*NAV_CONST::DEG2RADS);

  imu_o =  q_vat * veh_o * q_ned_enu;
  
  msg.pose.pose.orientation.x = imu_o.getY();
  msg.pose.pose.orientation.y = imu_o.getX();
  msg.pose.pose.orientation.z = imu_o.getZ();
  msg.pose.pose.orientation.w = imu_o.getW();

  msg.pose.covariance[0] = 0;
  // ...
  msg.pose.covariance[35] = 0;

  return msg;
}

std_msgs::msg::String RosNComWrapper::wrap_string (const NComRxC *nrx)
{
  auto msg = std_msgs::msg::String();
  msg.data = "Time, Lat, Long, Alt : "
                                 + std::to_string(nrx->mTimeWeekSecond) + ", " 
                                 + std::to_string(nrx->mLat) + ", "
                                 + std::to_string(nrx->mLon) + ", "
                                 + std::to_string(nrx->mAlt);

  return msg;
}


sensor_msgs::msg::Imu RosNComWrapper::wrap_imu (
                      const NComRxC *nrx,
                      std_msgs::msg::Header head)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header = head;

  auto q_vat = tf2::Quaternion(); // Quaternion representation of the vehicle-imu alignment
  auto r_vat = tf2::Matrix3x3();
  auto veh_o = tf2::Quaternion();
  auto imu_o = tf2::Quaternion();
  auto veh_w = tf2::Vector3();
  auto imu_w = tf2::Vector3();
  auto veh_a = tf2::Vector3();
  auto imu_a = tf2::Vector3();

  // Construct vehicle-imu frame transformation --------------------------------
  q_vat = RosNComWrapper::wrap_vat_to_quaternion(nrx);
  r_vat = tf2::Matrix3x3(q_vat);
  // Get vehicle orientation from HPR ------------------------------------------
  // Order would be RPY, we give it PR-Y to convert from NED to ENU
  veh_o.setRPY(
               NAV_CONST::DEG2RADS * nrx->mRoll,
               NAV_CONST::DEG2RADS * nrx->mPitch,
               NAV_CONST::DEG2RADS * nrx->mHeading
               );
  // NED to ENU rotation
  auto q_ned_enu = tf2::Quaternion();
  q_ned_enu.setRPY(180*NAV_CONST::DEG2RADS,0,90*NAV_CONST::DEG2RADS);

  // Find imu orientation
  imu_o =  q_vat * veh_o * q_ned_enu;
  tf2::convert(imu_o,msg.orientation);
  
  // Covariance = 0 => unknown. -1 => invalid
  msg.orientation_covariance[0] = 0.0;
  // ...
  msg.orientation_covariance[8] = 0.0;

  // Rotate angular rate data before copying into message ----------------------
  veh_w.setX(NAV_CONST::DEG2RADS * nrx->mWx);
  veh_w.setY(NAV_CONST::DEG2RADS * nrx->mWy);
  veh_w.setZ(NAV_CONST::DEG2RADS * nrx->mWz);
  imu_w = r_vat * veh_w;
  msg.angular_velocity.x = imu_w.getX();
  msg.angular_velocity.y = imu_w.getY();
  msg.angular_velocity.z = imu_w.getZ();
  //std::cout << "gyroscope:   " << imu_w.getX() << ", " << imu_w.getY() << ", " << imu_w.getY() << std::endl;
  msg.angular_velocity_covariance[0] = 0.0; //Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // Rotate linear acceleration data -------------------------------------------
  veh_a.setX(nrx->mAx);
  veh_a.setY(nrx->mAy);
  veh_a.setZ(nrx->mAz);
  imu_a = r_vat * veh_a;
  msg.linear_acceleration.x = imu_a.getX();
  msg.linear_acceleration.y = imu_a.getY();
  msg.linear_acceleration.z = imu_a.getZ();
  //std::cout << "lin acc:     " << imu_a.getX() << ", " << imu_a.getY() << ", " << imu_a.getY() << std::endl;

  msg.linear_acceleration_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.linear_acceleration_covariance[8] = 0.0;

  return msg;
}

geometry_msgs::msg::TwistStamped   RosNComWrapper::wrap_velocity   (
                                   const NComRxC *nrx,
                                   std_msgs::msg::Header head)
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header = head;

  msg.twist.linear.x  = nrx->mVf;
  msg.twist.linear.y  = nrx->mVl;
  msg.twist.linear.z  = nrx->mVd;
  msg.twist.angular.x = nrx->mWf;
  msg.twist.angular.y = nrx->mWl;
  msg.twist.angular.z = nrx->mWd;

  return msg;
}


  sensor_msgs::msg::TimeReference   RosNComWrapper::wrap_time_reference  (
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


