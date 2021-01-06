#include "ros-driver/ros_ncom_wrapper.hpp"




std_msgs::msg::Header RosNComWrapper::wrap_header_ncom_time(const NComRxC *nrx)
{
  auto header = std_msgs::msg::Header();

  header.stamp.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  header.stamp.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS );
  header.frame_id = "WGS84"; /*! @todo Change this */

  return header;
}

sensor_msgs::msg::NavSatStatus RosNComWrapper::wrap_nav_sat_status(const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::NavSatStatus();

  switch(nrx->mGpsPosMode)
  {
    // No Fix
    case NAV_CONST::GNSS_MODE::NONE          :
    case NAV_CONST::GNSS_MODE::SEARCH        :
    case NAV_CONST::GNSS_MODE::DOPPLER       :
    case NAV_CONST::GNSS_MODE::NODATA        :
    case NAV_CONST::GNSS_MODE::BLANKED       :
    case NAV_CONST::GNSS_MODE::PP_DOPPLER    :
    case NAV_CONST::GNSS_MODE::NOT_KNOWN     :
    case NAV_CONST::GNSS_MODE::GX_DOPPLER    :
    case NAV_CONST::GNSS_MODE::IX_DOPPLER    :
    case NAV_CONST::GNSS_MODE::UNKNOWN       :
      msg.status = msg.STATUS_NO_FIX;
      break;
    // Fix
    case NAV_CONST::GNSS_MODE::SPS           :
    case NAV_CONST::GNSS_MODE::PP_SPS        :
    case NAV_CONST::GNSS_MODE::GX_SPS        :
    case NAV_CONST::GNSS_MODE::IX_SPS        :
    case NAV_CONST::GNSS_MODE::GENAID        :
    case NAV_CONST::GNSS_MODE::SEGMENT       :
      msg.status = msg.STATUS_FIX;
      break;
    // Ground-based augmentation
    case NAV_CONST::GNSS_MODE::DIFF          :
    case NAV_CONST::GNSS_MODE::FLOAT         :
    case NAV_CONST::GNSS_MODE::INTEGER       :
    case NAV_CONST::GNSS_MODE::PP_DIFF       :
    case NAV_CONST::GNSS_MODE::PP_FLOAT      :
    case NAV_CONST::GNSS_MODE::PP_INTEGER    :
    case NAV_CONST::GNSS_MODE::GX_DIFF       :
    case NAV_CONST::GNSS_MODE::GX_FLOAT      :
    case NAV_CONST::GNSS_MODE::GX_INTEGER    :
    case NAV_CONST::GNSS_MODE::IX_DIFF       :
    case NAV_CONST::GNSS_MODE::IX_FLOAT      :
    case NAV_CONST::GNSS_MODE::IX_INTEGER    :
      msg.status = msg.STATUS_GBAS_FIX;
      break;
    // Satellite-based augmentation
    case NAV_CONST::GNSS_MODE::WAAS          :
    case NAV_CONST::GNSS_MODE::OMNISTAR      :
    case NAV_CONST::GNSS_MODE::OMNISTARHP    :
    case NAV_CONST::GNSS_MODE::OMNISTARXP    :
    case NAV_CONST::GNSS_MODE::CDGPS         :
    case NAV_CONST::GNSS_MODE::PPP_CONVERGING:
    case NAV_CONST::GNSS_MODE::PPP           :
    case NAV_CONST::GNSS_MODE::GX_SBAS       :
    case NAV_CONST::GNSS_MODE::IX_SBAS       :
      msg.status = msg.STATUS_SBAS_FIX;
      break;
  }

  msg.service = 0; /*! @todo Output value based on use of constellations */

  return msg;
}

sensor_msgs::msg::NavSatFix RosNComWrapper::wrap_nav_sat_fix(const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::NavSatFix();

  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);

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


nav_msgs::msg::Odometry RosNComWrapper::wrap_odometry (const NComRxC *nrx)
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);

  msg.child_frame_id = "";

  // Together, msgs Point and Quaternion make a geometry_msgs/Pose
  // geometry_msgs/msg/Point
  msg.pose.pose.position.x = nrx->mLat; // float64, make local coords
  msg.pose.pose.position.y = nrx->mLon; // float64, make local coords
  msg.pose.pose.position.z = nrx->mAlt; // float64, make local coords

  std::vector<double> quaternion = Convert::hpr_to_quaternion(nrx->mHeading,
                                                              nrx->mPitch,
                                                              nrx->mRoll);

  // geometry_msgs/msg/Quaternion
  msg.pose.pose.orientation.x = quaternion[0]; // float64
  msg.pose.pose.orientation.y = quaternion[1]; // float64
  msg.pose.pose.orientation.z = quaternion[2]; // float64
  msg.pose.pose.orientation.w = quaternion[3]; // float64
  
  msg.pose.covariance[0] = 0.0;
  //  1 ... 34
  msg.pose.covariance[35] = 0.0;

  // geometry_msgs/msg/TwistWithCovariance
  // This expresses velocity in free space broken into its linear and angular parts.
  msg.twist.twist.linear.x  =  nrx->mVe; 
  msg.twist.twist.linear.y  =  nrx->mVn; 
  msg.twist.twist.linear.z  = -nrx->mVd; 
  msg.twist.twist.angular.x =  nrx->mWx; /*! @todo also needs converting to enu */ 
  msg.twist.twist.angular.y =  nrx->mWy; 
  msg.twist.twist.angular.z =  nrx->mWz; 

  msg.twist.covariance[ 0] = nrx->mVeAcc;
  msg.twist.covariance[ 7] = nrx->mVnAcc;
  msg.twist.covariance[14] = nrx->mVdAcc;
  msg.twist.covariance[21] = 0;
  msg.twist.covariance[28] = 0;
  msg.twist.covariance[35] = 0;

  return msg;
}


std_msgs::msg::String RosNComWrapper::wrap_string (const NComRxC *nrx)
{
  auto msg = std_msgs::msg::String();
  msg.data = "Lat, Long, Alt : " + std::to_string(nrx->mLat) + ", "
                                 + std::to_string(nrx->mLon) + ", "
                                 + std::to_string(nrx->mAlt);

  return msg;
}


sensor_msgs::msg::Imu RosNComWrapper::wrap_imu (const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);

  // geometry_msgs/Quaternion
  tf2::Quaternion q;

  q.setRPY(NAV_CONST::DEG2RADS * nrx->mRoll,
           NAV_CONST::DEG2RADS * nrx->mPitch,
           NAV_CONST::DEG2RADS * nrx->mHeading );
           
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  // Covariance = 0 => unknown. -1 => invalid
  msg.orientation_covariance[0] = 0.0;
  // ...
  msg.orientation_covariance[8] = 0.0;

  // geometry_msgs/Vector3
  msg.angular_velocity.x = nrx->mWx;
  msg.angular_velocity.y = nrx->mWy;
  msg.angular_velocity.z = nrx->mWz;

  msg.angular_velocity_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // geometry_msgs/Vector3
  msg.linear_acceleration.x = nrx->mAx;
  msg.linear_acceleration.y = nrx->mAy;
  msg.linear_acceleration.z = nrx->mAz;

  msg.linear_acceleration_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.linear_acceleration_covariance[8] = 0.0;

  return msg;
}

geometry_msgs::msg::TwistStamped   RosNComWrapper::wrap_velocity   (const NComRxC *nrx)
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);

  msg.twist.linear.x  = nrx->mVf;
  msg.twist.linear.y  = nrx->mVl;
  msg.twist.linear.z  = nrx->mVd;
  msg.twist.angular.x = nrx->mWf;
  msg.twist.angular.y = nrx->mWl;
  msg.twist.angular.z = nrx->mWd;

  return msg;
}


sensor_msgs::msg::TimeReference   RosNComWrapper::wrap_ins_time   (const NComRxC *nrx)
{
  auto msg = sensor_msgs::msg::TimeReference();

  /*! @todo This should be system time not ncom time in header */
  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);

  msg.time_ref.sec     = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msg.time_ref.nanosec = static_cast<uint32_t>(
    (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS );
  msg.source = "ins"; 

  return msg;
}


geometry_msgs::msg::TransformStamped   RosNComWrapper::wrap_tf2   (const NComRxC *nrx)
{
  auto msg = geometry_msgs::msg::TransformStamped();
  msg.header = RosNComWrapper::wrap_header_ncom_time(nrx);
  msg.header.frame_id = "earth";
  msg.child_frame_id =  "imu";
  // We need to convert WGS84 to ECEF, the "global" frame in ROS2

  double radius = 6378137.0;
  double flatFactor = (1.0 / 298.257223563) ;
  double cosLat = std::cos(nrx->mLat * NAV_CONST::DEG2RADS);
  double sinLat = std::sin(nrx->mLat * NAV_CONST::DEG2RADS);
  double flatFactorSqr = std::pow((1.0-flatFactor),2);
  double c = 1/std::sqrt(std::pow(cosLat,2) + (flatFactorSqr * std::pow(sinLat,2)));
  double s = c * flatFactorSqr;

  msg.transform.translation.x = ((radius * c) + nrx->mAlt) * cosLat * std::cos(NAV_CONST::DEG2RADS*nrx->mLon);
  msg.transform.translation.y = ((radius * c) + nrx->mAlt) * cosLat * std::sin(NAV_CONST::DEG2RADS*nrx->mLon);
  msg.transform.translation.z = ((radius * s) + nrx->mAlt) * sinLat;

  tf2::Quaternion q;

  q.setRPY(NAV_CONST::DEG2RADS * nrx->mRoll,
           NAV_CONST::DEG2RADS * nrx->mPitch,
           NAV_CONST::DEG2RADS * nrx->mHeading );

  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();

  return msg;
}