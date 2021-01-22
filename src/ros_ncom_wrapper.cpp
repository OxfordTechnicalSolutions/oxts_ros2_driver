#include "ros-driver/ros_ncom_wrapper.hpp"


geometry_msgs::msg::TransformStamped RosNComWrapper::wrap_vat_to_tf(
                                     const NComRxC *nrx)
{
  auto tfStamped = geometry_msgs::msg::TransformStamped();
  tf2::Quaternion q_align; 
  geometry_msgs::msg::Quaternion q_align_msg;
  q_align.setEuler(NAV_CONST::DEG2RADS * nrx->mImu2VehHeading,
                   NAV_CONST::DEG2RADS * nrx->mImu2VehPitch,
                   NAV_CONST::DEG2RADS * nrx->mImu2VehRoll    );
  //q_align = q_align.inverse();
  tf2::convert(q_align,q_align_msg);
  tfStamped.transform.rotation = q_align_msg;

  return tfStamped;
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


nav_msgs::msg::Odometry RosNComWrapper::wrap_odometry (
                        const NComRxC *nrx,
                        std_msgs::msg::Header head)
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header = head;
  msg.header.frame_id = "earth";
  msg.child_frame_id = "odom";

  // Together, msgs Point and Quaternion make a geometry_msgs/Pose
  // geometry_msgs/msg/Point

  // We need to convert WGS84 to ECEF, the "global" frame in ROS2
  std::vector<double> transformVec(3);
  transformVec = Convert::lla_to_ecef(nrx->mLat, nrx->mLon, nrx->mAlt);

  msg.pose.pose.position.x = transformVec[0];
  msg.pose.pose.position.y = transformVec[1];
  msg.pose.pose.position.z = transformVec[2];


  // geometry_msgs/msg/Quaternion
  tf2::Quaternion q;
  q.setRPY(NAV_CONST::DEG2RADS * nrx->mRoll,
           NAV_CONST::DEG2RADS * nrx->mPitch,
           NAV_CONST::DEG2RADS * nrx->mHeading );
           
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  
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
  msg.data = "Time, Lat, Long, Alt : "+ std::to_string(nrx->mTimeWeekSecond) + ", " 
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

  auto t_vat_msg             = geometry_msgs::msg::TransformStamped(); // vehicle-imu frame transform
  auto q_veh_orientation_msg = geometry_msgs::msg::Quaternion();
  auto veh_w                 = geometry_msgs::msg::Vector3Stamped();
  auto imu_w                 = geometry_msgs::msg::Vector3Stamped();
  auto vm_in                 = geometry_msgs::msg::Vector3Stamped(); // vehicle frame acc.
  auto vm_out                = geometry_msgs::msg::Vector3Stamped(); // imu frame acc.
  
  // Construct vehicle-imu frame transformation --------------------------------
  tf2::Quaternion q_vat; // Quaternion representation of the vehicle-imu alignment
  t_vat_msg = RosNComWrapper::wrap_vat_to_tf(nrx);
  tf2::convert(t_vat_msg.transform.rotation,q_vat);
  // Rotate orientation data using q_vat ---------------------------------------
  tf2::Quaternion q_veh_orientation;
  q_veh_orientation.setEuler(NAV_CONST::DEG2RADS * nrx->mHeading,
                           NAV_CONST::DEG2RADS * nrx->mPitch,
                           NAV_CONST::DEG2RADS * nrx->mRoll );
  tf2::convert(q_veh_orientation,q_veh_orientation_msg);

  tf2::Quaternion q_vat_temp = q_vat;
  q_vat_temp *= q_veh_orientation;
  //q_veh_orientation *= q_vat ;
  // tf2::convert(q_veh_orientation,msg.orientation);
  tf2::convert(q_vat_temp,msg.orientation);

  // Covariance = 0 => unknown. -1 => invalid
  msg.orientation_covariance[0] = 0.0;
  // ...
  msg.orientation_covariance[8] = 0.0;

  // Rotate angular rate data using t_vat_msg ----------------------------------

  veh_w.vector.x = NAV_CONST::DEG2RADS * nrx->mWx;
  veh_w.vector.y = NAV_CONST::DEG2RADS * nrx->mWy;
  veh_w.vector.z = NAV_CONST::DEG2RADS * nrx->mWz;
  tf2::doTransform(veh_w,imu_w,t_vat_msg);
  msg.angular_velocity = imu_w.vector;

  msg.angular_velocity_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // Rotate linear acceleration data using t_vat_msg ---------------------------
  vm_in.vector.x = nrx->mAx;
  vm_in.vector.y = nrx->mAy;
  vm_in.vector.z = nrx->mAz;
  tf2::doTransform(vm_in,vm_out,t_vat_msg); // Apply transformation to acc.
  msg.linear_acceleration = vm_out.vector;

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


geometry_msgs::msg::TransformStamped  RosNComWrapper::wrap_tf2   (
                                      const NComRxC *nrx,
                                      std_msgs::msg::Header head)
{
  auto msg = geometry_msgs::msg::TransformStamped();
  msg.header = head;
  msg.header.frame_id = "earth";
  msg.child_frame_id  = "imu";
  // We need to convert WGS84 to ECEF, the "global" frame in ROS2
  std::vector<double> transformVec(3);
  transformVec = Convert::lla_to_ecef(nrx->mLat, nrx->mLon, nrx->mAlt);

  msg.transform.translation.x = transformVec[0];
  msg.transform.translation.y = transformVec[1];
  msg.transform.translation.z = transformVec[2];

  tf2::Quaternion q;

  q.setEuler(NAV_CONST::DEG2RADS * nrx->mHeading,
             NAV_CONST::DEG2RADS * nrx->mPitch,
             NAV_CONST::DEG2RADS * nrx->mRoll );

  tf2::convert(q,msg.transform.rotation);


  return msg;
}