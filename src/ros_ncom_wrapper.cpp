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


nav_msgs::msg::Odometry RosNComWrapper::wrap_odometry (
                        const NComRxC *nrx,
                        std_msgs::msg::Header head,
                        nav_msgs::msg::Odometry prev)
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header = head;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Construct vehicle-imu frame transformation --------------------------------
  auto q_vat = tf2::Quaternion(); // Quaternion representation of the vehicle-imu alignment
  auto r_vat = tf2::Matrix3x3();
  q_vat = RosNComWrapper::wrap_vat_to_quaternion(nrx);
  q_vat.normalize();
  r_vat = tf2::Matrix3x3(q_vat);
  // Get FLD velocity data and angular rates from the NCom decoder. ------------
  // This data is in the vehicle frame
  auto veh_v = tf2::Vector3();
  auto veh_w = tf2::Vector3();
  auto imu_v = tf2::Vector3();
  auto imu_w = tf2::Vector3();
  veh_v.setX(nrx->mVf);
  veh_v.setY(nrx->mVl);
  veh_v.setZ(nrx->mVd);
  veh_w.setX(nrx->mWx);
  veh_w.setY(nrx->mWy);
  veh_w.setZ(nrx->mWz);

  // Convert this data to imu frame --------------------------------------------
  imu_v = r_vat * veh_v;
  imu_w = r_vat * veh_w;
  // Convert to base_link frame (Feature TBA) - until then, assume base_link ---
  // and imu_link are aligned.

  // Put velocity and angular rates (base_link) into twist ---------------------
  msg.twist.twist.linear.x  = imu_v.getX();
  msg.twist.twist.linear.y  = imu_v.getY();
  msg.twist.twist.linear.z  = imu_v.getZ();
  msg.twist.twist.angular.x = imu_w.getX();
  msg.twist.twist.angular.y = imu_w.getY();
  msg.twist.twist.angular.z = imu_w.getZ();

  msg.twist.covariance[ 0] = 0;
  msg.twist.covariance[ 7] = 0;
  msg.twist.covariance[14] = 0;
  msg.twist.covariance[21] = 0;
  msg.twist.covariance[28] = 0;
  msg.twist.covariance[35] = 0;

  // Transform velocity and angular rates into odom frame ----------------------

  // Position estimate calc: integrate velocity and angular rates --------------
  // double dt = msg.header.time.secs - prev.header.time.secs; 
  double dt = 0.01; 
  // double dt = (msg.header.stamp.nanosec - prev.header.stamp.nanosec)/1000.0; 
  auto imu_pose_o_prev = tf2::Quaternion();
  auto imu_pose_t_prev = tf2::Vector3();
  auto o_matrix        = tf2::Matrix3x3();

  //std::cout << "prev       : " << prev.pose.pose.orientation.x << "," << prev.pose.pose.orientation.y << "," << prev.pose.pose.orientation.z << std::endl;
  
  tf2::convert(prev.pose.pose.orientation, imu_pose_o_prev);
  imu_pose_t_prev.setX(prev.pose.pose.position.x); 
  imu_pose_t_prev.setY(prev.pose.pose.position.y); 
  imu_pose_t_prev.setZ(prev.pose.pose.position.z); 

  //std::cout << "post_t_prev: " << imu_pose_t_prev.getX() << "," << imu_pose_t_prev.getY() << "," << imu_pose_t_prev.getZ()<< std::endl;

  auto o = tf2::Quaternion();
  auto p = tf2::Vector3();
 
  o = o + imu_pose_o_prev * imu_w * dt * 0.5;
  o.normalize(); 
  //std::cout << "o:           " << o.getX() << "," << o.getY() << "," << o.getZ()<<o.getW()<< std::endl;

  o_matrix         = tf2::Matrix3x3(o);
  p = imu_pose_t_prev + o_matrix * imu_v * dt;
  
  // Put position estimate into pose -------------------------------------------
  msg.pose.pose.position.x = p.getX();
  msg.pose.pose.position.y = p.getY();
  msg.pose.pose.position.z = p.getZ();
  tf2::convert(o,msg.pose.pose.orientation);

  msg.pose.covariance[0] = 0.0;
  //  1 ... 34
  msg.pose.covariance[35] = 0.0;

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
  // Get vehicle orientation from HPR -------------------------------------------
  veh_o.setRPY(NAV_CONST::DEG2RADS * nrx->mRoll,
               NAV_CONST::DEG2RADS * nrx->mPitch,
               NAV_CONST::DEG2RADS * nrx->mHeading);
  imu_o =  q_vat * veh_o;
  tf2::convert(imu_o,msg.orientation);

  // Covariance = 0 => unknown. -1 => invalid
  msg.orientation_covariance[0] = 0.0;
  // ...
  msg.orientation_covariance[8] = 0.0;

  // Rotate angular rate data ----------------------------------
  veh_w.setX(NAV_CONST::DEG2RADS * nrx->mWx);
  veh_w.setY(NAV_CONST::DEG2RADS * nrx->mWy);
  veh_w.setZ(NAV_CONST::DEG2RADS * nrx->mWz);
  imu_w = r_vat * veh_w;
  msg.angular_velocity.x = imu_w.getX();
  msg.angular_velocity.y = imu_w.getY();
  msg.angular_velocity.z = imu_w.getZ();

  msg.angular_velocity_covariance[0] = 0.0;//Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // Rotate linear acceleration data ---------------------------
  veh_a.setX(nrx->mAx);
  veh_a.setY(nrx->mAy);
  veh_a.setZ(nrx->mAz);
  imu_a = r_vat * veh_a;
  msg.linear_acceleration.x = imu_a.getX();
  msg.linear_acceleration.y = imu_a.getY();
  msg.linear_acceleration.z = imu_a.getZ();

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