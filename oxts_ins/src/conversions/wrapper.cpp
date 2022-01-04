// Copyright 2021 Oxford Technical Solutions Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <utility>

#include "oxts_ins/wrapper.hpp"

namespace RosNComWrapper {

**
 * Helper function for constructing a quaternion from Euler angles, where the
 * rotations are applied in the order qx(roll), qy(pitch), qz(yaw)
 * 
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 * @return A quaternion representing a roll-pitch-yaw rotation.
 */
tf2::Quaternion fromEulerYPR(const double roll, const double pitch, const double yaw)
{
  // Construct the individual rotations
  tf2::Quaternion qx(std::cos(roll / 2.0), std::sin(roll / 2.0), 0.0, 0.0);
  tf2::Quaternion qy(std::cos(pitch / 2.0), 0.0, std::sin(pitch / 2.0), 0.0);
  tf2::Quaternion qz(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
  // Return the compound rotation in the correct order
  return qz * qy * qx;
}
  
tf2::Quaternion getVat(const NComRxC *nrx) {
  tf2::Quaternion vat = RosNComWrapper::fromEulerYPR(
    NAV_CONST::DEG2RADS * nrx->mImu2VehRoll, 
    NAV_CONST::DEG2RADS * nrx->mImu2VehPitch, 
    NAV_CONST::DEG2RADS * nrx->mImu2VehHeading);
  return vat;
}

tf2::Vector3 getNsp(const NComRxC *nrx) {
  // Translation of rear axle in imu frame
  return tf2::Vector3(nrx->mNoSlipLeverArmX, nrx->mNoSlipLeverArmY,
                      nrx->mNoSlipLeverArmZ);
}

// tf2::Vector3 getNvsp(const NComRxC *nrx)
// {
//   // Translation of rear axle in imu frame
//   return tf2::Vector3(
//     nrx->mNoSlipLeverArmX,
//     nrx->mNoSlipLeverArmY,
//     nrx->mNoSlipLeverArmZ);
// }

tf2::Quaternion getVehRPY(const NComRxC *nrx) {
  auto rpyVehNED = tf2::Quaternion(); // Orientation of the vehicle (NED frame)
  auto rpyVehENU = tf2::Quaternion(); // Orientation of the vehicle (ENU frame)
  auto ned2enu = tf2::Quaternion(); // NED to ENU rotation

  rpyVehNED = RosNComWrapper::fromEulerYPR(
    NAV_CONST::DEG2RADS * nrx->mRoll,
    NAV_CONST::DEG2RADS * nrx->mPitch,
    NAV_CONST::DEG2RADS * nrx->mHeading
  );
  // NED to ENU rotation
  //ned2enu.setRPY(180.0*NAV_CONST::DEG2RADS,0,90.0*NAV_CONST::DEG2RADS);
  // transform from NED to ENU
  rpyVehENU = rpyVehNED;// ned2enu * rpyVehNED;
  return rpyVehENU;
}

tf2::Quaternion getBodyRPY(const NComRxC *nrx) {
  auto rpyVehENU =
      RosNComWrapper::getVehRPY(nrx); // Orientation of the vehicle (ENU frame)
  auto rpyBodENU = tf2::Quaternion(); // Orientation of the body (ENU frame)
  auto vat = RosNComWrapper::getVat(nrx); // Body to vehicle frame rotation

  // transform from vehicle to body
  rpyBodENU = rpyVehENU * vat.inverse(); // vehicle to body
  return rpyBodENU;
}

Lrf getNcomLrf(const NComRxC *nrx) {
  // Origin to use for map frame
  return Lrf(nrx->mRefLat, nrx->mRefLon, nrx->mRefAlt,
             // mRefHeading is in NED. Get angle between ENU and LRF
             (nrx->mRefHeading - 90) * NAV_CONST::DEG2RADS);
}

sensor_msgs::msg::NavSatStatus nav_sat_status(const NComRxC *nrx) {
  auto msg = sensor_msgs::msg::NavSatStatus();

  switch (nrx->mGpsPosMode) {
  // No Fix
  case NAV_CONST::GNSS_POS_MODE::NONE:
  case NAV_CONST::GNSS_POS_MODE::SEARCH:
  case NAV_CONST::GNSS_POS_MODE::DOPPLER:
  case NAV_CONST::GNSS_POS_MODE::NODATA:
  case NAV_CONST::GNSS_POS_MODE::BLANKED:
  case NAV_CONST::GNSS_POS_MODE::PP_DOPPLER:
  case NAV_CONST::GNSS_POS_MODE::NOT_KNOWN:
  case NAV_CONST::GNSS_POS_MODE::GX_DOPPLER:
  case NAV_CONST::GNSS_POS_MODE::IX_DOPPLER:
  case NAV_CONST::GNSS_POS_MODE::UNKNOWN:
    msg.status = msg.STATUS_NO_FIX;
    break;
  // Fix
  case NAV_CONST::GNSS_POS_MODE::SPS:
  case NAV_CONST::GNSS_POS_MODE::PP_SPS:
  case NAV_CONST::GNSS_POS_MODE::GX_SPS:
  case NAV_CONST::GNSS_POS_MODE::IX_SPS:
  case NAV_CONST::GNSS_POS_MODE::GENAID:
  case NAV_CONST::GNSS_POS_MODE::SEGMENT:
    msg.status = msg.STATUS_FIX;
    break;
  // Ground-based augmentation
  case NAV_CONST::GNSS_POS_MODE::DIFF:
  case NAV_CONST::GNSS_POS_MODE::FLOAT:
  case NAV_CONST::GNSS_POS_MODE::INTEGER:
  case NAV_CONST::GNSS_POS_MODE::PP_DIFF:
  case NAV_CONST::GNSS_POS_MODE::PP_FLOAT:
  case NAV_CONST::GNSS_POS_MODE::PP_INTEGER:
  case NAV_CONST::GNSS_POS_MODE::GX_DIFF:
  case NAV_CONST::GNSS_POS_MODE::GX_FLOAT:
  case NAV_CONST::GNSS_POS_MODE::GX_INTEGER:
  case NAV_CONST::GNSS_POS_MODE::IX_DIFF:
  case NAV_CONST::GNSS_POS_MODE::IX_FLOAT:
  case NAV_CONST::GNSS_POS_MODE::IX_INTEGER:
    msg.status = msg.STATUS_GBAS_FIX;
    break;
  // Satellite-based augmentation
  case NAV_CONST::GNSS_POS_MODE::WAAS:
  case NAV_CONST::GNSS_POS_MODE::OMNISTAR:
  case NAV_CONST::GNSS_POS_MODE::OMNISTARHP:
  case NAV_CONST::GNSS_POS_MODE::OMNISTARXP:
  case NAV_CONST::GNSS_POS_MODE::CDGPS:
  case NAV_CONST::GNSS_POS_MODE::PPP_CONVERGING:
  case NAV_CONST::GNSS_POS_MODE::PPP:
  case NAV_CONST::GNSS_POS_MODE::GX_SBAS:
  case NAV_CONST::GNSS_POS_MODE::IX_SBAS:
    msg.status = msg.STATUS_SBAS_FIX;
    break;
  }

  msg.service = 0; /*! @todo Output value based on use of constellations */

  return msg;
}

sensor_msgs::msg::NavSatFix nav_sat_fix(const NComRxC *nrx,
                                        std_msgs::msg::Header head) {
  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header = std::move(head);

  msg.status = nav_sat_status(nrx);

  msg.latitude = nrx->mLat;
  msg.longitude = nrx->mLon;
  msg.altitude = nrx->mAlt;

  // Square accuracy to get variance (could be incorrect)
  msg.position_covariance[0] = std::pow(nrx->mEastAcc, 2);
  msg.position_covariance[4] = std::pow(nrx->mNorthAcc, 2);
  msg.position_covariance[8] = std::pow(nrx->mAltAcc, 2);

  msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return msg;
}

oxts_msgs::msg::NavSatRef nav_sat_ref(Lrf lrf, std_msgs::msg::Header head) {
  auto msg = oxts_msgs::msg::NavSatRef();
  msg.header = std::move(head);
  msg.latitude = lrf.lat();
  msg.longitude = lrf.lon();
  msg.altitude = lrf.alt();
  msg.heading = lrf.heading();
  return msg;
}

oxts_msgs::msg::LeverArm lever_arm_gap(const NComRxC *nrx,
                                       std_msgs::msg::Header head) {
  auto msg = oxts_msgs::msg::LeverArm();
  msg.header = std::move(head);
  msg.lever_arm_id = "gap";
  msg.offset_x = nrx->mGAPx;
  msg.offset_y = nrx->mGAPy;
  msg.offset_z = nrx->mGAPz;
  return msg;
}

oxts_msgs::msg::ImuBias imu_bias(const NComRxC *nrx,
                                 std_msgs::msg::Header head) {
  auto msg = oxts_msgs::msg::ImuBias();
  msg.header = std::move(head);
  // Accelerometer biases
  msg.accel.x = nrx->mAxBias;
  msg.accel.y = nrx->mAyBias;
  msg.accel.z = nrx->mAzBias;
  // Gyroscope biases
  msg.gyro.x = nrx->mWxBias;
  msg.gyro.y = nrx->mWyBias;
  msg.gyro.z = nrx->mWzBias;
  return msg;
}

geometry_msgs::msg::PointStamped ecef_pos(const NComRxC *nrx,
                                          std_msgs::msg::Header head) {
  auto msg = geometry_msgs::msg::PointStamped();
  msg.header = std::move(head);

  Point::Cart ecef =
      NavConversions::geodeticToEcef(nrx->mLat, nrx->mLon, nrx->mAlt);
  msg.point.x = ecef.x();
  msg.point.y = ecef.y();
  msg.point.z = ecef.z();

  return msg;
}

std_msgs::msg::String string(const NComRxC *nrx) {
  auto msg = std_msgs::msg::String();
  msg.data = "Time, Lat, Long, Alt : " + std::to_string(nrx->mTimeWeekSecond) +
             ", " + std::to_string(nrx->mLat) + ", " +
             std::to_string(nrx->mLon) + ", " + std::to_string(nrx->mAlt);

  return msg;
}

sensor_msgs::msg::Imu imu(const NComRxC *nrx, std_msgs::msg::Header head) {
  auto msg = sensor_msgs::msg::Imu();
  msg.header = std::move(head);

  auto q_vat = tf2::Quaternion(); // Quaternion representation of the
                                  // vehicle-imu alignment
  auto imu_o = tf2::Quaternion(); // Orientation of the IMU (ENU frame)
  auto veh_w = tf2::Vector3();    // Angular rate in vehicle frame (rads)
  auto imu_w = tf2::Vector3();    // Angular rate in imu frame (rads)
  auto veh_a =
      tf2::Vector3(); // Linear Acceleration in the vehicle frame (rads)
  auto imu_a = tf2::Vector3(); // Linear Acceleration in the imu frame (rads)

  // Construct vehicle-imu frame transformation --------------------------------
  q_vat = getVat(nrx);

  // Get imu orientation from NCOM packet -------------------------------------
  imu_o = getBodyRPY(nrx); // ENU frame
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
  msg.angular_velocity_covariance[0] = 0.0; // Row major about x, y, z axes
  // ...
  msg.angular_velocity_covariance[8] = 0.0;

  // Rotate linear acceleration data -------------------------------------------
  veh_a = tf2::Vector3(nrx->mAx, nrx->mAy, nrx->mAz);
  imu_a = tf2::quatRotate(q_vat, veh_a);
  msg.linear_acceleration.x = imu_a.x();
  msg.linear_acceleration.y = imu_a.y();
  msg.linear_acceleration.z = imu_a.z();

  msg.linear_acceleration_covariance[0] = 0.0; // Row major about x, y, z axes
  // ...
  msg.linear_acceleration_covariance[8] = 0.0;

  return msg;
}

geometry_msgs::msg::TwistStamped velocity(const NComRxC *nrx,
                                          std_msgs::msg::Header head) {
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header = std::move(head);

  // Construct vehicle-imu frame transformation --------------------------------
  auto q_vat = getVat(nrx);
  auto r_vat = tf2::Matrix3x3(q_vat);
  auto veh_v = tf2::Vector3(nrx->mIsoVoX, nrx->mIsoVoY, nrx->mIsoVoZ);
  auto veh_w = tf2::Vector3(nrx->mWx, nrx->mWy, nrx->mWz);
  auto imu_w = tf2::Vector3();
  auto imu_v = tf2::Vector3();
  auto q_iso_oxts = tf2::Quaternion();

  q_iso_oxts.setRPY(180.0 * NAV_CONST::DEG2RADS, 0.0, 0.0);
  auto r_iso_oxts = tf2::Matrix3x3(q_iso_oxts);

  imu_v = r_vat * r_iso_oxts * veh_v;
  imu_w = r_vat * veh_w;

  msg.twist.linear.x = imu_v.getX();
  msg.twist.linear.y = imu_v.getY();
  msg.twist.linear.z = imu_v.getZ();
  msg.twist.angular.x = imu_w.getX();
  msg.twist.angular.y = imu_w.getY();
  msg.twist.angular.z = imu_w.getZ();

  return msg;
}

tf2::Matrix3x3 getRotEnuToEcef(double lat0, double lon0) {
  double lambda = (lon0)*NAV_CONST::DEG2RADS;
  double phi = (lat0)*NAV_CONST::DEG2RADS;

  double s_phi = std::sin(phi);
  double c_phi = std::cos(phi);
  double s_lambda = std::sin(lambda);
  double c_lambda = std::cos(lambda);

  auto r = tf2::Matrix3x3(-s_lambda, -c_lambda * s_phi, c_lambda * c_phi,
                           c_lambda, -s_lambda * s_phi, s_lambda * c_phi,
                                  0,             c_phi,            s_phi);

  return r;
}

tf2::Matrix3x3 getRotEnuToLrf(double theta) {
  double s_theta = std::sin(theta);
  double c_theta = std::cos(theta);

  return tf2::Matrix3x3(c_theta, -s_theta, 0,
                        s_theta,  c_theta, 0,
                              0,        0, 1);
}

nav_msgs::msg::Odometry odometry(const NComRxC *nrx,
                                 const std_msgs::msg::Header &head, Lrf lrf) {
  auto msg = nav_msgs::msg::Odometry();
  msg.header = head;
  msg.child_frame_id = "oxts_link";
  // pose with covariance ======================================================

  Point::Cart p_enu;
  p_enu = NavConversions::geodeticToEnu(nrx->mLat, nrx->mLon, nrx->mAlt,
                                        lrf.lat(), lrf.lon(), lrf.alt());

  Point::Cart p_lrf =
      NavConversions::enuToLrf(p_enu.x(), p_enu.y(), p_enu.z(), lrf.heading());

  msg.pose.pose.position.x = p_lrf.x();
  msg.pose.pose.position.y = p_lrf.y();
  msg.pose.pose.position.z = p_lrf.z();

  // Orientation must be taken from NCom (NED - pseudo polar) and rotated into
  // ENU - tangent
  auto rpyBodENU = RosNComWrapper::getBodyRPY(nrx);
  auto rpyBodLRF = tf2::Quaternion();
  auto enu2lrf = tf2::Quaternion();
  // ENU to LRF rotation
  enu2lrf.setRPY(0, 0, lrf.heading());
  // transform from ENU to LRF
  rpyBodLRF = enu2lrf * rpyBodENU;

  msg.pose.pose.orientation.x = rpyBodLRF.x();
  msg.pose.pose.orientation.y = rpyBodLRF.y();
  msg.pose.pose.orientation.z = rpyBodLRF.z();
  msg.pose.pose.orientation.w = rpyBodLRF.w();

  // Covariance from NCom is in the NED local coordinate frame. This must be
  // rotated into the LRF

  // rotation from the ENU frame defined by the LRF origin to the full LRF frame
  tf2::Matrix3x3 r_enu_lrf = getRotEnuToLrf(lrf.heading());
  // rotation from ECEF frame to the ENU frame defined by the LRF origin
  tf2::Matrix3x3 r_ecef_enu = getRotEnuToEcef(lrf.lat(), lrf.lon()).transpose();
  // rotation from ECEF frame to the ENU frame defined by the current position
  tf2::Matrix3x3 r_pos_ecef = getRotEnuToEcef(nrx->mLat, nrx->mLon);

  auto diff = tf2::Matrix3x3(r_enu_lrf);
  diff *= r_ecef_enu;
  diff *= r_pos_ecef;

  auto tmp = tf2::Matrix3x3(diff);
  auto cov = tf2::Matrix3x3(std::pow(nrx->mEastAcc, 2), 0.0, 0.0,
                            0.0, std::pow(nrx->mNorthAcc, 2), 0.0,
                            0.0, 0.0, std::pow(nrx->mAltAcc, 2));
  // cov_b = R * cov_a * R^T
  tmp *= cov;
  tmp *= diff.transpose();

  // Copy the position covariance data into the output message
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      msg.pose.covariance[(6 * i) + j] = tmp[i][j];

  // twist =====================================================================

  auto twist_stamped = RosNComWrapper::velocity(nrx, head);
  msg.twist.twist = twist_stamped.twist;

  /** \todo Twist covariance */

  return msg;
}

nav_msgs::msg::Path
path(const std::vector<geometry_msgs::msg::PoseStamped> &poses,
     std_msgs::msg::Header head) {
  auto msg = nav_msgs::msg::Path();
  msg.header = std::move(head);
  msg.poses = poses;

  return msg;
}

sensor_msgs::msg::TimeReference time_reference(const NComRxC *nrx,
                                               std_msgs::msg::Header head) {
  auto msg = sensor_msgs::msg::TimeReference();

  msg.header = std::move(head);

  msg.time_ref.sec = static_cast<int32_t>(nrx->mTimeWeekSecond);
  msg.time_ref.nanosec = static_cast<uint32_t>(
      (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond)) *
      NAV_CONST::SECS2NANOSECS);
  msg.source = "ins";

  return msg;
}

} // namespace RosNComWrapper
