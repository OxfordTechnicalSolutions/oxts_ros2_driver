#include "ros-driver/ncom_publisher_node.hpp"


int NComPublisherNode::ncom_callback(const NComRxC* nrx)
{
  //////////////////////////////////////////////////////////////////////////////
  // Construct std_msgs/msg/String
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubStringRate && ((this->count_ % this->ncomPerStringPublished) == 0))
  {
    auto msgString = RosNComWrapper::wrap_string (nrx);
    pubString_->publish(msgString);

    RCLCPP_INFO(this->get_logger(), "'%d' Publishing: '%s'", 
                                    this->count_, msgString.data.c_str());
  }

  //////////////////////////////////////////////////////////////////////////////
  // Construct nav_msgs/msg/Odometry
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubOdometryRate && ((this->count_ % this->ncomPerOdometryPublished) == 0))
  {
    auto msgOdometry = RosNComWrapper::wrap_odometry (nrx);
    pubOdometry_->publish(msgOdometry);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/NavSatFix
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubNavSatFixRate && ((this->count_ % this->ncomPerNavSatFixPublished) == 0))
  {
    auto msgNavSatFix = RosNComWrapper::wrap_nav_sat_fix(nrx);
    pubNavSatFix_->publish(msgNavSatFix);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/Imu
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubImuRate && ((this->count_ % this->ncomPerImuPublished) == 0))
  {
    auto msgImu = RosNComWrapper::wrap_imu(nrx);
    pubImu_->publish(msgImu);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TwistStamped
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubVelocityRate && ((this->count_ % this->ncomPerVelocityPublished) == 0))
  {
    auto msgVelocity = RosNComWrapper::wrap_velocity(nrx);
    pubVelocity_->publish(msgVelocity);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TransformStamped
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubTf2Rate && ((this->count_ % this->ncomPerTf2Published) == 0))
  {
    auto msgTf2 = RosNComWrapper::wrap_tf2(nrx);
    pubTf2_->publish(msgTf2);
  }

  this->count_++;  
  return 0;
}


std::string NComPublisherNode::get_unit_ip()
{
  return this->unitIp;
}

short       NComPublisherNode::get_unit_port()
{
  return this->unitPort;
}