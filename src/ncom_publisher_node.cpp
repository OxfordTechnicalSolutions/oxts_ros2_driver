#include "ros-driver/ncom_publisher_node.hpp"


int NComPublisherNode::ncom_callback(const NComRxC* nrx)
{
  //////////////////////////////////////////////////////////////////////////////
  // Construct std_msgs/msg/String
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubStringFlag == 1)
  {
    auto msgString = RosNComWrapper::wrap_string (nrx);
    pubString_->publish(msgString);

    if ((this->count_ % 100) == 0)
      RCLCPP_INFO(this->get_logger(), "'%d' Publishing: '%s'", 
                                      this->count_, msgString.data.c_str());
  }

  //////////////////////////////////////////////////////////////////////////////
  // Construct nav_msgs/msg/Odometry
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubOdometryFlag == 1)
  {
    auto msgOdometry = RosNComWrapper::wrap_odometry (nrx);
    pubOdometry_->publish(msgOdometry);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/NavSatFix
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubNavSatFixFlag == 1)
  {
    auto msgNavSatFix = RosNComWrapper::wrap_nav_sat_fix(nrx);
    pubNavSatFix_->publish(msgNavSatFix);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/Imu
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubImuFlag == 1)
  {
    auto msgImu = RosNComWrapper::wrap_imu(nrx);
    pubImu_->publish(msgImu);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TwistStamped
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubVelocityFlag == 1)
  {
    auto msgVelocity = RosNComWrapper::wrap_velocity(nrx);
    pubVelocity_->publish(msgVelocity);
  }


  this->count_++;  
  return 0;
}