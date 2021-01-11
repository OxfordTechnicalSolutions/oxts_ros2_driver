#include "ros-driver/ncom_publisher_node.hpp"


int NComPublisherNode::ncom_callback(const NComRxC* nrx)
{
  rclcpp::Time currentTime;

  // Get time for timestamps from the configured source
  if (this->timestampMode == PUB_TIMESTAMP_MODE::NCOM)
  {
    currentTime = RosNComWrapper::ncom_time_to_time(nrx);
  }
  else
  {
    currentTime = clock_.now();
  }

  //RCLCPP_INFO(this->get_logger(), "Count: %d, Seconds: %f, Nanoseconds: %u", this->count_, currentTime.seconds(), currentTime.nanoseconds());


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
    auto headerOdometry = RosNComWrapper::wrap_header(currentTime, "ins");
    auto msgOdometry = RosNComWrapper::wrap_odometry (nrx, headerOdometry);
    pubOdometry_->publish(msgOdometry);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/NavSatFix
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubNavSatFixRate && ((this->count_ % this->ncomPerNavSatFixPublished) == 0))
  {
    auto headerNavSatFix = RosNComWrapper::wrap_header(currentTime, "ins");
    auto msgNavSatFix = RosNComWrapper::wrap_nav_sat_fix(nrx, headerNavSatFix);
    pubNavSatFix_->publish(msgNavSatFix);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct sensor_msgs/msg/Imu
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubImuRate && ((this->count_ % this->ncomPerImuPublished) == 0))
  {
    auto headerImu = RosNComWrapper::wrap_header(currentTime, "ins");
    auto msgImu = RosNComWrapper::wrap_imu(nrx, headerImu);
    pubImu_->publish(msgImu);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TwistStamped
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubVelocityRate && ((this->count_ % this->ncomPerVelocityPublished) == 0))
  {
    auto headerVelocity = RosNComWrapper::wrap_header(currentTime, "ins");
    auto msgVelocity = RosNComWrapper::wrap_velocity(nrx,headerVelocity);
    pubVelocity_->publish(msgVelocity);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TimeReference
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubTimeReferenceRate && ((this->count_ % this->ncomPerTimeReferencePublished) == 0))
  {
    auto headerTimeReference = RosNComWrapper::wrap_header(currentTime, "ins");
    auto msgTimeReference = RosNComWrapper::wrap_time_reference(nrx, headerTimeReference);
    pubTimeReference_->publish(msgTimeReference);
  }
  //////////////////////////////////////////////////////////////////////////////
  // Construct geometry_msgs/msg/TransformStamped
  //////////////////////////////////////////////////////////////////////////////
  if (this->pubTf2Rate && ((this->count_ % this->ncomPerTf2Published) == 0))
  {
    auto headerTf2 = RosNComWrapper::wrap_header(currentTime, "earth");
    auto msgTf2 = RosNComWrapper::wrap_tf2(nrx, headerTf2);
    pubTf2_->publish(msgTf2);
  }


  // Reset count to 1 if we have crossed over 1s. Increment upTime.
  if ((this->count_ % this->ncomRate) == 0)
  {
    this->count_ = 0;
    this->upTime++;
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