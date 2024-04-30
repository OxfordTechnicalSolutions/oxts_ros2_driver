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

#include "oxts_ins/convert.hpp"

namespace oxts_ins {

void OxtsIns::ncomCallbackRegular(const oxts_msgs::msg::Ncom::SharedPtr msg) {
  // Add data to decoder
  if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) ==
      COM_NEW_UPDATE) {
    double current_time = rclcpp::Time(msg->header.stamp).seconds();
    int sec_idx = round((current_time - floor(current_time)) * this->ncom_rate);

    // Publish IMU message if being subscribed to and enabled in config
    if (this->pub_imu_flag)
      this->imu(msg->header);
    if (this->pub_tf_flag) {
      this->tf_world_to_base_link(msg->header);
    }
    if (this->pubStringInterval && (sec_idx % this->pubStringInterval == 0))
      this->string();
    if (this->pubNavSatRefInterval &&
        (sec_idx % this->pubNavSatRefInterval == 0))
      this->nav_sat_ref(msg->header);
    if (this->pubEcefPosInterval && (sec_idx % this->pubEcefPosInterval == 0))
      this->ecef_pos(msg->header);
    if (this->pubNavSatFixInterval &&
        (sec_idx % this->pubNavSatFixInterval == 0))
      this->nav_sat_fix(msg->header);
    if (this->pubVelocityInterval && (sec_idx % this->pubVelocityInterval == 0))
      this->velocity(msg->header);
    if (this->pubOdometryInterval && (sec_idx % this->pubOdometryInterval == 0))
      this->odometry(msg->header);
    if (this->pubPathInterval && (sec_idx % this->pubPathInterval == 0))
      this->path(msg->header);
    if (this->pubTimeReferenceInterval &&
        (sec_idx % this->pubTimeReferenceInterval == 0))
      this->time_reference(msg->header);
    if (this->pubLeverArmInterval && (sec_idx % this->pubLeverArmInterval == 0))
      this->lever_arm_gap(msg->header);
    if (this->pubIMUBiasInterval && (sec_idx % this->pubIMUBiasInterval == 0))
      this->imu_bias(msg->header);
  }
}

void OxtsIns::string() {
  auto msgString = RosNComWrapper::string(this->nrx);
  pubString_->publish(msgString);

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msgString.data.c_str());
}

void OxtsIns::nav_sat_fix(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::nav_sat_fix(this->nrx, header);
  pubNavSatFix_->publish(msg);
}

void OxtsIns::nav_sat_ref(std_msgs::msg::Header header) {
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  if (this->lrf_valid) {
    header.frame_id = this->frame_id;
    auto msg = RosNComWrapper::nav_sat_ref(this->lrf, header);
    pubNavSatRef_->publish(msg);
  }
}

void OxtsIns::lever_arm_gap(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::lever_arm_gap(this->nrx, header);
  pubLeverArm_->publish(msg);
}

void OxtsIns::imu_bias(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::imu_bias(this->nrx, header);
  pubIMUBias_->publish(msg);
}

void OxtsIns::ecef_pos(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::ecef_pos(this->nrx, header);
  pubEcefPos_->publish(msg);
}

void OxtsIns::imu(std_msgs::msg::Header header) {
  header.frame_id = "imu_link";
  auto msg = RosNComWrapper::imu(this->nrx, header);
  pubImu_->publish(msg);
}

void OxtsIns::tf_world_to_base_link(const std_msgs::msg::Header &header) {
  this->getLrf();
  if (this->lrf_valid) {
    auto odometry =
        RosNComWrapper::odometry(this->nrx, header, this->lrf, this->frame_id);
    tf2::Transform tf_world_to_oxts;
    tf_world_to_oxts.setOrigin(tf2::Vector3(odometry.pose.pose.position.x,
                                            odometry.pose.pose.position.y,
                                            odometry.pose.pose.position.z));
    tf_world_to_oxts.setRotation(tf2::Quaternion(
        odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));
    geometry_msgs::msg::TransformStamped tf_oxts_to_base_link_msg;
    try {
      tf_oxts_to_base_link_msg = this->tf_buffer->lookupTransform(
          this->frame_id, this->base_link_frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Could not lookup transform from %s to %s: %s",
          this->frame_id.c_str(), this->base_link_frame_id.c_str(), ex.what());
      return;
    }

    tf2::Transform transfrom_oxts_to_base_link;
    transfrom_oxts_to_base_link.setOrigin(
        tf2::Vector3(tf_oxts_to_base_link_msg.transform.translation.x,
                     tf_oxts_to_base_link_msg.transform.translation.y,
                     tf_oxts_to_base_link_msg.transform.translation.z));
    transfrom_oxts_to_base_link.setRotation(
        tf2::Quaternion(tf_oxts_to_base_link_msg.transform.rotation.x,
                        tf_oxts_to_base_link_msg.transform.rotation.y,
                        tf_oxts_to_base_link_msg.transform.rotation.z,
                        tf_oxts_to_base_link_msg.transform.rotation.w));

    auto tf_world_to_base_link = tf_world_to_oxts * transfrom_oxts_to_base_link;

    geometry_msgs::msg::TransformStamped tf_world_to_base_link_msg;
    tf_world_to_base_link_msg.header = header;
    tf_world_to_base_link_msg.header.frame_id = this->pub_odometry_frame_id;
    tf_world_to_base_link_msg.child_frame_id = this->base_link_frame_id;
    tf_world_to_base_link_msg.transform.translation.x =
        tf_world_to_base_link.getOrigin().getX();
    tf_world_to_base_link_msg.transform.translation.y =
        tf_world_to_base_link.getOrigin().getY();
    tf_world_to_base_link_msg.transform.translation.z = 0.0;
    // tf_world_to_base_link.getOrigin().getZ() < 0 ? 0 : tf_world_to_base_link.getOrigin().getZ();
    tf_world_to_base_link_msg.transform.rotation.x =
        tf_world_to_base_link.getRotation().getX();
    tf_world_to_base_link_msg.transform.rotation.y =
        tf_world_to_base_link.getRotation().getY();
    tf_world_to_base_link_msg.transform.rotation.z =
        tf_world_to_base_link.getRotation().getZ();
    tf_world_to_base_link_msg.transform.rotation.w =
        tf_world_to_base_link.getRotation().getW();

    tf_broadcaster_->sendTransform(tf_world_to_base_link_msg);
  }
}

void OxtsIns::tf(const std_msgs::msg::Header &header) {
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  if (this->lrf_valid) {

    auto vat = RosNComWrapper::getVat(this->nrx);
    auto nsp = RosNComWrapper::getNsp(this->nrx);

    if (this->nrx->mIsNoSlipLeverArmXValid) {
      geometry_msgs::msg::TransformStamped tf_vat;
      tf_vat.header = header;
      tf_vat.header.frame_id = this->frame_id;
      tf_vat.child_frame_id = "rear_axle_link";
      tf_vat.transform.translation.x = nsp.x();
      tf_vat.transform.translation.y = nsp.y();
      tf_vat.transform.translation.z = nsp.z();
      tf_vat.transform.rotation.x = vat.x();
      tf_vat.transform.rotation.y = vat.y();
      tf_vat.transform.rotation.z = vat.z();
      tf_vat.transform.rotation.w = vat.w();
      tf_broadcaster_->sendTransform(tf_vat);

      if (true) // if vertical slip lever arm is valid
      {
        // auto nvsp    = RosNComWrapper::getNvsp(this->nrx);
        /** \todo Make this real */
        // The transform to create the front axle pose is spoofed with a
        // hardcoded offset, for now.
        auto nvsp = nsp;
        nvsp += tf2::quatRotate(vat, tf2::Vector3(2.6, 0, 0));

        geometry_msgs::msg::TransformStamped tf_front_axle;
        tf_front_axle.header = header;
        tf_front_axle.header.frame_id = this->frame_id;
        tf_front_axle.child_frame_id = "front_axle_link";
        tf_front_axle.transform.translation.x = nvsp.x();
        tf_front_axle.transform.translation.y = nvsp.y();
        tf_front_axle.transform.translation.z = nvsp.z();
        tf_front_axle.transform.rotation.x = vat.x();
        tf_front_axle.transform.rotation.y = vat.y();
        tf_front_axle.transform.rotation.z = vat.z();
        tf_front_axle.transform.rotation.w = vat.w();
        tf_broadcaster_->sendTransform(tf_front_axle);
      }
    }
  }
}

void OxtsIns::velocity(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::velocity(this->nrx, header);
  pubVelocity_->publish(msg);
}

void OxtsIns::odometry(std_msgs::msg::Header header) {
  header.frame_id = this->pub_odometry_frame_id;
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  if (this->lrf_valid) {
    auto msg =
        RosNComWrapper::odometry(this->nrx, header, this->lrf, this->frame_id);
    if (this->pubPathInterval) {
      auto new_pose_stamped = geometry_msgs::msg::PoseStamped();
      new_pose_stamped.header = msg.header;
      new_pose_stamped.pose = msg.pose.pose;
      this->past_poses.push_back(new_pose_stamped);
    }
    pubOdometry_->publish(msg);
  }
}

void OxtsIns::path(std_msgs::msg::Header header) {
  header.frame_id = this->pub_odometry_frame_id;
  auto msg = RosNComWrapper::path(this->past_poses, header);
  pubPath_->publish(msg);
}

void OxtsIns::time_reference(std_msgs::msg::Header header) {
  header.frame_id = this->frame_id;
  auto msg = RosNComWrapper::time_reference(this->nrx, header);
  pubTimeReference_->publish(msg);
}

void OxtsIns::getLrf() {
  // Configured to come from NCom LRF, and the NCom LRF is valid.
  if (this->lrf_source == LRF_SOURCE::NCOM_LRF && nrx->mIsRefLatValid) {
    this->lrf = RosNComWrapper::getNcomLrf(nrx);
    this->lrf_valid = true;
  }
  // Configured to come from the first NCom packet
  else if (!this->lrf_valid && this->lrf_source == LRF_SOURCE::NCOM_FIRST) {
    this->lrf.origin(nrx->mLat, nrx->mLon, nrx->mAlt);
    // mHeading is in NED. Get angle between LRF and ENU
    this->lrf.heading((nrx->mHeading - 90) * NAV_CONST::DEG2RADS);
    this->lrf_valid = true;
  } else if (!this->lrf_valid &&
             this->lrf_source == LRF_SOURCE::NCOM_FIRST_ENU) {
    this->lrf.origin(nrx->mLat, nrx->mLon, nrx->mAlt);
    this->lrf.heading((0.0) * NAV_CONST::DEG2RADS); // LRF aligned to ENU
    this->lrf_valid = true;
  }
}

} // namespace oxts_ins
