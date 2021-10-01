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

#include "oxts_driver/driver.hpp"

namespace oxts_driver {

void OxtsDriver::timerNcomSocketCallback() {
  OxtsDriver::getSocketPacket();
  OxtsDriver::publishPacket();
}

void OxtsDriver::timerNcomFileCallback() {
  OxtsDriver::getFilePacket();
  OxtsDriver::publishPacket();
}

void OxtsDriver::getFilePacket() {
  char c;

  do {
      if (!this->inFileNCom.get(c)) {
          RCLCPP_INFO(this->get_logger(), "End of NCom file reached.");
          rclcpp::shutdown();
          return;
      }
  } while (NComNewChar(this->nrx, (unsigned char)c) != COM_NEW_UPDATE);
}

void OxtsDriver::getSocketPacket() {
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(
      this->buff, NCOM_PACKET_LENGTH, this->unitEndpointNCom);
  // Add data to decoder
  while (NComNewChars(this->nrx, this->buff, size) != COM_NEW_UPDATE) {
  }
}

void OxtsDriver::publishPacket() {
  // publish the NCOM packet
  switch (this->nrx->mOutputPacketType) {
  case OUTPUT_PACKET_REGULAR: {
    if (this->checkRate(this->prevRegularWeekSecond,
                        this->nrx->mTimeWeekSecond))
      return;

    auto msg = oxts_msgs::msg::Ncom();
    msg.header.stamp = this->getTimestamp();
    msg.header.frame_id = "oxts_sn" + std::to_string(this->nrx->mSerialNumber);
    for (int i = 0; i < NCOM_PACKET_LENGTH; ++i)
      msg.raw_packet[i] = this->nrx->mInternal->mCurPkt[i];
    this->pubNCom_->publish(msg);
    this->prevRegularWeekSecond = this->nrx->mTimeWeekSecond;
    break;
  }
  case OUTPUT_PACKET_STATUS: {
    break;
  }
  default:
    break;
  }
}

bool OxtsDriver::checkRate(double prevPktSec, double currPktSec) {
  bool skip_packet = false;
  // perform error checking on nrx timestamps
  if (prevPktSec <= 0)
    ;
  else if (currPktSec - prevPktSec > (1.5 / this->ncom_rate)) {
    RCLCPP_WARN(this->get_logger(), "Packet drop detected.");
  } else if (currPktSec < prevPktSec) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Current packet is older than previous packet, skipping packet.");
    skip_packet = true;
  } else if (currPktSec == prevPktSec) {
    RCLCPP_ERROR(this->get_logger(),
                 "Duplicate NCOM packet detected, skipping packet.");
    skip_packet = true;
  } else if (currPktSec - prevPktSec < (0.5 / this->ncom_rate)) {
    RCLCPP_WARN(this->get_logger(),
                "Early packet detected, ncom_rate may be misconfigured.");
  }
  return skip_packet;
}

rclcpp::Time OxtsDriver::getTimestamp() {
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return this->get_clock()->now();
  else
    return this->getNcomTime(this->nrx);
}

rclcpp::Time OxtsDriver::getNcomTime(const NComRxC *nrx) {
  auto time =
      rclcpp::Time(static_cast<int32_t>(nrx->mTimeWeekSecond) +
                       (nrx->mTimeWeekCount * NAV_CONST::WEEK_SECS) +
                       nrx->mTimeUtcOffset + NAV_CONST::GPS2UNIX_EPOCH,
                   static_cast<uint32_t>((nrx->mTimeWeekSecond -
                                          std::floor(nrx->mTimeWeekSecond)) *
                                         NAV_CONST::SECS2NANOSECS));

  return time;
}

std::string OxtsDriver::getUnitIp() { return this->unit_ip; }

short OxtsDriver::getUnitPort() { return this->unit_port; }

} // namespace oxts_driver