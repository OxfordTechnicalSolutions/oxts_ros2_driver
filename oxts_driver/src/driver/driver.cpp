#include "oxts_driver/driver.hpp"

namespace oxts_driver
{

void OxtsDriver::timer_ncom_socket_callback()
{
  OxtsDriver::get_socket_packet();
  OxtsDriver::publish_packet();
}

void OxtsDriver::timer_ncom_file_callback()
{
  OxtsDriver::get_file_packet();
  OxtsDriver::publish_packet();
}

void OxtsDriver::get_file_packet()
{
  char c;

  while(NComNewChar(this->nrx, (unsigned char) c) != COM_NEW_UPDATE)
    if (!this->inFileNCom.get(c))
    {
      RCLCPP_INFO(this->get_logger(), "End of NCom file reached.");
      rclcpp::shutdown();
      return;
    };
}

void OxtsDriver::get_socket_packet()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, NCOM_PACKET_LENGTH, this->unitEndpointNCom);
  // Add data to decoder
  while (NComNewChars(this->nrx, this->buff, size) != COM_NEW_UPDATE) {};
}

void OxtsDriver::publish_packet()
{
  auto msg = oxts_msgs::msg::Ncom();
  // perform error checking on nrx timestamps
  if (this->prevWeekSecond > 0) {
    if (this->nrx->mTimeWeekSecond - this->prevWeekSecond > (1.5/this->ncom_rate))
      RCLCPP_WARN(this->get_logger(), "Packet drop detected.");
    if (this->nrx->mTimeWeekSecond < this->prevWeekSecond) {
      RCLCPP_ERROR(this->get_logger(), "Current packet is older than previous packet, skipping packet.");
      return;
    }
    if (this->nrx->mTimeWeekSecond == this->prevWeekSecond) {
      RCLCPP_ERROR(this->get_logger(), "Duplicate NCOM packet detected, skipping packet.");
      return;
    }
    if (this->nrx->mTimeWeekSecond - this->prevWeekSecond < (0.5/this->ncom_rate))
      RCLCPP_WARN(this->get_logger(), "Early packet detected, ncom_rate may be misconfigured.");
  }
  // publish the NCOM packet
  msg.header.stamp = this->get_timestamp();
  msg.header.frame_id = "oxts_sn" + std::to_string(this->nrx->mSerialNumber);
  for (int i=0; i < NCOM_PACKET_LENGTH ;++i)
    msg.raw_packet[i] = this->nrx->mInternal->mCurPkt[i];
  this->pubNCom_->publish(msg);
  this->prevWeekSecond = this->nrx->mTimeWeekSecond;
}

rclcpp::Time OxtsDriver::get_timestamp()
{
  if (this->timestamp_mode == PUB_TIMESTAMP_MODE::ROS)
    return this->get_clock()->now();
  else
    return this->ncomTime(this->nrx);
}

rclcpp::Time OxtsDriver::ncomTime(const NComRxC *nrx)
{
  auto time = rclcpp::Time(static_cast<int32_t>(nrx->mTimeWeekSecond) + 
                           (nrx->mTimeWeekCount * NAV_CONST::WEEK_SECS) + 
                           nrx->mTimeUtcOffset + NAV_CONST::GPS2UNIX_EPOCH,
  static_cast<uint32_t>((nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond))
    * NAV_CONST::SECS2NANOSECS ));

  return time;
}

std::string OxtsDriver::get_unit_ip()
{
  return this->unit_ip;
}

short       OxtsDriver::get_unit_port()
{
  return this->unit_port;
}

} // namespace oxts_driver