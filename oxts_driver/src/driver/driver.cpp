#include "oxts_driver/driver.hpp"

namespace oxts_driver
{

void OxtsDriver::timer_ncom_socket_callback()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpointNCom);
  // Add data to decoder
  NComNewChars(this->nrx, this->buff, size);
  auto msg = oxts_msgs::msg::Ncom();
  msg.raw_packet = (char[72])*nrx;
  this->pubNCom_->publish(msg);
}

void OxtsDriver::timer_ncom_file_callback()
{
  char c;

  while(this->inFileNCom.get(c))
  {
    // Decode the data
    if(NComNewChar(this->nrx, (unsigned char) c) == COM_NEW_UPDATE)
    {
      break;
    }
  }
  auto msg = oxts_msgs::msg::Ncom();
  msg.raw_packet = (char[72])*nrx;
  this->pubNCom_->publish(msg);
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