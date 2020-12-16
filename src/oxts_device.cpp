#include "oxts_device.hpp"

int OxtsDevice::handle_ncom()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpoint);
  // Add data to decoder
  NComNewChars(nrx, buff, size); 
  // Run callback function to publish configured ROS messages
  this->ncomPublisherNode.ncom_callback(nrx);

  return 0;
}