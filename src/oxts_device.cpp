#include "ros-driver/oxts_device.hpp"

int OxtsDevice::HandleNCom()
{
  // Read from open socket
  std::size_t size = this->udpClient.receive_from(this->buff, 72, this->unitEndpointNCom);
  // Add data to decoder
  NComNewChars(nrx, buff, size); 
  // Run callback function to publish configured ROS messages
  this->ncomPublisherNode.ncom_callback(nrx);

  return 0;
}


void OxtsDevice::SetUnitEndpointNCom(std::string unitEndpointIp = "0.0.0.0", 
                                      int unitEndpointPort = 3000 )
{
  unitEndpointNCom = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(unitEndpointIp), unitEndpointPort);
}
