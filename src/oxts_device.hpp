#ifndef OXTS_DEVICE_HPP
#define OXTS_DEVICE_HPP

#include <string>
#include <iostream>

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

// Include NCom Decoder
#include "NComRxC.h"

// Other includes
#include "ncom_publisher_node.hpp"
#include "udp_server_client.h"
/**
 *Oxts Device
 */
class OxtsDevice
{
private:
  // Config

public:

  /**
   * NCom decoder class
   */
  NComRxC *nrx;
  /**
   * ROS2 Publisher Node for NCom data
   */
  NComPublisherNode ncomPublisherNode;

  // Sockets
  unsigned char buff[1024];
  networking_udp::client udpClient;
  boost::asio::ip::udp::endpoint unitEndpointNCom;

  /**
   * Constructor
   */
  OxtsDevice(std::string deviceIp = "0.0.0.0")
  {
    nrx = NComCreateNComRxC();

    unitEndpointNCom = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(deviceIp), 3000);

  }

  //~OxtsDevice();


  /**
   *  Look for new NCom packet. Decode, then publish ROS messages if there is
   */
  int handle_ncom();   


};

#endif // OXTS_DEVICE_HPP