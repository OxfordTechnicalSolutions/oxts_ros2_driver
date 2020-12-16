#ifndef OXTS_DEVICE_HPP
#define OXTS_DEVICE_HPP

#include <string>
#include <iostream>
#include <memory>

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
   * Output rate of NCom on the device. Typically 100Hz
   */
  int ncomOutputRate;
  /**
   * NCom decoder instance
   */
  NComRxC *nrx;
  /**
   * ROS2 Publisher Node for NCom data
   * 
   * @TODO Change to std::shared_ptr<NComPublisherNode>
   */
  NComPublisherNode ncomPublisherNode;
  /**
   * Buffer for UDP data
   */
  unsigned char buff[1024];
  /**
   * UDP Client to receive data from the device. 
   */
  networking_udp::client udpClient;
  /**
   * Endpoint for the udpClient to receive data from 
   */
  boost::asio::ip::udp::endpoint unitEndpointNCom;

  /**
   * Constructor
   * 
   * @param deviceIp The IP address of the device to connect to.
   */
  OxtsDevice(std::string deviceIp = "0.0.0.0")
  {
    ncomOutputRate = 100;
    nrx = NComCreateNComRxC();

    unitEndpointNCom = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(deviceIp), 3000);

  }

  /**
   *  Look for new NCom packet. Decode, then publish ROS messages if there is
   */
  int handle_ncom();   


};

#endif // OXTS_DEVICE_HPP