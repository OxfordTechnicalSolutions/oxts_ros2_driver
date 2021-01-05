/**
 * \file oxts_device.hpp 
 * OxtsDevice class for handling interactions with an OxTS INS.
 */

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
#include "nav/NComRxC.h"

// Other includes
#include "ros-driver/ncom_publisher_node.hpp"
#include "ros-driver/udp_server_client.h"
/**
* Oxts Device
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
   * \todo Change to std::shared_ptr<NComPublisherNode>
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
   * @param unitEndpointIp The IP address of the device to connect to.
   */
  OxtsDevice(std::string unitEndpointIp = "0.0.0.0")
  {
    nrx = NComCreateNComRxC();

    unitEndpointNCom = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(unitEndpointIp), 3000);

    this->udpClient.set_local_port(3000);

  }

  /**
   *  Look for new NCom packet. Decode, then publish ROS messages if there is
   */
  int HandleNCom();   


  void SetUnitEndpointNCom(std::string deviceIp, int port);

};

#endif // OXTS_DEVICE_HPP