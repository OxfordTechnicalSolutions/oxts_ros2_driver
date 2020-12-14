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

//==============================================================================
// UDP Socket
//==============================================================================

class UdpSocket
{

};



//==============================================================================
// Oxts Device
//==============================================================================
class OxtsDevice
{
  int port = 3000;
  std::string ip = "192.168.25.34";

  NComRxC *nrx;
  NComPublisherNode ncomPublisherNode;

  // Sockets

  // member functions
  //OxtsDevice();
  //~OxtsDevice();

  int handle_ncom();        //!> Look for new NCom packet. Publish ROS messages if there is


};

#endif // OXTS_DEVICE_HPP