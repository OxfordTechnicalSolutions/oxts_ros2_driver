// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Other includes
#include "ncom_publisher_node.hpp"
//#include "oxts_device.hpp"
#include "udp_server_client.h"

// gad-sdk includes
#include "nav/NComRxC.h"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto pub_node = std::make_shared<NComPublisherNode>();

  // Initialise publishers for all supported (and configured) messages
  // rclcpp::Publisher <msg>_publisher.advertise<sensor_msgs::NavSatFix>("gps/fix",2);

  // Initialise NCom decoder
  NComRxC *nrx;
  nrx = NComCreateNComRxC();
  if (nrx == NULL)
    RCLCPP_ERROR(pub_node->get_logger(), "Failed to create NCom decoder");


//==============================================================================
// Create UDP client to receive NCom data from unit
  networking_udp::client local_client                   (3000); // receive to

  auto unit_endpoint = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string("192.168.25.34"), 5001); //received from

  unsigned char buff[1024];

//==============================================================================

  RCLCPP_INFO(pub_node->get_logger(), "Spinning up node");
  while (rclcpp::ok())
  {
    std::size_t size = local_client.receive_from(buff, 72, unit_endpoint); // retreive data from unit_endpoint on the port we specify in the constructor of local_client

    //NComNewChars(nrx, buff, size); // add data to decoder

    RCLCPP_ERROR(pub_node->get_logger(), "New Packet\n");
    RCLCPP_ERROR(pub_node->get_logger(), "%s", buff);

    // Take data from ncom and publish it in ROS messages
    //pub_node->ncom_callback(nrx); 

  }


  rclcpp::shutdown();
  return 0;
}
