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

// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "nav/NComRxC.h"

// Other includes
#include "ncom_publisher_node.hpp"
#include "oxts_device.hpp"


using namespace std::chrono_literals;

class OxtsDevice
{
  int port = 3000;
  std::string ip = "192.168.25.34";
  // Socket itself


};


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
  
  // Open socket


  while (rclcpp::ok())
  {
    // Read socket for next packet

    // Read NCom from buffer (temp variables, replace when socket is implemented)
    unsigned char *temp = NULL;
    int packetLength = 72;
    NComNewChars(nrx, temp, packetLength);

    pub_node->ncom_callback(nrx); 
    // 

    rclcpp::spin_some(pub_node);
  }


  rclcpp::shutdown();
  return 0;
}
