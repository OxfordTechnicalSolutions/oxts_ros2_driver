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
#include "oxts_device.hpp"
#include "udp_server_client.h"

// gad-sdk includes
#include "nav/NComRxC.h"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

//==============================================================================
// Set up the UDP connection to the INS device

  std::string unitEndpointAddress = "192.168.25.34";
  short       unitEndpointPort    = 3000;

  OxtsDevice device(unitEndpointAddress);
  device.udpClient.set_local_port(unitEndpointPort);
  
  // Initialise publishers for all supported (and configured) messages
  // rclcpp::Publisher <msg>_publisher.advertise<sensor_msgs::NavSatFix>("gps/fix",2);
//==============================================================================

  RCLCPP_INFO(device.ncomPublisherNode.get_logger(), "Starting up node");
  while (rclcpp::ok())
  {
    device.handle_ncom();
  }

  rclcpp::shutdown();
  return 0;
}
