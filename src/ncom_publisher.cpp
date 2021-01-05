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
#include <fstream> // Will be used for NCom streaming from file (or recording to bag)

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Other includes
#include "ros-driver/ncom_publisher_node.hpp"
#include "ros-driver/oxts_device.hpp"
#include "ros-driver/udp_server_client.h"

// gad-sdk includes
#include "nav/NComRxC.h"
#include <map>

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

//==============================================================================  
  // Test
  
  //const std::map<std::string, uint8_t,uint8_t> GpsPosModeMap;

 // GpsPosModeMap["None"] = 0;


//==============================================================================
// Set up the UDP connection to the INS device

  OxtsDevice device;
  
  device.SetUnitEndpointNCom(device.ncomPublisherNode.unitIp, 
                             device.ncomPublisherNode.unitPort );

//==============================================================================
  //! @todo Add try/catch 
  RCLCPP_INFO(device.ncomPublisherNode.get_logger(), "Starting up node");
  while (rclcpp::ok())
  {
    device.HandleNCom();
  }

  rclcpp::shutdown();
  return 0;
}
