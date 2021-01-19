/**
 * \file ncom_publisher_node.hpp
 * Defines node to take NCom data and publish it in ROS messages.
 */

#ifndef NCOM_PUBLISHER_NODE_HPP
#define NCOM_PUBLISHER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "nav/NComRxC.h"
#include "ros-driver/nav_const.hpp"
#include "ros-driver/ros_ncom_wrapper.hpp"
#include "ros-driver/udp_server_client.h"

using namespace std::chrono_literals;


/**
 * Enumeration of driver timestamp modes for published topics
 */
enum PUB_TIMESTAMP_MODE
{
  /**
   * Use time within the driver. ROS time.
   */
  ROS = 0,
  /**
   * Use NCom time.
   */
  NCOM = 1
};


/**
 * This class creates a subclass of Node designed to take NCom data from the 
 * NCom decoder and publish it to pre-configured ROS topics.
 * 
 * @todo Add config struct to hold data which will hold config parsed from the 
 *       .yaml file.
 * @todo Change topic names to a standard form /imu/.. 
 * @todo Refactor timestamping if statements out of callback functions and into
 *       node initialisation.
 */
class NComPublisherNode : public rclcpp::Node
{
private:
  /*! Rate at which to sample NCom. Expected that this will typically match
    the rate of NCom itself, though can be set lower to save computation. */
  int ncom_rate;                  
  /*! IP address of the INS to connect to */
  std::string unit_ip;            
  /*! Endpoint Port of the INS to be connected to. Default 3000 for NCom. */
  int unit_port;                  
  /*! File path to NCom file to be used as input. Not required if running
    in real time. */
  std::string ncom_path;               
  /*! Timestamp type to be applied to published packets
    {0 : Driver time, 1 : NCom time} */
  int timestamp_mode;             
  /*! Frame ID of outgoing packets. @todo Having a general frame ID may not
    make sense. This isn't implemented. */
  std::string frame_id;           
  /*! Publishing rate for debug String message. */
  double pub_string_rate;         
  /*! Publishing rate for Odometry message. */
  double pub_odometry_rate;       
  /*! Publishing rate for NavSatFix message. */
  double pub_nav_sat_fix_rate;    
  /*! Publishing rate for Imu message. */
  double pub_imu_rate;            
  /*! Publishing rate for Velocity message. */
  double pub_velocity_rate;       
  /*! Publishing rate for TimeReference message.*/
  double pub_time_reference_rate; 
  /*! Publishing rate for Tf2 message. */
  double pub_tf2_rate;            

  std::chrono::duration<uint64_t,std::milli> ncomInterval;
  std::chrono::duration<uint64_t,std::milli> pubStringInterval;
  std::chrono::duration<uint64_t,std::milli> pubOdometryInterval;
  std::chrono::duration<uint64_t,std::milli> pubNavSatFixInterval;
  std::chrono::duration<uint64_t,std::milli> pubImuInterval;
  std::chrono::duration<uint64_t,std::milli> pubVelocityInterval;
  std::chrono::duration<uint64_t,std::milli> pubTimeReferenceInterval;
  std::chrono::duration<uint64_t,std::milli> pubTf2Interval;
  // ...

  rclcpp::TimerBase::SharedPtr timer_ncom_;
  rclcpp::TimerBase::SharedPtr timer_string_;
  rclcpp::TimerBase::SharedPtr timer_odometry_;
  rclcpp::TimerBase::SharedPtr timer_nav_sat_fix_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::TimerBase::SharedPtr timer_velocity_;
  rclcpp::TimerBase::SharedPtr timer_time_reference_;
  rclcpp::TimerBase::SharedPtr timer_tf2_;

  /**
   * Callback function for NCom sampling. Receives data from chosen source
   * (UDP or file) and parses a packet to nrx.
   * 
   * @todo Implement file parsing.
   */
  void timer_ncom_callback();
  void timer_ncom_file_callback();
  /** 
   * Callback function for debug String message. Wraps message, publishes, and
   * prints some information to the console.
   */
  void timer_string_callback();
  /** 
   * Callback function for Odometry message. Wraps message and publishes.
   */
  void timer_odometry_callback();
  /** 
   * Callback function for NavSatFix message. Wraps message and publishes.
   */
  void timer_nav_sat_fix_callback();
  /** 
   * Callback function for Imu message. Wraps message and publishes.
   */
  void timer_imu_callback();
  /** 
   * Callback function for TimeReference message. Wraps message and publishes.
   */
  void timer_time_reference_callback();
  /** 
   * Callback function for Velocity message. Wraps message and publishes.
   */
  void timer_velocity_callback();
  /** 
   * Callback function for Tf2 message. Wraps message and publishes.
   */
  void timer_tf2_callback();


  /**
   * Publisher for std_msgs/msg/string. Only used for debugging, currently 
   * outputs lat, long, alt in string form.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       pubString_;
  /**
   * Publisher for /sensor_msgs/msg/Odometry
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     pubOdometry_;
  /**
   * Publisher for /sensor_msgs/msg/NavSatFix
   */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNavSatFix_;
  /**
   * Publisher for /sensor_msgs/msg/Imu
   */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       pubImu_;
  /**
   * Publisher for /sensor_msgs/msg/TwistStamped
   */
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  pubVelocity_;
  /**
   * Publisher for /sensor_msgs/msg/TimeReference
   */
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr  pubTimeReference_;
  /**
   * Publisher for /geometry_msgs/msg/TransformStamped
   */
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr  pubTf2_;
  /**
   * Node clock.
   */ 
  rclcpp::Clock clock_;

public:
  /**
   * Default constructor for the NComPublisherNode. Parses options from the 
   * .yaml params/config file, sets up UDP connection to unit.
   */
  explicit NComPublisherNode(const rclcpp::NodeOptions & options) : Node("ncom_publisher", options)
  {
    // Get parameters (from config, command line, or from default)
    // Initialise configurable parameters (all params should have defaults)
    ncom_rate               = this->declare_parameter("ncom_rate", 100.0);                        
    unit_ip                 = this->declare_parameter("unit_ip", "0.0.0.0");                      
    unit_port               = this->declare_parameter("unit_port", 3000);                         
    ncom_path               = this->declare_parameter("ncom", std::string(""));                   
    timestamp_mode          = this->declare_parameter("timestamp_mode", 0); 
    frame_id                = this->declare_parameter("frame_id", "base_link");                   
    pub_string_rate         = this->declare_parameter("pub_string_rate", 1.0);                    
    pub_odometry_rate       = this->declare_parameter("pub_odometry_rate", 1.0);                  
    pub_nav_sat_fix_rate    = this->declare_parameter("pub_nav_sat_fix_rate", 1.0);               
    pub_imu_rate            = this->declare_parameter("pub_imu_rate", 1.0);                       
    pub_velocity_rate       = this->declare_parameter("pub_velocity_rate", 1.0);                  
    pub_time_reference_rate = this->declare_parameter("pub_time_reference_rate", 1.0);            
    pub_tf2_rate            = this->declare_parameter("pub_tf2_rate", 1.0);                       

    ncomInterval             = std::chrono::milliseconds(int(1000.0 / ncom_rate));
    pubStringInterval        = std::chrono::milliseconds(int(1000.0 / pub_string_rate));
    pubOdometryInterval      = std::chrono::milliseconds(int(1000.0 / pub_odometry_rate));
    pubNavSatFixInterval     = std::chrono::milliseconds(int(1000.0 / pub_nav_sat_fix_rate));
    pubImuInterval           = std::chrono::milliseconds(int(1000.0 / pub_imu_rate));
    pubVelocityInterval      = std::chrono::milliseconds(int(1000.0 / pub_velocity_rate));
    pubTimeReferenceInterval = std::chrono::milliseconds(int(1000.0 / pub_time_reference_rate));
    pubTf2Interval           = std::chrono::milliseconds(int(1000.0 / pub_tf2_rate));

 
    // Initialise publishers for each message - all are initialised, even if not
    // configured
    pubString_        = this->create_publisher<std_msgs::msg::String>               ("ins/debug_string_pos", 10); 
    pubOdometry_      = this->create_publisher<nav_msgs::msg::Odometry>             ("ins/odom",             10); 
    pubNavSatFix_     = this->create_publisher<sensor_msgs::msg::NavSatFix>         ("ins/nav_sat_fix",      10); 
    pubImu_           = this->create_publisher<sensor_msgs::msg::Imu>               ("imu/data",             10); 
    pubVelocity_      = this->create_publisher<geometry_msgs::msg::TwistStamped>    ("ins/velocity",         10); 
    pubTimeReference_ = this->create_publisher<sensor_msgs::msg::TimeReference>     ("ins/time_reference",   10);
    pubTf2_           = this->create_publisher<geometry_msgs::msg::TransformStamped>("ins/tf2",              10); 

    clock_ = rclcpp::Clock(RCL_ROS_TIME); /*! @todo Add option for RCL_SYSTEM_TIME */


    //this->get_parameter("use_sim_time");

    // Assign callback functions to timers (callbacks are called at a rate
    // dictated by the associated timer)
    if (!ncom_path.empty())
    {
      timer_ncom_ = this->create_wall_timer(
                   ncomInterval, std::bind(&NComPublisherNode::timer_ncom_file_callback, this));
    }
    else 
    {
      timer_ncom_ = this->create_wall_timer(
                    ncomInterval, std::bind(&NComPublisherNode::timer_ncom_callback, this));
    }
    timer_string_ = this->create_wall_timer(
                  pubStringInterval, std::bind(&NComPublisherNode::timer_string_callback, this));
    timer_odometry_ = this->create_wall_timer(
                  pubOdometryInterval, std::bind(&NComPublisherNode::timer_odometry_callback, this));
    timer_nav_sat_fix_ = this->create_wall_timer(
                  pubNavSatFixInterval, std::bind(&NComPublisherNode::timer_nav_sat_fix_callback, this));
    timer_imu_ = this->create_wall_timer(
                  pubImuInterval, std::bind(&NComPublisherNode::timer_imu_callback, this));
    timer_velocity_ = this->create_wall_timer(
                  pubVelocityInterval, std::bind(&NComPublisherNode::timer_velocity_callback, this));
    timer_time_reference_ = this->create_wall_timer(
                  pubTimeReferenceInterval, std::bind(&NComPublisherNode::timer_time_reference_callback, this));
    timer_tf2_ = this->create_wall_timer(
                  pubTf2Interval, std::bind(&NComPublisherNode::timer_tf2_callback, this));

    nrx = NComCreateNComRxC();

    if (!ncom_path.empty())
    {
      inFileNCom.open(ncom_path);
      if(!inFileNCom.is_open())
      {
        // error  
        RCLCPP_ERROR(this->get_logger(), "Unable to open NCOM: %s", ncom_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Opened NCOM: %s", ncom_path.c_str());
      }
    }
    else 
    {
      unitEndpointNCom = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(this->unit_ip), this->unit_port);

      this->udpClient.set_local_port(this->unit_port);
      RCLCPP_INFO(this->get_logger(), "Connecting: %s:%d", this->unit_ip.c_str(), this->unit_port);
    }

  }

  /**
   * NCom decoder instance
   */
  NComRxC *nrx;
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

  std::fstream inFileNCom;

  /**
   * Get the IP address of the OxTS unit, as set in the .yaml params file
   * 
   * @returns IP address as a string
   */
  std::string get_unit_ip();
  /**
   * Get the endpoint port of the OxTS unit, as set in the .yaml params file
   * 
   * @returns Port as a short
   */
  short get_unit_port();

};






#endif //NCOM_PUBLISHER_NODE_HPP