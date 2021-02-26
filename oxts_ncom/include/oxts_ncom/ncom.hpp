



/**
 * \file ncom.hpp
 * Defines node to take NCom data and publish it in ROS messages.
 */

#ifndef OXTS_NCOM__NCOM_HPP_
#define OXTS_NCOM__NCOM_HPP_

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
#include <geometry_msgs/msg/pose_with_covariance.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point.h>
#include <oxts_msgs/msg/ncom.hpp>
// Boost includes
#include <boost/asio.hpp>

// gad-sdk includes
#include "oxts_ncom/NComRxC.h"
#include "oxts_ncom/nav_const.hpp"
#include "oxts_ncom/wrapper.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace oxts_ncom
{

/**
 * Enumeration of timestamp modes for published topics
 */
enum PUB_TIMESTAMP_MODE
{
  /**
   * Use ROS time.
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
class OxtsNCom : public rclcpp::Node
{
private:
  /*! Rate at which to sample NCom. Expected that this will typically match
    the rate of NCom itself, though can be set lower to save computation. */
  int ncom_rate;
  /*! Timestamp type to be applied to published packets
    {0 : ROS time, 1 : NCom time} */
  int timestamp_mode;
  /*! Frame ID of outgoing packets. @todo Having a general frame ID may not
    make sense. This isn't implemented. */
  std::string frame_id;
  /*! Publishing rate for debug String message. */
  double pub_string_rate;
  /*! Publishing rate for NavSatFix message. */
  double pub_nav_sat_fix_rate;
  /*! Publishing rate for Imu message. */
  double pub_imu_rate;
  /*! Publishing rate for Velocity message. */
  double pub_velocity_rate;
  /*! Publishing rate for TimeReference message.*/
  double pub_time_reference_rate; 
  /*! Publishing rate for PointStamped message. */
  double pub_ecef_pos_rate;
  /*! Publishing rate for PointStamped message. */
  double pub_nav_sat_ref_rate;
  /*! Publish Tf messages. */
  bool pub_tf_flag;

  std::chrono::duration<uint64_t,std::milli> ncomInterval;
  std::chrono::duration<uint64_t,std::milli> pubStringInterval;
  std::chrono::duration<uint64_t,std::milli> pubNavSatFixInterval;
  std::chrono::duration<uint64_t,std::milli> pubImuInterval;
  std::chrono::duration<uint64_t,std::milli> pubTfInterval;
  std::chrono::duration<uint64_t,std::milli> pubVelocityInterval;
  std::chrono::duration<uint64_t,std::milli> pubTimeReferenceInterval;
  std::chrono::duration<uint64_t,std::milli> pubEcefPosInterval;
  std::chrono::duration<uint64_t,std::milli> pubNavSatRefInterval;
  // ...

  rclcpp::TimerBase::SharedPtr timer_ncom_;
  rclcpp::TimerBase::SharedPtr timer_string_;
  rclcpp::TimerBase::SharedPtr timer_nav_sat_fix_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::TimerBase::SharedPtr timer_tf_;
  rclcpp::TimerBase::SharedPtr timer_velocity_;
  rclcpp::TimerBase::SharedPtr timer_time_reference_;
  rclcpp::TimerBase::SharedPtr timer_ecef_pos_;
  rclcpp::TimerBase::SharedPtr timer_nav_sat_ref_;

  void NCom_callback(const oxts_msgs::msg::Ncom::SharedPtr msg) const;

  /**
   * Callback function for NCom sampling. Receives data from chosen source
   * (UDP or file) and parses a packet to nrx.
   * 
   * @todo Refactor into input class
   */
  void timer_ncom_socket_callback();
  void timer_ncom_file_callback();
  /** 
   * Callback function for debug String message. Wraps message, publishes, and
   * prints some information to the console.
   */
  void timer_string_callback();
  /** 
   * Callback function for NavSatFix message. Wraps message and publishes.
   */
  void timer_nav_sat_fix_callback();
  /** 
   * Callback function for Imu message. Wraps message and publishes.
   */
  void timer_imu_callback();
  /** 
   * Callback function for Tf messages. Wraps messages and broadcasts.
   */
  void timer_tf_callback();
  /** 
   * Callback function for TimeReference message. Wraps message and publishes.
   */
  void timer_time_reference_callback();
  /** 
   * Callback function for Velocity message. Wraps message and publishes.
   */
  void timer_velocity_callback();
  /** 
   * Callback function for PointStamped message. Wraps message and 
   * publishes.
   */
  void timer_ecef_pos_callback();
  /** 
   * Callback function for OxTS NavSatRef message. Wraps message and 
   * publishes.
   */
  void timer_nav_sat_ref_callback();


  /** 
   * Subscriber for oxts_msgs/msg/NCom.
   */
  rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr     subNCom_;
  /**
   * Publisher for std_msgs/msg/string. Only used for debugging, currently 
   * outputs lat, long, alt in string form.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       pubString_;
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
   * Publisher for /geometry_msgs/msg/PointStamped
   */
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pubEcefPos_;
  /**
   * Publisher for /oxts_msgs/msg/NavSatRef
   */
  rclcpp::Publisher<oxts_msgs::msg::NavSatRef>::SharedPtr  pubNavSatRef_;
  /**
   * Node clock.
   */ 
  rclcpp::Clock clock_;
  /**
   * TF broadcaster
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  /**
   * Default constructor for the OxtsNCom. Parses options from the 
   * .yaml params/config file, sets up UDP connection to unit.
   */
  explicit OxtsNCom(const rclcpp::NodeOptions & options) : Node("oxts_ncom", options)
  {
    // Initilize tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Get parameters (from config, command line, or from default)
    // Initialise configurable parameters (all params should have defaults)
    ncom_rate               = this->declare_parameter("ncom_rate", 100.0);
    timestamp_mode          = this->declare_parameter("timestamp_mode", 0); 
    frame_id                = this->declare_parameter("frame_id", "oxts_link");
    pub_string_rate         = this->declare_parameter("pub_string_rate", 1.0);
    pub_nav_sat_fix_rate    = this->declare_parameter("pub_nav_sat_fix_rate", 1.0);
    pub_imu_rate            = this->declare_parameter("pub_imu_rate", 1.0);
    pub_velocity_rate       = this->declare_parameter("pub_velocity_rate", 1.0);
    pub_time_reference_rate = this->declare_parameter("pub_time_reference_rate", 1.0);
    pub_ecef_pos_rate       = this->declare_parameter("pub_ecef_pos_rate", 1.0);
    pub_nav_sat_ref_rate    = this->declare_parameter("pub_nav_sat_ref_rate", 1.0);
    pub_tf_flag             = this->declare_parameter("pub_tf_flag", 1);

    pubStringInterval        = std::chrono::milliseconds(int(1000.0 / pub_string_rate));
    pubNavSatFixInterval     = std::chrono::milliseconds(int(1000.0 / pub_nav_sat_fix_rate));
    pubImuInterval           = std::chrono::milliseconds(int(1000.0 / pub_imu_rate));
    pubVelocityInterval      = std::chrono::milliseconds(int(1000.0 / pub_velocity_rate));
    pubTimeReferenceInterval = std::chrono::milliseconds(int(1000.0 / pub_time_reference_rate));
    pubEcefPosInterval       = std::chrono::milliseconds(int(1000.0 / pub_ecef_pos_rate));
    pubNavSatRefInterval     = std::chrono::milliseconds(int(1000.0 / pub_nav_sat_ref_rate));

    // Initialise subscriber for ncom message
    subNCom_ = this->create_subscription<oxts_msgs::msg::Ncom>
                      ("ncom",10,std::bind(&OxtsNCom::NCom_callback,this,_1));
    // Initialise publishers for each message - all are initialised, even if not
    // configured
    pubString_        = this->create_publisher<std_msgs::msg::String>                      
                                                   ("ins/debug_string_pos", 10); 
    pubNavSatFix_     = this->create_publisher<sensor_msgs::msg::NavSatFix>                
                                                   ("ins/nav_sat_fix",      10); 
    pubImu_           = this->create_publisher<sensor_msgs::msg::Imu>                      
                                                   ("imu/data",             10); 
    pubVelocity_      = this->create_publisher<geometry_msgs::msg::TwistStamped>           
                                                   ("ins/velocity",         10); 
    pubTimeReference_ = this->create_publisher<sensor_msgs::msg::TimeReference>            
                                                   ("ins/time_reference",   10);
    pubEcefPos_       = this->create_publisher<geometry_msgs::msg::PointStamped>
                                                   ("ins/ecef_pos",         10);
    pubNavSatRef_     = this->create_publisher<oxts_msgs::msg::NavSatRef>
                                                   ("ins/nav_sat_ref",      10);

    // clock_ = rclcpp::Clock(RCL_ROS_TIME); /*! @todo Add option for RCL_SYSTEM_TIME */

    // timer_string_ = this->create_wall_timer(
    //               pubStringInterval, std::bind(&OxtsNCom::timer_string_callback, this));
    // timer_nav_sat_fix_ = this->create_wall_timer(
    //               pubNavSatFixInterval, std::bind(&OxtsNCom::timer_nav_sat_fix_callback, this));
    // timer_imu_    = this->create_wall_timer(
    //               pubImuInterval, std::bind(&OxtsNCom::timer_imu_callback, this));
    // timer_velocity_ = this->create_wall_timer(
    //               pubVelocityInterval, std::bind(&OxtsNCom::timer_velocity_callback, this));
    // timer_time_reference_ = this->create_wall_timer(
    //               pubTimeReferenceInterval, std::bind(&OxtsNCom::timer_time_reference_callback, this));
    // timer_ecef_pos_   = this->create_wall_timer(
    //               pubEcefPosInterval, std::bind(&OxtsNCom::timer_ecef_pos_callback, this));
    // timer_nav_sat_ref_   = this->create_wall_timer(
    //               pubNavSatRefInterval, std::bind(&OxtsNCom::timer_nav_sat_ref_callback, this));
    // if (pub_tf_flag)
    // {
    //   timer_tf_   = this->create_wall_timer(
    //               pubImuInterval, std::bind(&OxtsNCom::timer_tf_callback, this));
    // } 

    nrx = NComCreateNComRxC();

  }

  /**
   * NCom decoder instance
   */
  NComRxC *nrx;
  /**
   * Buffer for UDP data
   */
  unsigned char buff[1024];

  rclcpp::Time get_timestamp();

};

} // namespace oxts_ncom

#endif //OXTS_NCOM__NCOM_HPP_
