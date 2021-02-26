



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

using std::placeholders::_1;

namespace oxts_ncom
{

/**
 * Enumeration of timestamp modes for published topics
 */
enum PUB_TIMESTAMP_MODE
{
  /** Use ROS time. */
  ROS = 0,
  /** Use NCom time. */
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
  uint8_t ncom_rate;
  /*! Timestamp type to be applied to published packets
    {0 : ROS time, 1 : NCom time} */
  int timestamp_mode;
  /*! Frame ID of outgoing packets. @todo Having a general frame ID may not
    make sense. This isn't implemented. */
  std::string frame_id;
  /*! Publishing rate for debug String message. */
  uint8_t pub_string_rate;
  /*! Publishing rate for NavSatFix message. */
  uint8_t pub_nav_sat_fix_rate;
  /*! Publishing rate for Velocity message. */
  uint8_t pub_velocity_rate;
  /*! Publishing rate for TimeReference message.*/
  uint8_t pub_time_reference_rate; 
  /*! Publishing rate for PointStamped message. */
  uint8_t pub_ecef_pos_rate;
  /*! Publishing rate for PointStamped message. */
  uint8_t pub_nav_sat_ref_rate;
  /*! Flag to enable publishing of Imu message. */
  bool pub_imu_flag;
  /*! Flag to enable publishing of Tf messages. */
  bool pub_tf_flag;

  uint8_t ncomInterval;
  uint8_t pubStringInterval;
  uint8_t pubNavSatFixInterval;
  uint8_t pubTfInterval;
  uint8_t pubVelocityInterval;
  uint8_t pubTimeReferenceInterval;
  uint8_t pubEcefPosInterval;
  uint8_t pubNavSatRefInterval;
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

  void NCom_callback(const oxts_msgs::msg::Ncom::SharedPtr msg);
  /** Callback function for debug String message. Wraps message, publishes, and
   *  prints some information to the console.*/
  void string();
  /** Callback function for NavSatFix message. Wraps message and publishes. */
  void nav_sat_fix();
  /** Callback function for Imu message. Wraps message and publishes. */
  void imu();
  /** Callback function for Tf messages. Wraps messages and broadcasts. */
  void tf();
  /** Callback function for TimeReference message. Wraps message and publishes. */
  void time_reference();
  /** Callback function for Velocity message. Wraps message and publishes. */
  void velocity();
  /** Callback function for PointStamped message. Wraps message and 
   *  publishes.*/
  void ecef_pos();
  /** Callback function for OxTS NavSatRef message. Wraps message and 
   *  publishes.*/
  void nav_sat_ref();


  /**  Subscriber for oxts_msgs/msg/NCom. */
  rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr     subNCom_;
  /** Publisher for std_msgs/msg/string. Only used for debugging, currently 
   *  outputs lat, long, alt in string form. */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       pubString_;
  /** Publisher for /sensor_msgs/msg/NavSatFix */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNavSatFix_;
  /** Publisher for /sensor_msgs/msg/Imu */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       pubImu_;
  /** Publisher for /sensor_msgs/msg/TwistStamped */
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  pubVelocity_;
  /** Publisher for /sensor_msgs/msg/TimeReference */
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr  pubTimeReference_;
  /** Publisher for /geometry_msgs/msg/PointStamped */
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pubEcefPos_;
  /** Publisher for /oxts_msgs/msg/NavSatRef */
  rclcpp::Publisher<oxts_msgs::msg::NavSatRef>::SharedPtr  pubNavSatRef_;
  /** TF broadcaster */
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
    ncom_rate               = this->declare_parameter("ncom_rate", 100);
    timestamp_mode          = this->declare_parameter("timestamp_mode", 0); 
    frame_id                = this->declare_parameter("frame_id", "oxts_link");
    pub_string_rate         = this->declare_parameter("pub_string_rate", 1);
    pub_nav_sat_fix_rate    = this->declare_parameter("pub_nav_sat_fix_rate", 1);
    pub_imu_flag            = this->declare_parameter("pub_imu_flag", true);
    pub_velocity_rate       = this->declare_parameter("pub_velocity_rate", 1);
    pub_time_reference_rate = this->declare_parameter("pub_time_reference_rate", 1);
    pub_ecef_pos_rate       = this->declare_parameter("pub_ecef_pos_rate", 1);
    pub_nav_sat_ref_rate    = this->declare_parameter("pub_nav_sat_ref_rate", 1);
    pub_tf_flag             = this->declare_parameter("pub_tf_flag", true);

    pubStringInterval        = (pub_string_rate         == 0) ? 0 : ncom_rate / pub_string_rate;
    pubNavSatFixInterval     = (pub_nav_sat_fix_rate    == 0) ? 0 : ncom_rate / pub_nav_sat_fix_rate;
    pubVelocityInterval      = (pub_velocity_rate       == 0) ? 0 : ncom_rate / pub_velocity_rate;
    pubTimeReferenceInterval = (pub_time_reference_rate == 0) ? 0 : ncom_rate / pub_time_reference_rate;
    pubEcefPosInterval       = (pub_ecef_pos_rate       == 0) ? 0 : ncom_rate / pub_ecef_pos_rate;
    pubNavSatRefInterval     = (pub_nav_sat_ref_rate    == 0) ? 0 : ncom_rate / pub_nav_sat_ref_rate;

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
    // Initialise subscriber for ncom message
    subNCom_ = this->create_subscription<oxts_msgs::msg::Ncom>
                      ("ncom",10,std::bind(&OxtsNCom::NCom_callback,this,_1));

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
