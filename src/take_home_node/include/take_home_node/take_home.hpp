#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheelSlip_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                          raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg,
                          raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg);
  void topJitter_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_msg);
  void botJitter_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_msg);
  void vnavJitter_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vnav_msg);

 private:
  const float WF = 1.638f;   // front track width (meters)
  const float WR = 1.523f;    //  rear track width (meters)
  const float LF = 1.7238f;  // longitdinal dist from COG to front wheels (meters)

  float wheelSlip_rear(float vx, float w, float vw, bool leftW);        // calc the wheel slips
  float wheelSlip_front(float vx, float w, float vy, float angle, float vw, bool leftW);
  float convertSpeed(float kmph);                   // convert from kmph to m/s
  float convertAngle(float deg);          // convert from deg to rad

  //keep map of dequeus here

  float calcJitter(int windowId);   // calculate the jitter for the current specified window

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> Wodometry_subscriber_;
  message_filters::Subscriber<raptor_dbw_msgs::msg::WheelSpeedReport> wspeed_subscriber_;           //  subscriptions for data for wheel speed
  message_filters::Subscriber<raptor_dbw_msgs::msg::SteeringExtendedReport> wangle_subscriber_;
  //  https://docs.ros.org/en/humble/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_rl_publisher_;                // wheel slip publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_fl_publisher_;

  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
   nav_msgs::msg::Odometry,
   raptor_dbw_msgs::msg::WheelSpeedReport,                           // synchronizer for wheel slip
   raptor_dbw_msgs::msg::SteeringExtendedReport>>> sync;

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr topImu_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr bottomImu_subscriber_;     // subscriptions for imus
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr vnImu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_top_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_bottom_publisher_;    // publishers for jitters
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_vn_publisher_;

};
