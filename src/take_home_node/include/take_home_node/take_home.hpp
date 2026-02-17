#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

 private:
  float wheelSlip_rear(float vx, float w, float vw, bool leftW);        // calc the wheel slips
  float wheelSlip_front(float vx, float w, float vy, float angle, float vw, bool leftW);

  const float WF = 1.638;   // front track width (meters)
  const float WR = 1.523;    //  rear track width (meters)
  const float LF = 1.7238;  // longitdinal dist from COG to front wheels (meters)

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  rclcpp::Subscription<raptor_dbw_msgs::msgs::WheelSpeedReport>::SharedPtr wspeed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msgs::SteeringExtededReport>::SharedPtr wangle_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_rl_publisher_;  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wslip_fl_publisher_;

};
