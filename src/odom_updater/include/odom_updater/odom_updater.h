#pragma once
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

/**
 * @brief Class with an aim to broadcast the robot1/base_footprint 
 * in robot1/odom, dynamicaly working broadcaster has to be used
 */
class FramePublisher : public rclcpp::Node
{
public:
/**
 * @brief Construct a new Frame Publisher object and creates a odom_updater node
 * 
 */
  FramePublisher(): Node("odom_updater")
  {
    // Declare and acquire `robotname` parameter
    robotname_ = this->declare_parameter<std::string>("robotname", "robot1");

    // Initialize the transform broadcaster
    pose_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /*By the means of ostringstream topic
    regarding to the current robot's name is created
    */
    std::ostringstream stream;
    stream << "/" << robotname_.c_str() << "/odom";
    std::string topic_name = stream.str();
    //Subcribing to the robot's odom, messages are handled by 
    robot_pose = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_robot_pose, this, std::placeholders::_1));
  }

private:
/**
 * @brief Broadcasting the robot1/base_footprint 
 * in robot1/odom,
 * 
 * @param msg The required message type to nav_msgs
 */
  void handle_robot_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
 
  //Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose;
  //Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> pose_broadcaster;
  //Name of the robot
  std::string robotname_;
};