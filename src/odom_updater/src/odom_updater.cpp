
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "odom_updater/odom_updater.h"
#include <nav_msgs/msg/odometry.hpp>

/**
 * @brief Broadcasting the robot1/base_footprint 
 * in robot1/odom,
 * 
 * @param msg The required message type to nav_msgs
 */
void FramePublisher::handle_robot_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    //t contains information about the message itself and the transformation
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot1/odom";
    t.child_frame_id = "robot1/base_footprint";

    // Extracting the robot's x,y,z-position from msg
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // Extracting the robot's x,y,z-orientation from msg
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    pose_broadcaster->sendTransform(t);
  }

/**
 * @brief Main for running the transformation continously
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  
  return 0;
}