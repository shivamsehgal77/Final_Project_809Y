#pragma once
#include<chrono>
#include<memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <functional>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

/**
 * @brief The given class with the aim to reach the required targets
 * 
 */
class TargetReacher : public rclcpp::Node
{
public:
/**
 * @brief Construct a new Target Reacher object and creates a target_reacher node
 * 
 * @param bot_controller controller that controls the motion of the robot using a pid controller
 */
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        //The motion controller and the x,y values of the marker are initialized
        m_bot_controller = bot_controller;
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");

        //The position parameters are decleared, so they can be used later on
        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y"); 
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");
        
        //The robot moves to the Marker
        m_bot_controller->set_goal(aruco_target_x,aruco_target_y);
        
        //Initializing the subscription to the sub_goal
        sub_goal = this->create_subscription<std_msgs::msg::Bool>(
      "/goal_reached", 10, std::bind(&TargetReacher::reached_callback, this, std::placeholders::_1));
      
        //Initializing the subscriber to the Marker
        sub_marker = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", 10, std::bind(&TargetReacher::final_move, this, std::placeholders::_1));

        //Initializing the publisher for rotation
        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // Initialize the transform broadcaster
        m_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        

        // Initializing the Buffer to initialize the TransformListener
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    // Attributes that were given for controlling the movement
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_marker;
    
    /**
     * @brief Callback function when target has been reached.
     * At the first reach it starts rotating since a value c >0
     * is being published to msg.angular.z
     * After the Marker was scanned, in the listener function
     * c=0 is set, therefore rotation will not occour anymore.
     * @param msg The required message type to std_msgs
     */
    void reached_callback(const std_msgs::msg::Bool::SharedPtr msg);

    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    //Broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster;


    /**
     * @brief Broadcasting the final_destination in the required origin,
     * that upgrades dynamicaly
     * @param a Target in X
     * @param b Target in Y
     */
    void broadcaster_callback(double a, double b);

     /**
     * @brief When the marker was scanned corresponding ID is extracted,
     * based on it the final destination can be selected.
     * 
     * @param msg The required message type to ros2_aruco_interfaces
     */
    void final_move(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Listening to the transformation of the final_destination
     * to the robot1's frame, therefore the absolut position is known
     * and movement can start
     */
    void listener_callback();



    //TrasnformListener
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};

    //Buffer for time
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    //Variable for the Z-rotation
    double c=0.2;
    
    
};