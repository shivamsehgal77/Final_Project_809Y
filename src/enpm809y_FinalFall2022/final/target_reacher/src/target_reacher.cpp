#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"



void TargetReacher::reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      // Printing the state of the action
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
      // In case target has been reached the condition is true and the message is being published
      if(msg->data){
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = c;
        pub_vel->publish(msg);
        
      }
    }

   
    void TargetReacher::final_move(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
      // Printing the state of the action
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard:" << msg->marker_ids.at(0));

      //Extracting the ID and converting it to string
      std::string m_id=std::to_string(msg->marker_ids.at(0));   
     
     //Concatenating the final_destination based on the ID
      std::string final_x = "final_destination.aruco_" + m_id + ".x";
      std::string final_y ="final_destination.aruco_" + m_id + ".y";
      
      //Getting the already declaered positions
      auto target_x=this->get_parameter(final_x).as_double();
      auto target_y=this->get_parameter(final_y).as_double();

      /*Calling the broadcaster_callback with the
      targets in the given origin's frame
      */
      broadcaster_callback(target_x,target_y);

      //Calling the listener_callback
      listener_callback();
      
    }

    //broadcaster function create

    void TargetReacher::broadcaster_callback(double a,double b) {
      geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        //Since the already decleared parameters are used it work dinamically
        t.header.frame_id =this->get_parameter("final_destination.frame_id").as_string();
        t.child_frame_id = "final_destination";
        
        // The final_destination is at this position from given origin
        t.transform.translation.x = a;
        t.transform.translation.y = b;
        t.transform.translation.z = 0.0;

        // The final_destination is at this orientation from given origin
        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w =1;

        // Send the transformation
        m_tf_broadcaster->sendTransform(t);
    }

     void TargetReacher::listener_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        /*Look up for the transformation between odom and final_destination
        In case the transformation has been found the movement can start
        to the absolute location of final_destination
        c is set to 0 to stop the rotation when target has been reached
        If the transformation hasn't been found error message is printed
        */ 
        try
        {
            t = m_tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
            m_bot_controller->set_goal(t.transform.translation.x,t.transform.translation.y); 
            c=0.0;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                "robot1/odom", "final_destination", ex.what());
            return;
        }
        //Printing the final_destination in the odom's frame
        RCLCPP_INFO(
            this->get_logger(), "Position of object in odom: [%f, %f, %f]", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    }

    
 
