#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <unistd.h>

#include "nav_msgs/msg/odometry.hpp"
#include "icuas25_msgs/msg/target_info.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "crazyflie_interfaces/srv/go_to.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class NavTest : public rclcpp::Node{
  public:
    NavTest(): Node("nav_test_node"){
      
      agent_pose_subscriber  = this->create_subscription<nav_msgs::msg::Odometry>("cf_1/odom", 10, std::bind(&NavTest::agent_pose_callback, this, std::placeholders::_1));
      aruco_subscriber       = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("cf_1/aruco_markers", 10, std::bind(&NavTest::aruco_callback, this, std::placeholders::_1));

      // Publishers
      target_publisher = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);

      // Service client
      goto_client = this->create_client<crazyflie_interfaces::srv::GoTo>("cf_1/go_to");

      // Make 0.1s timer
      //control_timer = this->create_wall_timer(10ms, std::bind(&NavTest::control_callback, this));

      // Initial goto instruction

      // Call the service
      request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
      request->goal.x = 1.0;
      request->goal.y = 0.0;
      request->goal.z = 1.0;
      request->yaw = 0.0;

      std::cout << "Running req" << std::endl;
      auto result = goto_client->async_send_request(request);

      sleep(5);
      request->yaw = 1.57;

      std::cout << "Running req" << std::endl;
      result = goto_client->async_send_request(request);

      std::cout << "Req returned " << std::endl;

    }

    void agent_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      agent_pose = *msg;
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
      aruco_id = msg->marker_ids[0];
      if( (msg->poses.size() > 0) && (aruco_id != aruco_id_old)){ 
	icuas25_msgs::msg::TargetInfo target_info;
	target_info.id = msg->marker_ids[0];
	tf2::Quaternion q;
	tf2::Vector3 v;
	tf2::fromMsg(agent_pose.pose.pose.orientation, q);
	// Place aruco pose in v
	tf2::fromMsg(msg->poses[0].position, v);
	tf2::Vector3 rotated_v = tf2::quatRotate(q, v);
	// Rotate the vision vector to the agent frame
	target_info.location.x = agent_pose.pose.pose.position.x + rotated_v.x(); 
	target_info.location.y = agent_pose.pose.pose.position.y + rotated_v.y();
	target_info.location.z = agent_pose.pose.pose.position.z + rotated_v.z();
	target_publisher->publish(target_info);

	// Call the service
	request->goal.x = -15.0; 
	request->goal.y = -2.0; 

	auto result = goto_client->async_send_request(request);

	aruco_id_old = aruco_id;
      }
    }

  private:

    int aruco_id = -2;
    int aruco_id_old = -1;

    nav_msgs::msg::Odometry agent_pose;

    crazyflie_interfaces::srv::GoTo::Request::SharedPtr request;

    //rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agent_pose_subscriber;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;

    rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr target_publisher;

    rclcpp::Client<crazyflie_interfaces::srv::GoTo>::SharedPtr goto_client;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavTest>());
  rclcpp::shutdown();
  return 0;
}
