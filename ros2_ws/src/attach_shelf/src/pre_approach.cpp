#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>
#include <cmath>


class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach_node") 
  {
    // Initialize parameters
    this->declare_parameter("obstacle", 1.0);
    this->declare_parameter("degrees", 90);

    getting_params();

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_;

    // Create publisher for cmd_vel
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PreApproachNode::cmdvelCallBack, this), callback_group_);

    
    // Create subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproachNode::laserscanCallback, this, std::placeholders::_1), options1);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&PreApproachNode::odomCallback, this, std::placeholders::_1), options1);
  }

  void getting_params() {
    obstacle_ = this->get_parameter("obstacle").get_parameter_value().get<double>();
    degrees_ = this->get_parameter("degrees").get_parameter_value().get<int>();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  geometry_msgs::msg::Twist twist_msg_;
  double obstacle_;
  int degrees_;
  double distance_= 100.0;
  double yaw_ = 0;

  // flags
  bool flag_forward_ = true;
  bool flag_rotation_ = false;
  bool flag_rotate_once_ = true;
  bool finish_flag_ = false;  

void laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
  {
    double zero =  0; // front
    
    int zero_index = int((zero - msg->angle_min) / msg->angle_increment);
    
    if (!std::isnan(msg->ranges[zero_index]) && !std::isinf(msg->ranges[zero_index])) 
    {
        distance_ = msg->ranges[zero_index];
    }
  }

  void laserscanCallback2(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
  {
    /* Me gusto esta funcion asi que la voy a guardar*/
    
    double min_angle =  -0.00872; // -0.5degree
    double max_angle =  0.00872; // 0.5 degree
    int minus_index = int((min_angle - msg->angle_min) / msg->angle_increment);
    int pos_index   = int((max_angle - msg->angle_min) / msg->angle_increment);

    std::vector<double> laser_data;

    for (int i = minus_index; i <= pos_index; i++) 
    {
        if (min_angle > msg->ranges.size() || max_angle > msg->ranges.size() || i < 0) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Index out of size.");
            return;
        } 
        if (!std::isnan(msg->ranges[i]) && !std::isinf(msg->ranges[i])) 
        {
            laser_data.push_back(msg->ranges[i]);
        }
    }

    if (laser_data.size() == 0)
    {
        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "laser data empty");
        return;
    }
    distance_ = std::accumulate(laser_data.begin(), laser_data.end(), 0) / static_cast<double>(laser_data.size());
  }


  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    double roll, pitch;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_);
    yaw_ = yaw_ * (180.0 / M_PI);
  }

  void cmdvelCallBack() 
  {
    // First move forward
    if (flag_forward_) 
    {
        // Move forward
        twist_msg_.linear.x = 0.5;
        twist_msg_.angular.z = 0.0;
        flag_forward_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "1");
    }

    // Stop the robot
    if (distance_ <= obstacle_ && flag_rotate_once_) 
    {
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        flag_rotation_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "2");
    } 
    
    // Rotation
    if (flag_rotation_ && flag_rotate_once_)
    {
        double angular_velocity;
        if (degrees_ < 0)
        {
            angular_velocity = -0.5;
        }
        else
        {
            angular_velocity = 0.55;
        }
        twist_msg_.linear.x = 0.0; 
        twist_msg_.angular.z = angular_velocity;
        flag_rotate_once_ = false;
        finish_flag_ = true;       
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%f",twist_msg_.angular.z); 
    }

    // Stop the rotation
    if (std::fabs(yaw_ - degrees_) <= 1 && finish_flag_)
    {
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        
        // After the sleep, reset the flag
        flag_rotate_once_ = false;
        finish_flag_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "4");
    }
    // Publish cmd_vel based on decision
    cmd_vel_pub_->publish(twist_msg_);
 }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
 
  std::shared_ptr<PreApproachNode> preapproach_node = std::make_shared<PreApproachNode>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(preapproach_node);
  executor.spin();

  return 0;
}
