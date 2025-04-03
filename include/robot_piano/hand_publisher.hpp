#pragma once
#include "rclcpp/rclcpp.hpp"
#include "robot_piano/utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class HandPublisher : public rclcpp::Node {
   public:
    HandPublisher(const std::string &topic_name, const std::vector<double> &init_position);

    std::vector<double> getHandAngles();
    void setHandAngles(const std::vector<double> &angles);

   private:
    void timer_callback();
    const std::string arm_name_;
    trajectory_msgs::msg::JointTrajectoryPoint hand_point_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr hand_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
