#include "robot_piano/hand_publisher.hpp"

using namespace std::chrono_literals;

class HandPublisher : public rclcpp::Node {
   public:
    HandPublisher(const std::string &topic_name) : Node("hand_publisher") {
        hand_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(topic_name, 10);
        hand_point_.positions = {3.14, 0.0, 0.0, 0.0, 0.0};
        timer_ = create_wall_timer(10ms, std::bind(&HandPublisher::timer_callback, this));
    }

    std::vector<double> getHandAngles() { return hand_point_.positions; }
    void setHandAngles(const std::vector<double> &angles) { hand_point_.positions = angles; }

   private:
    void timer_callback() { hand_pub_->publish(hand_point_); }
    const std::string arm_name_;
    trajectory_msgs::msg::JointTrajectoryPoint hand_point_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr hand_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
