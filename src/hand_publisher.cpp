#include "robot_piano/hand_publisher.hpp"

using namespace std::chrono_literals;

HandPublisher::HandPublisher(const std::string &topic_name) : Node("hand_publisher") {
    hand_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(topic_name, 10);
    hand_point_.positions = {3.14, 0.0, 0.0, 0.0, 0.0};
    timer_ = create_wall_timer(10ms, std::bind(&HandPublisher::timer_callback, this));
}

std::vector<double> HandPublisher::getHandAngles() { return hand_point_.positions; }
void HandPublisher::setHandAngles(const std::vector<double> &angles) {
    hand_point_.positions = angles;
}
void HandPublisher::timer_callback() { hand_pub_->publish(hand_point_); }
