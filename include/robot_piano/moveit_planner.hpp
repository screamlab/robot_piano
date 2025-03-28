#pragma once
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif
#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "robot_piano/params.hpp"

class MoveItPlanner {
   public:
    // Constructor: creates the ROS node, initializes MoveGroupInterface,
    // and sets init_pose_, target_pose_, and prev_pose_ to the same value.
    MoveItPlanner(const rclcpp::Node::SharedPtr &node = rclcpp::Node::make_shared(
                      "right_arm_ik_calc",
                      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
                  const std::string &planning_group = "right_arm",
                  const geometry_msgs::msg::Pose &init_pose = right_init_pose,
                  const std::vector<moveit_msgs::msg::CollisionObject> &collision_objects = {
                      piano_object});

    // Returns the constant initial pose.
    geometry_msgs::msg::Pose getInitPose();

    // Sets a new target pose relative to prev_pose_.
    // The current target_pose_ is saved to prev_pose_ before updating.
    void setTargetPose(const double x_bias, const double y_bias, const double z_bias);

    // Rotate the current target pose along the x-axis.
    // The current target_pose_ is saved to prev_pose_ before updating.
    void rotateTargetPoseX(const double angle);

    // Returns the current target pose.
    geometry_msgs::msg::Pose getTargetPose();

    // Plans and executes a motion to the target_pose_ using the regular
    // planner.
    bool planToPose(const double velocity = 0.1);

    // Plans and executes a Cartesian path from prev_pose_ to target_pose_.
    bool planCartesianPath(const double velocity = 0.1);

   private:
    // Adds a collision-object vector to the planning scene.
    void addCollisionObject(const std::vector<moveit_msgs::msg::CollisionObject> &co);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    const geometry_msgs::msg::Pose init_pose_;
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose prev_pose_;
};
