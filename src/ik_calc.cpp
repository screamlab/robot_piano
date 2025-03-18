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
    MoveItPlanner(
        const rclcpp::Node::SharedPtr &node = rclcpp::Node::make_shared(
            "right_arm_ik_calc",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
        const std::string &planning_group = "right_arm",
        const geometry_msgs::msg::Pose &init_pose = right_init_pose,
        const std::vector<moveit_msgs::msg::CollisionObject> &collision_objects = {piano_object})
        : node_(node), init_pose_(init_pose), target_pose_(init_pose_), prev_pose_(init_pose_) {
        move_group_interface_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
        addCollisionObject(collision_objects);
    }

    // Returns the constant initial pose.
    geometry_msgs::msg::Pose getInitPose() { return init_pose_; }

    // Sets a new target pose relative to prev_pose_.
    // The current target_pose_ is saved to prev_pose_ before updating.
    void setTargetPose(const double x_bias, const double y_bias, const double z_bias) {
        RCLCPP_INFO(node_->get_logger(), "Setting new target pose...");
        prev_pose_ = target_pose_;
        target_pose_.position.x += x_bias;
        target_pose_.position.y += y_bias;
        target_pose_.position.z += z_bias;
    }

    // Returns the current target pose.
    geometry_msgs::msg::Pose getTargetPose() { return target_pose_; }

    // Plans and executes a motion to the target_pose_ using the regular
    // planner.
    bool planToPose(const double velocity = 0.1) {
        RCLCPP_INFO(node_->get_logger(), "Planning to target pose...");
        move_group_interface_->setPoseTarget(target_pose_);
        move_group_interface_->setMaxVelocityScalingFactor(velocity);
        move_group_interface_->setMaxAccelerationScalingFactor(velocity);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface_->plan(plan));
        if (success) {
            move_group_interface_->execute(plan);
            prev_pose_ = target_pose_;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for the target pose.");
        }
        return success;
    }

    // Plans and executes a Cartesian path from prev_pose_ to target_pose_.
    bool planCartesianPath(const double velocity = 0.1) {
        RCLCPP_INFO(node_->get_logger(), "Planning Cartesian path from previous to target pose...");
        move_group_interface_->setMaxVelocityScalingFactor(velocity);
        move_group_interface_->setMaxAccelerationScalingFactor(velocity);
        double eef_step = 0.01;  // Distance between interpolated points
        moveit_msgs::msg::RobotTrajectory trajectory;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(prev_pose_);
        waypoints.push_back(target_pose_);
        double fraction =
            move_group_interface_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
        if (fraction == 1.0) {
            move_group_interface_->execute(trajectory);
            prev_pose_ = target_pose_;
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cartesian path planning did not compute a complete path.");
            return false;
        }
    }

   private:
    // Adds a collision-object vector to the planning scene.
    void addCollisionObject(const std::vector<moveit_msgs::msg::CollisionObject> &co) {
        const auto collision_objects = [&]() {
            std::vector<moveit_msgs::msg::CollisionObject> collision_objects = co;
            for (auto &c : collision_objects) {
                c.header.frame_id = move_group_interface_->getPlanningFrame();
            }
            return collision_objects;
        }();

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.addCollisionObjects(collision_objects);
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    const geometry_msgs::msg::Pose init_pose_;
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose prev_pose_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create separate nodes with unique names.
    auto left_node = rclcpp::Node::make_shared(
        "left_arm_ik_calc",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto right_node = rclcpp::Node::make_shared(
        "right_arm_ik_calc",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Instantiate each planner with its own node.
    MoveItPlanner left_planner(left_node, "left_arm", left_init_pose, {piano_object});
    MoveItPlanner right_planner(right_node, "right_arm", right_init_pose, {piano_object});

    // Create threads to run planning for each arm concurrently.
    std::thread left_thread([&]() {
        if (!left_planner.planToPose(0.5)) {
            RCLCPP_ERROR(left_node->get_logger(), "Left arm: planning to initial pose failed.");
            return;
        }
        left_planner.setTargetPose(0.0, 0.4, 0.0);
        if (!left_planner.planCartesianPath(0.25)) {
            RCLCPP_ERROR(left_node->get_logger(), "Left arm: Cartesian path planning failed.");
        }
    });

    std::thread right_thread([&]() {
        if (!right_planner.planToPose(0.5)) {
            RCLCPP_ERROR(right_node->get_logger(), "Right arm: planning to initial pose failed.");
            return;
        }
        right_planner.setTargetPose(0.0, -0.4, 0.0);
        if (!right_planner.planCartesianPath(0.25)) {
            RCLCPP_ERROR(right_node->get_logger(), "Right arm: Cartesian path planning failed.");
        }
    });

    // Use a MultiThreadedExecutor to concurrently spin both nodes.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(left_node);
    executor.add_node(right_node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    left_thread.join();
    right_thread.join();

    // Stop the executor and join its thread.
    executor.cancel();
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
