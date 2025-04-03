#include "robot_piano/moveit_planner.hpp"

MoveItPlanner::MoveItPlanner(
    const rclcpp::Node::SharedPtr &node, const std::string &planning_group,
    const geometry_msgs::msg::Pose &init_pose,
    const std::vector<moveit_msgs::msg::CollisionObject> &collision_objects)
    : node_(node), init_pose_(init_pose), target_pose_(init_pose_), prev_pose_(init_pose_) {
    move_group_interface_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
    addCollisionObject(collision_objects);
}

geometry_msgs::msg::Pose MoveItPlanner::getInitPose() { return init_pose_; }

void MoveItPlanner::setTargetPose(const double x_bias, const double y_bias, const double z_bias) {
    RCLCPP_INFO(node_->get_logger(), "Setting new target pose...");
    prev_pose_ = target_pose_;
    target_pose_.position.x += x_bias;
    target_pose_.position.y += y_bias;
    target_pose_.position.z += z_bias;
}

void MoveItPlanner::rotateTargetPoseX(const double angle_rad) {
    RCLCPP_INFO(node_->get_logger(), "Rotating target pose along the X-axis...");
    prev_pose_ = target_pose_;
    // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d using tf2::fromMsg
    Eigen::Isometry3d eigen_transform;
    tf2::fromMsg(prev_pose_, eigen_transform);

    Eigen::AngleAxisd rotation_vector(angle_rad, Eigen::Vector3d::UnitX());

    // Apply the rotation to the original transform
    eigen_transform.rotate(rotation_vector);

    // Convert the Eigen transform back to geometry_msgs::msg::Pose
    target_pose_ = tf2::toMsg(eigen_transform);
}

geometry_msgs::msg::Pose MoveItPlanner::getTargetPose() { return target_pose_; }

bool MoveItPlanner::planToPose(const double velocity) {
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

bool MoveItPlanner::planCartesianPath(const double velocity) {
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
    if (fraction >= 0.0) {
        RCLCPP_INFO(node_->get_logger(), "fraction: %.2f", fraction);
        move_group_interface_->execute(trajectory);
        prev_pose_ = target_pose_;
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "Cartesian path planning did not compute a complete path.");
        return false;
    }
}

void MoveItPlanner::addCollisionObject(const std::vector<moveit_msgs::msg::CollisionObject> &co) {
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

void MoveItPlanner::removeCollisionObject(const std::vector<std::string> &object_ids) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.removeCollisionObjects(object_ids);
}
