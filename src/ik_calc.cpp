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
#include <vector>

class MoveItPlanner {
   public:
    // Constructor: creates the ROS node, initializes MoveGroupInterface,
    // and sets init_pose_, target_pose_, and prev_pose_ to the same value.
    MoveItPlanner(const std::string &planning_group = "right_arm")
        : node_(rclcpp::Node::make_shared("ik_calc",
                                          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
          init_pose_([]() {
              geometry_msgs::msg::Pose p;
              p.position.x = 0.3178071837760669;
              p.position.y = -0.10556869093811046;
              p.position.z = 1.1215113837188624;
              p.orientation.x = -0.1345047736672149;
              p.orientation.y = -0.1247301629367219;
              p.orientation.z = 0.6927253372844873;
              p.orientation.w = 0.697482945596954;
              return p;
          }()),
          target_pose_(init_pose_),
          prev_pose_(init_pose_) {
        move_group_interface_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
        addCollisionObject();
    }

    // Returns the constant initial pose.
    geometry_msgs::msg::Pose getInitPose() {
        return init_pose_;
    }

    // Sets a new target pose relative to prev_pose_.
    // The current target_pose_ is saved to prev_pose_ before updating.
    void setTargetPose(const double x_bias, const double y_bias, const double z_bias) {
        prev_pose_ = target_pose_;
        target_pose_.position.x += x_bias;
        target_pose_.position.y += y_bias;
        target_pose_.position.z += z_bias;
    }

    // Returns the current target pose.
    geometry_msgs::msg::Pose getTargetPose() {
        return target_pose_;
    }

    // Plans and executes a motion to the target_pose_ using the regular planner.
    bool planToPose(const double velocity = 0.1) {
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
        move_group_interface_->setMaxVelocityScalingFactor(velocity);
        move_group_interface_->setMaxAccelerationScalingFactor(velocity);
        double eef_step = 0.01;  // Distance between interpolated points
        moveit_msgs::msg::RobotTrajectory trajectory;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(prev_pose_);
        waypoints.push_back(target_pose_);
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, trajectory);
        if (fraction == 1.0) {
            move_group_interface_->execute(trajectory);
            prev_pose_ = target_pose_;
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning did not compute a complete path.");
            return false;
        }
    }

    // Runs the planner: first plans to the initial pose,
    // then sets a new target pose and plans a Cartesian path.
    void run() {
        // --- Plan to the initial pose ---
        RCLCPP_INFO(node_->get_logger(), "Planning to initial pose...");
        // Initially, target_pose_ is equal to init_pose_
        if (!planToPose(0.5)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan to initial pose.");
            return;
        }

        // --- Set a new target pose using a bias (e.g., offset along y-axis by -0.4) ---
        RCLCPP_INFO(node_->get_logger(), "Setting new target pose...");
        setTargetPose(0.0, -0.4, 0.0);

        // --- Plan a Cartesian path from the previous to the new target pose ---
        RCLCPP_INFO(node_->get_logger(), "Planning Cartesian path from previous to target pose...");
        if (!planCartesianPath(0.25)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan Cartesian path to target pose.");
        }
    }

   private:
    // Adds a collision object to the planning scene.
    void addCollisionObject() {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
        collision_object.id = "box1";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.284;
        primitive.dimensions[primitive.BOX_Y] = 1.3;
        primitive.dimensions[primitive.BOX_Z] = 1.1;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.36 + primitive.dimensions[primitive.BOX_X] / 2.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.0 + primitive.dimensions[primitive.BOX_Z] / 2.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.addCollisionObjects({collision_object});
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    const geometry_msgs::msg::Pose init_pose_;
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose prev_pose_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    MoveItPlanner planner;
    planner.run();
    rclcpp::shutdown();
    return 0;
}
