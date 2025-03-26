#include "robot_piano/moveit_planner.hpp"

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
