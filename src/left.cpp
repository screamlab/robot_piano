#include <signal.h>
#include <unistd.h>

#include "robot_piano/moveit_planner.hpp"
#include "robot_piano/utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Global flag to indicate signal reception.
volatile sig_atomic_t signal_received = 0;

// Signal handler to set the flag.
void signal_handler(int signum) {
    if (signum == SIGINT) {
        RCLCPP_WARN(rclcpp::get_logger("left_arm_ik_calc"), "Received SIGINT signal.");
        rclcpp::shutdown();
        exit(signum);
    } else if (signum == SIGUSR1) {
        signal_received = 1;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create a node for the left arm.
    auto left_node = rclcpp::Node::make_shared(
        "left_arm_ik_calc",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Retrieve the current process ID
    pid_t pid = getpid();

    // Log the process ID using rclcpp's logging mechanism
    RCLCPP_WARN(left_node->get_logger(), "Node is running with PID: %d", pid);

    // Instantiate the planner for the left arm.
    MoveItPlanner left_planner(left_node, "left_arm", left_init_pose, {piano_object});

    // Create a publisher for JointTrajectoryPoint on the '/left_hand' topic.
    auto left_hand_pub =
        left_node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/left_hand", 10);

    // Prepare the initial JointTrajectoryPoint message.
    trajectory_msgs::msg::JointTrajectoryPoint init_point;
    init_point.positions = {deg2rad(180.0), 0.0, 0.0, 0.0, 0.0};

    // Publish the initial angles.
    left_hand_pub->publish(init_point);
    RCLCPP_INFO(left_node->get_logger(), "Published initial left hand joint angles.");

    // Register signal handler for SIGUSR1.
    signal(SIGUSR1, signal_handler);
    signal(SIGINT, signal_handler);

    // Perform the first planning phase.
    RCLCHECK(left_planner.planToPose(0.8), "Left");

    // // Wait for the signal.
    // RCLCPP_WARN(left_node->get_logger(),
    //             "Waiting for SIGUSR1 to proceed with the second planning phase...");
    // while (!signal_received) {
    //     sleep(1);  // Sleep to reduce CPU usage while waiting.
    // }

    // Proceed with the main planning phase.
    left_planner.setTargetPose(0.0, 0.4, 0.0);
    RCLCHECK(left_planner.planCartesianPath(0.8), "Left");

    init_point.positions[1] = deg2rad(90);
    left_hand_pub->publish(init_point);

    left_planner.rotateTargetPoseX(deg2rad(-10));
    RCLCHECK(left_planner.planCartesianPath(0.3), "Left");

    // left_planner.setTargetPose(-0.1, 0.0, 0.1);
    // RCLCHECK(left_planner.planCartesianPath(0.1), "Left");

    // Shutdown the process after finishing all tasks.
    rclcpp::shutdown();
    return 0;
}
