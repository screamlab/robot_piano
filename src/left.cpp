#include <signal.h>
#include <unistd.h>

#include "robot_piano/hand_publisher.hpp"
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

    auto hand_publisher = std::make_shared<HandPublisher>("/left_hand");
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    RCLCPP_INFO(left_node->get_logger(), "Published initial left hand joint angles.");

    rclcpp::executors::SingleThreadedExecutor hand_executor;
    hand_executor.add_node(hand_publisher);
    std::thread hand_thread([&hand_executor]() { hand_executor.spin(); });

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

    // Adjust the hand angles during runtime
    hand_publisher->setHandAngles({deg2rad(180.0), deg2rad(90.0), 0.0, 0.0, 0.0});

    left_planner.rotateTargetPoseX(deg2rad(-10));
    RCLCHECK(left_planner.planCartesianPath(0.3), "Left");

    // left_planner.setTargetPose(-0.1, 0.0, 0.1);
    // RCLCHECK(left_planner.planCartesianPath(0.1), "Left");

    // Shutdown the process after finishing all tasks.
    rclcpp::shutdown();

    // Stop the hand publisher executor and join the thread
    hand_executor.cancel();
    hand_thread.join();
    return 0;
}
