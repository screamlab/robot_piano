#include <unistd.h>

#include <condition_variable>
#include <mutex>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_piano/hand_publisher.hpp"
#include "robot_piano/moveit_planner.hpp"
#include "robot_piano/utils.hpp"

class SyncService : public rclcpp::Node {
   public:
    SyncService() : Node("left_sync_service"), start_triggered_(false) {
        // Create a service server for "start_sync"
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "left_start_sync", std::bind(&SyncService::start_callback, this, std::placeholders::_1,
                                         std::placeholders::_2));
    }

    // Blocks until the "start_sync" service call is received.
    void wait_for_start() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this]() { return start_triggered_; });
    }

   private:
    void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            start_triggered_ = true;
        }
        cond_var_.notify_one();
        response->success = true;
        response->message = "Start triggered.";
        RCLCPP_INFO(this->get_logger(), "Received start_sync service call. Starting work...");
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    bool start_triggered_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
};

int main(int argc, char *argv[]) {
    /**
     * Initialize the ROS 2 C++ client library.
     */
    rclcpp::init(argc, argv);

    auto sync_node = std::make_shared<SyncService>();

    // Use a separate thread for spinning so that service callbacks are processed.
    std::thread sync_thread([sync_node]() { rclcpp::spin(sync_node); });

    // Create a node for the left arm.
    auto left_node = rclcpp::Node::make_shared(
        "left_arm_ik_calc",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Instantiate the planner for the left arm.
    MoveItPlanner left_planner(left_node, "left_arm", left_init_pose, {piano_object});

    auto hand_publisher = std::make_shared<HandPublisher>(
        "/left_hand", std::vector<double>{deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    RCLCPP_INFO(left_node->get_logger(), "Published initial left hand joint angles.");

    rclcpp::executors::SingleThreadedExecutor hand_executor;
    hand_executor.add_node(hand_publisher);
    std::thread hand_thread([&hand_executor]() { hand_executor.spin(); });

    // Perform the first planning phase.
    left_planner.setTargetPose("left_t_pose");
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");
    left_planner.setTargetPose(left_mid_pose);
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");
    left_planner.setTargetPose([&]() {
        auto pose = left_init_pose;
        pose.position.y += 0.2;
        return pose;
    }());
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");
    left_planner.setTargetPose(left_init_pose);
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");

    /**
     * Block here until the service callback sets the flag.
     */
    RCLCPP_WARN(sync_node->get_logger(), "Waiting for start_sync service call...");
    sync_node->wait_for_start();

    RCLCPP_WARN(sync_node->get_logger(), "Sync triggered, proceeding with main work...");

    /**
     * Proceed with the main planning phase.
     */
    /* Move to the first note. */
    // left_planner.setTargetPose(0.0, 0.18, 0.015);
    left_planner.setTargetPose(0.0, 0.18, 0.0);
    RCLCHECK(left_planner.planCartesianPath(0.1), left_node, "Left");
    left_planner.removeCollisionObject({piano_object.id});

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, deg2rad(90.0)});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, deg2rad(90.0), 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, deg2rad(90.0), 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), deg2rad(90.0), 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    /* Move to the second note. */
    left_planner.addCollisionObject({piano_object});
    left_planner.setTargetPose(0.0, -0.095, -0.015);
    RCLCHECK(left_planner.planCartesianPath(0.1), left_node, "Left");
    left_planner.removeCollisionObject({piano_object.id});

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, deg2rad(90.0)});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, deg2rad(90.0), 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, deg2rad(90.0), 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    // Press the key.
    hand_publisher->setHandAngles({deg2rad(180.0), deg2rad(90.0), 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, -0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");

    usleep(500 * 1000);  // Sleep for 500ms

    hand_publisher->setHandAngles({deg2rad(180.0), 0.0, 0.0, 0.0, 0.0});
    left_planner.setTargetPose(0.0, 0.0, 0.05);
    RCLCHECK(left_planner.planCartesianPath(1.0), left_node, "Left");
    left_planner.addCollisionObject({piano_object});

    usleep(500 * 1000);  // Sleep for 500ms

    /**
     * Return to the rest pose.
     */
    left_planner.setTargetPose(0.0, 0.4, 0.25);
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");

    left_planner.setTargetPose(left_mid_pose);
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");

    left_planner.setTargetPose("left_t_pose");
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");

    left_planner.setTargetPose("left_rest");
    RCLCHECK(left_planner.planToPose(0.1), left_node, "Left");

    // Shutdown the process after finishing all tasks.
    rclcpp::shutdown();

    // Stop the hand publisher executor and join the thread
    hand_executor.cancel();
    hand_thread.join();

    // Stop the sync service thread
    sync_thread.join();
    return 0;
}
