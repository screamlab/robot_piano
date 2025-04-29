// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <domain_bridge/component_manager.hpp>
#include <domain_bridge/domain_bridge.hpp>
#include <domain_bridge/parse_domain_bridge_yaml_config.hpp>
#include <domain_bridge/process_cmd_line_arguments.hpp>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

int main(int argc, char **argv) {
    auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

    domain_bridge::DomainBridge domain_bridge;
    domain_bridge.bridge_service<std_srvs::srv::Trigger>("right_start_sync", 1, 0);

    // Add component manager node and domain bridge to single-threaded executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<domain_bridge::ComponentManager>(executor);
    RCLCPP_INFO(node->get_logger(), "Component manager node created");

    domain_bridge.add_to_executor(*executor);
    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
