cmake_minimum_required(VERSION 3.8)
project(robot_piano)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install pre-commit hook
execute_process(
  COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/ -type l -name "pre-commit" -delete || true
  COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/pre-commit.hook ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/pre-commit
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE HOOK_INSTALL_RESULT
  OUTPUT_QUIET ERROR_QUIET
)

if (HOOK_INSTALL_RESULT)
  message(WARNING "Failed to install pre-commit hook in ${CMAKE_CURRENT_SOURCE_DIR}.")
endif()

if(DEFINED ENV{ROS_DISTRO})
  string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO_UPPER)
  if(ROS_DISTRO_UPPER STREQUAL "FOXY")
    add_definitions(-DROS_DISTRO_FOXY)
  elseif(ROS_DISTRO_UPPER STREQUAL "GALACTIC")
    add_definitions(-DROS_DISTRO_GALACTIC)
  elseif(ROS_DISTRO_UPPER STREQUAL "HUMBLE")
    add_definitions(-DROS_DISTRO_HUMBLE)
  elseif(ROS_DISTRO_UPPER STREQUAL "JAZZY")
    add_definitions(-DROS_DISTRO_JAZZY)
  else()
    add_definitions(-DROS_DISTRO_UNKNOWN)
  endif()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(domain_bridge REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(ik_calc src/ik_calc.cpp src/moveit_planner.cpp)
add_executable(left src/left.cpp src/moveit_planner.cpp src/hand_publisher.cpp)
add_executable(right src/right.cpp src/moveit_planner.cpp src/hand_publisher.cpp)
add_executable(trigger src/trigger.cpp)
add_executable(bridge src/bridge.cpp)
target_include_directories(ik_calc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(left PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(right PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ik_calc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(left PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(right PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(trigger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ik_calc
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
  "moveit_msgs"
  "geometry_msgs"
  "shape_msgs"
  "std_msgs"
  "trajectory_msgs"
)
ament_target_dependencies(
  left
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
  "moveit_msgs"
  "geometry_msgs"
  "shape_msgs"
  "std_msgs"
  "trajectory_msgs"
  "std_srvs"
)
ament_target_dependencies(
  right
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
  "moveit_msgs"
  "geometry_msgs"
  "shape_msgs"
  "std_msgs"
  "trajectory_msgs"
  "std_srvs"
)
ament_target_dependencies(
  trigger
  "rclcpp"
  "std_msgs"
  "std_srvs"
)
ament_target_dependencies(
  bridge
  "domain_bridge"
  "rclcpp"
  "std_srvs"
)

install(TARGETS
  ik_calc
  left
  right
  trigger
  bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
