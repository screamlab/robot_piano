#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/header.hpp>

// This is the kinematic result of the left-joint angles
// [-28, 88, 45, 84, -45, 82] in degrees.
const auto left_init_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.31210279264757873;
    p.position.y = 0.10673761228487201;
    p.position.z = 1.1387751632083898;
    p.orientation.x = 0.13317917261291404;
    p.orientation.y = -0.16262911859128856;
    p.orientation.z = -0.6926819903446356;
    p.orientation.w = 0.6899324155455767;
    return p;
}();

// This is the kinematic result of the left-joint angles
// [-24, 22, -7, 72, 16, 29] in degrees.
const auto left_mid_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.27184990188146685;
    p.position.y = 0.6006422048308747;
    p.position.z = 1.3132380612684487;
    p.orientation.x = 0.053122085898056615;
    p.orientation.y = -0.047791463201040406;
    p.orientation.z = -0.6255856928897661;
    p.orientation.w = 0.7768761554370004;
    return p;
}();

// This is the kinematic result of the right-joint angles
// [28, 88, -45, -84, -45, -82] in degrees.
const auto right_init_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.31123710024038204;
    p.position.y = -0.10892418433260642;
    p.position.z = 1.1355713114965817;
    p.orientation.x = -0.13275623094736622;
    p.orientation.y = -0.15898911456916212;
    p.orientation.z = 0.6900454979187236;
    p.orientation.w = 0.6934951012049503;
    return p;
}();

const auto piano_object = []() {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "";
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
    return collision_object;
}();
