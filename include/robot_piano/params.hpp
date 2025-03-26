#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/header.hpp>

// This is the kinematic result of the left-joint angles
// [-28, 88, 45, 80, -45, 82] in degrees.
const auto left_init_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.31887877665212355;
    p.position.y = 0.10894773731092511;
    p.position.z = 1.122253189916326;
    p.orientation.x = 0.13249437284716514;
    p.orientation.y = -0.1291909894165388;
    p.orientation.z = -0.690468511102102;
    p.orientation.w = 0.6992911872702663;
    return p;
}();

// This is the kinematic result of the right-joint angles
// [28, 88, -45, -80, -45, -82] in degrees.
const auto right_init_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3178071837760669;
    p.position.y = -0.10556869093811046;
    p.position.z = 1.1215113837188624;
    p.orientation.x = -0.1345047736672149;
    p.orientation.y = -0.1247301629367219;
    p.orientation.z = 0.6927253372844873;
    p.orientation.w = 0.697482945596954;
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
