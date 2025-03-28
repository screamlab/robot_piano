#pragma once
#include <rclcpp/rclcpp.hpp>

#define RCLCHECK(plan, group)                                                         \
    {                                                                                 \
        if (!plan) {                                                                  \
            RCLCPP_ERROR(left_node->get_logger(), "%s arm: planning failed.", group); \
            rclcpp::shutdown();                                                       \
            return 1;                                                                 \
        }                                                                             \
    }

static inline double deg2rad(const double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(const double rad) { return rad * 180.0 / M_PI; }
