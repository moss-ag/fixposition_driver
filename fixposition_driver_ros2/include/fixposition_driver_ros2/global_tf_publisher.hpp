#pragma once

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "fixposition_driver_ros2/msg/vrtk.hpp"

namespace fixposition {

class GlobalTFPublisher {
public:
    GlobalTFPublisher(std::shared_ptr<rclcpp::Node> node);

    void OdomENU0Callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void VRTKCallback(const fixposition_driver_ros2::msg::VRTK::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    rclcpp::CallbackGroup::SharedPtr odom_sub_group_, tf_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_enu0_sub_;
    rclcpp::Subscription<fixposition_driver_ros2::msg::VRTK>::SharedPtr vrtk_sub_;

    std::atomic<bool> fusion_running_ = {false};
    double tf_delay_ = 0.2;
};

} // namespace fixposition