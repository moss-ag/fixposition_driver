#include "fixposition_driver_ros2/global_tf_publisher.hpp"

#include <chrono>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fixposition
{

GlobalTFPublisher::GlobalTFPublisher(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  br_(std::make_shared<tf2_ros::TransformBroadcaster>(node_))
{

  auto sub_opt = rclcpp::SubscriptionOptions();
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_opt.callback_group = tf_group_;
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_opt.callback_group = odom_sub_group_;
  odom_enu0_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/fixposition/odometry_enu", 10,
    std::bind(&GlobalTFPublisher::OdomENU0Callback, this, std::placeholders::_1),
    sub_opt);

  vrtk_sub_ = node_->create_subscription<fixposition_driver_ros2::msg::VRTK>(
    "/fixposition/vrtk", 10,
    std::bind(&GlobalTFPublisher::VRTKCallback, this, std::placeholders::_1));
}

void GlobalTFPublisher::OdomENU0Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (fusion_running_.load() == false) {
    return;
  }

  tf2::Transform world_base_link_trans, base_link_odom_trans, map_odom_trans;

  tf2::fromMsg(msg->pose.pose, world_base_link_trans);

  try {
    tf2::fromMsg(
      tf_buffer_->lookupTransform(
        "base_link", "odom",
        msg->header.stamp, rclcpp::Duration(std::chrono::duration<double>(0.1))).transform,
      base_link_odom_trans);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform base_link to odom: %s", ex.what());
    return;
  }

  map_odom_trans.mult(world_base_link_trans, base_link_odom_trans);

  geometry_msgs::msg::TransformStamped map_odom_trans_msg;
  map_odom_trans_msg.transform = tf2::toMsg(map_odom_trans);
  map_odom_trans_msg.header.stamp = static_cast<rclcpp::Time>(msg->header.stamp);
  map_odom_trans_msg.header.frame_id = "local_enu";
  map_odom_trans_msg.child_frame_id = "odom";

  br_->sendTransform(map_odom_trans_msg);

}

void GlobalTFPublisher::VRTKCallback(const fixposition_driver_ros2::msg::VRTK::SharedPtr msg) {
  if (msg->fusion_status >= 3) {
    if (fusion_running_.load() == false) {
      RCLCPP_INFO(node_->get_logger(), "Fusion has started");
    }
    fusion_running_.store(true);
  } else {
    fusion_running_.store(false);
  }
}


} // namespace fixposition
