/**
 *  @file
 *  @brief Main function for the fixposition driver ros node
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* SYSTEM / STL */
#include <memory>

/* ROS */
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/global_tf_publisher.hpp>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> tf_node = rclcpp::Node::make_shared("global_tf_publisher");

    RCLCPP_INFO(tf_node->get_logger(), "Starting global_tf_publisher node...");

    fixposition::GlobalTFPublisher tf_publisher(tf_node);
    rclcpp::spin(tf_node);
    rclcpp::shutdown();
    return  0;
}
