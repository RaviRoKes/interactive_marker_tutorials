#include "interactive_marker_tutorials/basic_controls.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>  // Include QApplication
#include <QPushButton>

int main(int argc, char **argv)
{
    // Initialize Qt Application first
    QApplication app(argc, argv);  // This is required for creating Qt widgets

    rclcpp::init(argc, argv);

    auto node = std::make_shared<interactive_marker_tutorials::BasicControlsNode>(rclcpp::NodeOptions());
    auto panel = std::make_shared<interactive_marker_tutorials::BasicControlsPanel>();
    panel->setBasicControlsNode(node.get());

    RCLCPP_INFO(rclcpp::get_logger("Test"), "Node and Panel linkage established.");

    rclcpp::spin(node);  // This will keep the ROS 2 node running

    rclcpp::shutdown();
    return app.exec();  // Start Qt event loop after ROS 2 node is shut down
}
