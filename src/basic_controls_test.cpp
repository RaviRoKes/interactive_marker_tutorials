// Integration between the ROS 2 node (BasicControlsNode) and the RViz panel (BasicControlsPanel
#include "interactive_marker_tutorials/basic_controls.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QApplication> // Include QApplication
#include <QPushButton>

int main(int argc, char **argv)
{
    // Initialize Qt Application first
    QApplication app(argc, argv); // This is required for creating Qt widgets

    rclcpp::init(argc, argv);

    // creates a shared pointer to the BasicControlsNode, which is the ROS 2 node responsible for managing interactive markers and broadcasting transforms.
    auto node = std::make_shared<interactive_marker_tutorials::BasicControlsNode>(rclcpp::NodeOptions());

    // creates a shared pointer to the BasicControlsPanel (the RViz panel containing the UI components like buttons).
    auto panel = std::make_shared<interactive_marker_tutorials::BasicControlsPanel>();

    // sets the node in the panel, linking the ROS 2 node to the panel so the button in the UI can trigger ROS-based functionality like marker creation.
    panel->setBasicControlsNode(node.get());

    RCLCPP_INFO(rclcpp::get_logger("Test"), "Node and Panel linkage established.");

    rclcpp::spin(node); // This will keep the ROS 2 node running

    rclcpp::shutdown();
    return app.exec(); // Start Qt event loop after ROS 2 node is shut down
}
