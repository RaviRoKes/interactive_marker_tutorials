#include "interactive_marker_tutorials/basic_controls.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QApplication> // Include QApplication
#include <QPushButton>

int main(int argc, char **argv)
{
    // Initialize Qt Application first
    QApplication app(argc, argv); // This is required for creating Qt widgets

    rclcpp::init(argc, argv);

    // Create a shared pointer to the BasicControlsNode, which manages markers and transforms
    auto node = std::make_shared<interactive_marker_tutorials::BasicControlsNode>(rclcpp::NodeOptions());

    // Create the RViz panel for UI components like buttons
    auto panel = std::make_shared<interactive_marker_tutorials::BasicControlsPanel>();

    // Set the node in the panel to link the ROS 2 node with UI functionality
    panel->setBasicControlsNode(node.get());

    RCLCPP_INFO(rclcpp::get_logger("Test"), "Node and Panel linkage established.");

    // Run the ROS 2 node in a separate thread to allow simultaneous Qt execution
    std::thread ros_thread([&]()
                           {
       rclcpp::spin(node);
       rclcpp::shutdown(); });

    // Start the Qt event loop
    int qt_result = app.exec();

    // Join the ROS thread after Qt exits
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }

    return qt_result;
}
