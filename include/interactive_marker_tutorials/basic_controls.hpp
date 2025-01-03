#ifndef BASIC_CONTROLS_HPP
#define BASIC_CONTROLS_HPP

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <rviz_common/panel.hpp>

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rviz_common/panel.hpp" // Include for RViz2 panel
#include <QPushButton>           // Include for UI controls in the panel
#include <QVBoxLayout>           // Include for layout management
#include <QLineEdit>             // Include for text input fields

namespace interactive_marker_tutorials
{
  // Forward declaration of BasicControlsNode
  class BasicControlsNode;

  // RViz Panel for Basic Controls
  class BasicControlsPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    BasicControlsPanel(QWidget *parent = nullptr);
    ~BasicControlsPanel();

    void setBasicControlsNode(BasicControlsNode *node);

  protected:
    void initializePanel();

  private slots:
    void onButtonClicked();
    void onPublishFrameClicked(); // Trigger publishing the transformation

  private:
    QPushButton *button_;                                    // Button to interact with the markers
    QPushButton *publish_frame_button_;                      // Button to publish frame transform
    QLineEdit *frame_name_input_;                            // Text input for frame name
    QLineEdit *parent_frame_name_input_;                     // Text input for parent frame name
    std::shared_ptr<BasicControlsNode> basic_controls_node_; // Pointer to the node to trigger marker creation
  };

  // ROS Node for Interactive Marker Control
  class BasicControlsNode : public rclcpp::Node
  {
  public:
    explicit BasicControlsNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~BasicControlsNode() = default;

    inline void applyChanges()
    {
      server_->applyChanges();
    }
    // Marker creation functions
    void createGridOfBoxes();
    void createBoxMarker(const tf2::Vector3 &position, const std::string &marker_name);

    void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof);

    // New method to publish frame transformation
    void publishFrameTransformation(const std::string &frame_id, const std::string &parent_frame_id);         // This method periodically broadcasts transform data for moving and rotating frames
    void processBoxClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback); // Processes box marker clicks
    void handleMenuSelect(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void changeMarkerColor(const std::string &marker_name, float r, float g, float b);

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;                                      // A unique pointer to an InteractiveMarkerServer
    interactive_markers::MenuHandler menu_handler_;                                                             // Instance of MenuHandler for marker context menu interactions
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                                             // TransformBroadcaster for sending transform updates
    rclcpp::TimerBase::SharedPtr frame_timer_;                                                                  // Timer for periodic updates
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_subscription_; // Add this line
  };

} // namespace interactive_marker_tutorials
#endif // BASIC_CONTROLS_HPP

// skjdk gfhfg fgdef