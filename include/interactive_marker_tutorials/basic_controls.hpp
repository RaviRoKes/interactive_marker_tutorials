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

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rviz_common/panel.hpp" // Include for RViz2 panel
#include <QPushButton>           // Include for UI controls in the panel
#include <QVBoxLayout>           // Include for layout management

namespace interactive_marker_tutorials
{

  class BasicControlsNode : public rclcpp::Node
  {
  public:
    explicit BasicControlsNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~BasicControlsNode() = default;

    inline void applyChanges()
    {
      server_->applyChanges();
    }
    // Helper Methods for Marker Control
    visualization_msgs::msg::Marker makeBox(const visualization_msgs::msg::InteractiveMarker &msg);
    visualization_msgs::msg::InteractiveMarkerControl &makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg);

    // Marker creation functions
    void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof);
    void makeRandomDofMarker(const tf2::Vector3 &position);
    void makeViewFacingMarker(const tf2::Vector3 &position);
    void makeQuadrocopterMarker(const tf2::Vector3 &position);
    void makeChessPieceMarker(const tf2::Vector3 &position);
    void makePanTiltMarker(const tf2::Vector3 &position);
    void makeMenuMarker(const tf2::Vector3 &position);
    void makeButtonMarker(const tf2::Vector3 &position);
    void makeMovingMarker(const tf2::Vector3 &position);

    // RViz Panel for Basic Controls
    class BasicControlsPanel : public rviz_common::Panel
    {
      Q_OBJECT
    public:
      BasicControlsPanel(QWidget *parent = nullptr);
      ~BasicControlsPanel();

    protected:
      void initializePanel();

    private:
      QPushButton *button_; // Button to interact with the markers
    };

  private:
    void frameCallback(); // This method periodically broadcasts transform data for moving and rotating frames
    // This method processes the feedback from user interactions with the interactive marker (such as button clicks or pose updates).
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    // This method aligns the marker to a grid when the user updates its pose.
    void alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_; // A unique pointer to an InteractiveMarkerServer, which manages the interactive markers.
    // An instance of MenuHandler for handling the context menu interactions with markers.
    interactive_markers::MenuHandler menu_handler_;
    // A unique pointer to a TransformBroadcaster, used for sending transform updates.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // A shared pointer to a timer used for periodic updates, likely to call frameCallback.
    rclcpp::TimerBase::SharedPtr frame_timer_;
  };

} // namespace interactive_marker_tutorials
#endif // BASIC_CONTROLS_HPP