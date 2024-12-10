#include "interactive_marker_tutorials/basic_controls.hpp"
#include <chrono>
#include <sstream>

namespace interactive_marker_tutorials
{

BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("basic_controls", options),
  menu_handler_()
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "basic_controls",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());

  // Defines menu entries and a submenu for interacting with the marker. Each menu item is associated with a callback function to process feedback when selected.
  menu_handler_.insert("First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
  menu_handler_.insert("Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Submenu");
  menu_handler_.insert(sub_menu_handle, "First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
  menu_handler_.insert(sub_menu_handle, "Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

  frame_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&BasicControlsNode::frameCallback, this));
}

// Creates a simple cube marker
visualization_msgs::msg::Marker BasicControlsNode::makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

// Adds the cube marker as part of the interactive marker's control
visualization_msgs::msg::InteractiveMarkerControl & BasicControlsNode::makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

// This function is called periodically (every 10 ms), and it broadcasts two transforms: one for a moving frame and one for a rotating frame
void BasicControlsNode::frameCallback()
{
  static uint32_t counter = 0;

  if (!tf_broadcaster_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

  tf2::TimePoint tf_time_point(std::chrono::nanoseconds(this->get_clock()->now().nanoseconds()));

  tf2::Stamped<tf2::Transform> transform;
  transform.stamp_ = tf_time_point;
  transform.frame_id_ = "base_link";
  transform.setOrigin(tf2::Vector3(0.0, 0.0, sin(static_cast<double>(counter) / 140.0) * 2.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "moving_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);
  transform.setRotation(quat);
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "rotating_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  counter++;
}

// Handles various types of feedback from the user, including button clicks and menu item selections
void BasicControlsNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  std::ostringstream oss;
  oss << "Feedback from marker '" << feedback->marker_name << "' / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x <<
                   ", " << feedback->mouse_point.y <<
                   ", " << feedback->mouse_point.z <<
                   " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      oss << ": button click" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      oss << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      oss << ": pose changed" <<
          "\nposition = " <<
          feedback->pose.position.x <<
          ", " << feedback->pose.position.y <<
          ", " << feedback->pose.position.z <<
          "\norientation = " <<
          feedback->pose.orientation.w <<
          ", " << feedback->pose.orientation.x <<
          ", " << feedback->pose.orientation.y <<
          ", " << feedback->pose.orientation.z <<
          "\nframe: " << feedback->header.frame_id <<
          " time: " << feedback->header.stamp.sec << "sec, " <<
          feedback->header.stamp.nanosec << " nsec";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      oss << ": mouse down" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      oss << ": mouse up" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;
  }

  server_->applyChanges();
}

// This function rounds the marker's position to the nearest 0.5 units (for grid alignment) when the pose is updated.
void BasicControlsNode::alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  std::ostringstream oss;
  oss << feedback->marker_name << ":" <<
      " aligning position = " << pose.position.x << ", " <<
      pose.position.y << ", " <<
      pose.position.z;

  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  server_->setPose(feedback->marker_name, pose);
}

}  // namespace interactive_marker_tutorials

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_tutorials::BasicControlsNode, rclcpp::Node)
