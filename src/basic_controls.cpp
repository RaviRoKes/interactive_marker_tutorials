#include "interactive_marker_tutorials/basic_controls.hpp"
#include <sstream>

namespace interactive_marker_tutorials
{
  // Constructor for BasicControlsPanel
  BasicControlsPanel::BasicControlsPanel(QWidget *parent)
      : rviz_common::Panel(parent), button_(nullptr), basic_controls_node_(nullptr)
  {
    initializePanel();
  }

  void BasicControlsPanel::initializePanel()
  {
    button_ = new QPushButton("Create Marker", this);
    connect(button_, &QPushButton::clicked, this, &BasicControlsPanel::onButtonClicked);

    // Create and link the node
    basic_controls_node_ = new BasicControlsNode(rclcpp::NodeOptions());
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Node initialized successfully.");
    }

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(button_);
    setLayout(layout);
  }

  void BasicControlsPanel::onButtonClicked()
  {
    // Ensure that the node is set and call the marker creation method
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Button clicked: Creating 6Dof Marker");
      basic_controls_node_->make6DofMarker(true, 0, tf2::Vector3(0.0, 0.0, 0.0), true);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode is not set.");
    }
  }

  void BasicControlsPanel::setBasicControlsNode(BasicControlsNode *node)
  {
    basic_controls_node_ = node;
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode successfully linked to BasicControlsPanel.");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "Failed to link BasicControlsNode to BasicControlsPanel.");
    }
  }

  // Destructor definition for BasicControlsPanel
  BasicControlsPanel::~BasicControlsPanel()
  {
    // Optional: Add any custom cleanup code if needed (e.g., deleting dynamic resources)
  }

  // Constructor for BasicControlsNode
  BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("basic_controls", options), menu_handler_()
  {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "basic_controls",
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_topics_interface(),
        get_node_services_interface());

    // Setup the menu and menu items for interaction
    menu_handler_.insert("First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
    menu_handler_.insert("Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Submenu");
    menu_handler_.insert(sub_menu_handle, "First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
    menu_handler_.insert(sub_menu_handle, "Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

    // Timer for frame broadcasting
    frame_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&BasicControlsNode::frameCallback, this));
  }

  // Method to create a box marker
  visualization_msgs::msg::Marker BasicControlsNode::makeBox(const visualization_msgs::msg::InteractiveMarker &msg)
  {
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE; // Type set to CUBE
    marker.scale.x = msg.scale * 0.45;                   // Scale factor for the box
    marker.scale.y = msg.scale * 0.45;                   // Scale factor for the box
    marker.scale.z = msg.scale * 0.45;                   // Scale factor for the box
    marker.color.r = 0.5;                                // Red component of the color
    marker.color.g = 0.5;                                // Green component of the color
    marker.color.b = 0.5;                                // Blue component of the color
    marker.color.a = 1.0;                                // Alpha (opacity) of the color

    return marker;
  }

  // Method to create a box control for an interactive marker
  visualization_msgs::msg::InteractiveMarkerControl &BasicControlsNode::makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;           // The control is always visible
    control.markers.push_back(makeBox(msg)); // Add the box marker to the control
    msg.controls.push_back(control);         // Add the control to the interactive marker's controls

    return msg.controls.back(); // Return the last control added
  }

  void BasicControlsNode::make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof)
  {
    // Suppress warnings for unused parameters
    (void)fixed;
    (void)show_6dof;

    // Define an interactive marker
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link"; // Adjust frame as needed
    int_marker.header.stamp = this->get_clock()->now();
    int_marker.name = "6dof_marker";
    int_marker.description = "6-DOF Control Marker";
    int_marker.scale = 1.0; // Adjust marker scale
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();

    // Add box control to the interactive marker
    makeBoxControl(int_marker); // Adding the box control to the marker
    // Define a control for moving and rotating along axes

    visualization_msgs::msg::InteractiveMarkerControl control;

    if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D)
    {
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
      control.name = "move_3d";
      int_marker.controls.push_back(control);
    }
    else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D)
    {
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
      control.name = "rotate_3d";
      int_marker.controls.push_back(control);
    }
    else
    {
      // Add 6-DOF controls (translation + rotation on all axes)
      std::vector<std::string> axes = {"x", "y", "z"};
      for (const std::string &axis : axes)
      {
        control.orientation.w = 1.0;
        if (axis == "x")
        {
          control.orientation.x = 1.0;
          control.orientation.y = 0.0;
          control.orientation.z = 0.0;
          control.name = "rotate_x";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_x";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
        else if (axis == "y")
        {
          control.orientation.x = 0.0;
          control.orientation.y = 1.0;
          control.orientation.z = 0.0;
          control.name = "rotate_y";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_y";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
        else if (axis == "z")
        {
          control.orientation.x = 0.0;
          control.orientation.y = 0.0;
          control.orientation.z = 1.0;
          control.name = "rotate_z";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_z";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
      }
    }

    // Add the marker to the server
    server_->insert(int_marker);
    // Marker callback
    server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

    // Apply the changes to the marker server
    server_->applyChanges();

    RCLCPP_INFO(get_logger(), "6Dof Marker created successfully.");
  }
  void BasicControlsNode::frameCallback()
  {
    static uint32_t counter = 0;

    if (!tf_broadcaster_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    }

    geometry_msgs::msg::TransformStamped transform_msg;

    // First transform: "base_link" -> "moving_frame"
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "base_link";
    transform_msg.child_frame_id = "moving_frame";
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = sin(static_cast<double>(counter) / 140.0) * 2.0;
    transform_msg.transform.rotation.w = 1.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;

    RCLCPP_INFO(get_logger(), "Broadcasting TF: %s -> %s",
                transform_msg.header.frame_id.c_str(), transform_msg.child_frame_id.c_str());

    tf_broadcaster_->sendTransform(transform_msg);

    // Second transform: "base_link" -> "rotating_frame"
    tf2::Quaternion quat;
    quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "base_link";
    transform_msg.child_frame_id = "rotating_frame";
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = tf2::toMsg(quat);

    tf_broadcaster_->sendTransform(transform_msg);

    counter++;
  }

  void BasicControlsNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    std::ostringstream oss;
    oss << "Feedback from marker '" << feedback->marker_name << "' / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid)
    {
      mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", " << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type)
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      oss << ": button click" << mouse_point_ss.str() << ".";
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      oss << ": menu item " << feedback->menu_entry_id << " selected" << mouse_point_ss.str() << ".";
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      oss << ": pose changed"
          << " position " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z
          << " orientation " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << ", " << feedback->pose.orientation.w;
      break;
    }

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_tutorials::BasicControlsPanel, rviz_common::Panel)