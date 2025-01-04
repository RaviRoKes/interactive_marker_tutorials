#include "interactive_marker_tutorials/basic_controls.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "rviz_common/display_context.hpp"
#include "rclcpp/rclcpp.hpp"
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
    // Initialize the button and connect the signal
    button_ = new QPushButton("Create Marker", this);
    connect(button_, &QPushButton::clicked, this, &BasicControlsPanel::onButtonClicked);
    frame_name_input_ = new QLineEdit(this); // Initialize the input fields for frame names
    frame_name_input_->setPlaceholderText("Enter Frame Name");
    parent_frame_name_input_ = new QLineEdit(this);
    parent_frame_name_input_->setPlaceholderText("Enter Parent Frame Name");

    // Initialize the "Publish Frame" button
    publish_frame_button_ = new QPushButton("Publish Frame", this);
    connect(publish_frame_button_, &QPushButton::clicked, this, &BasicControlsPanel::onPublishFrameClicked);

    // Set layout for the panel
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(button_);
    layout->addWidget(frame_name_input_);
    layout->addWidget(parent_frame_name_input_);
    layout->addWidget(publish_frame_button_);
    setLayout(layout);
    // Create and link the node for controlling markers
    basic_controls_node_ = std::make_shared<BasicControlsNode>();
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Node initialized successfully.");
    }
  }
  void BasicControlsPanel::onButtonClicked()
  {
    // Ensure that the node is set and call the marker creation method
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Button clicked: Creating 1x1 Grid of Markers");
      basic_controls_node_->createGridOfBoxes();
      // basic_controls_node_->make6DofMarker(true, 0, tf2::Vector3(0.0, 0.0, 0.0), true); // b1

      // Thread-safe UI update (if needed)
      QMetaObject::invokeMethod(this, [this]()
                                { button_->setText("Markers Created"); });
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode is not set.");
    }
    basic_controls_node_->applyChanges();
  }
  void BasicControlsPanel::onPublishFrameClicked()
  {
    // Ensure that the node is set and get the frame names from input fields
    if (basic_controls_node_)
    {
      std::string frame_name = frame_name_input_->text().toStdString();
      std::string parent_frame_name = parent_frame_name_input_->text().toStdString();

      if (frame_name.empty() || parent_frame_name.empty())
      {
        RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "Both frame names must be entered.");
        return; // Exit if input fields are empty
      }
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Publishing transformation between '%s' and '%s'.", frame_name.c_str(), parent_frame_name.c_str());
      basic_controls_node_->publishFrameTransformation(frame_name, parent_frame_name);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode is not set.");
    }
  }

  BasicControlsPanel::~BasicControlsPanel()
  {
    // Optional: Add any custom cleanup code if needed
  }
  void BasicControlsPanel::setBasicControlsNode(BasicControlsNode *node)
  {
    basic_controls_node_ = std::shared_ptr<BasicControlsNode>(node);
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode successfully linked to BasicControlsPanel.");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "Failed to link BasicControlsNode to BasicControlsPanel.");
    }
  }
  BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("basic_controls", options)
  {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "basic_controls",
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_topics_interface(),
        get_node_services_interface());

    RCLCPP_DEBUG(get_logger(), "InteractiveMarkerServer initialized successfully."); //

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subscribe to feedback topic and listen to IMF message type
    feedback_subscription_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        "basic_controls/feedback", 10,
        std::bind(&BasicControlsNode::processBoxClick, this, std::placeholders::_1));
  }

  void BasicControlsNode::publishFrameTransformation(const std::string &frame_id, const std::string &parent_frame_id)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = rclcpp::Clock().now();
    transformStamped.header.frame_id = parent_frame_id;
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 1.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // No rotation for simplicity
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transformStamped);
  }
  void BasicControlsNode::createGridOfBoxes()
  {
    const int rows = 1;
    const int cols = 1;
    const double spacing = 2.0;
    int marker_index = 0; // Add an index to ensure unique names even for identical positions

    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols; ++j)
      {
        tf2::Vector3 position(i * spacing, j * spacing, 0);
        std::string marker_name = "box_marker_" + std::to_string(marker_index++);
        createBoxMarker(position, marker_name);
      }
    }
    server_->applyChanges();
    RCLCPP_INFO(rclcpp::get_logger("BasicControlsNode"), "Markers applied to server.");
  }
  void BasicControlsNode::createBoxMarker(const tf2::Vector3 &position, const std::string &marker_name)
  {
    RCLCPP_DEBUG(get_logger(), "Creating box marker with name: '%s'", marker_name.c_str());

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp = this->get_clock()->now();
    int_marker.name = marker_name;
    int_marker.description = "Interactive Box Marker";
    int_marker.scale = 1.0;
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();

    RCLCPP_DEBUG(get_logger(), "Position set to: x = %.2f, y = %.2f, z = %.2f", position.x(), position.y(), position.z());
    // Add a simple box visual for the button
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 1.5;
    box_marker.scale.y = 1.5;
    box_marker.scale.z = 1.5;
    box_marker.color.r = 0.0f;
    box_marker.color.g = 1.0f;
    box_marker.color.b = 0.0f;
    box_marker.color.a = 1.0f;
    RCLCPP_DEBUG(get_logger(), "Box marker visual created with scale: x = %.2f, y = %.2f, z = %.2f",
                 box_marker.scale.x, box_marker.scale.y, box_marker.scale.z);

    // Add a control for interaction (ensure it's a BUTTON type)
    visualization_msgs::msg::InteractiveMarkerControl button_control;
    button_control.name = "box_control";                                                         // Ensure the control name is unique and relevant
    button_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON; // MoVE_3D , BUTTON
    button_control.always_visible = true;
    RCLCPP_DEBUG(get_logger(), "Setting interaction mode to BUTTON for control: %s", button_control.name.c_str());

    // Create a marker for the box
    button_control.markers.push_back(box_marker);  // Add the box marker to the control
    int_marker.controls.push_back(button_control); // Add the control to the interactive marker

    // Insert the marker into the server
    RCLCPP_INFO(get_logger(), "before InteractiveMarkerServer contains %zu markers.", server_->size());
    server_->insert(int_marker);

    RCLCPP_DEBUG(get_logger(), "Inserting marker '%s' into server...", int_marker.name.c_str());
    visualization_msgs::msg::InteractiveMarker check_marker;
    if (!server_->get(int_marker.name, check_marker))
    {
      RCLCPP_ERROR(get_logger(), "Failed to retrieve marker '%s' after insertion.", int_marker.name.c_str());
    }

    // Bind the callback for the box click
    RCLCPP_DEBUG(get_logger(), "Binding callback to marker: %s", int_marker.name.c_str());
    server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processBoxClick, this, std::placeholders::_1)); // Callback binding
    RCLCPP_DEBUG(get_logger(), "Callback bound to marker: %s", int_marker.name.c_str());

    // Apply the changes to the server
    server_->applyChanges();
    RCLCPP_INFO(get_logger(), "after InteractiveMarkerServer contains %zu markers.", server_->size());
    RCLCPP_DEBUG(get_logger(), "Marker '%s' inserted and changes applied to InteractiveMarkerServer.", int_marker.name.c_str());
  }

  void BasicControlsNode::processBoxClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    RCLCPP_DEBUG(get_logger(), "Received feedback: %s with event type: %d", feedback->marker_name.c_str(), feedback->event_type);

    switch (feedback->event_type)
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      RCLCPP_INFO(get_logger(), "Marker clicked: %s", feedback->marker_name.c_str());
      // Proceed with marker color change
      changeMarkerColor(feedback->marker_name, 1.0f, 0.0f, 0.0f); // Change to red
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      RCLCPP_INFO(get_logger(), "Marker pose updated: %s", feedback->marker_name.c_str());
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      RCLCPP_INFO(get_logger(), "Menu item selected for marker: %s", feedback->marker_name.c_str());
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unexpected event type: %d", feedback->event_type);
      break;
    }
  }
  void BasicControlsNode::changeMarkerColor(const std::string &marker_name, float r, float g, float b)
  {
    RCLCPP_INFO(get_logger(), "InteractiveMarkerServer contains %zu markers.", server_->size());
    visualization_msgs::msg::InteractiveMarker int_marker;
    if (!server_->get(marker_name, int_marker))
    {
      RCLCPP_WARN(get_logger(), "Marker '%s' not found!", marker_name.c_str());
      return; // Exit if the marker isn't found
    }

    for (auto &control : int_marker.controls)
    {
      for (auto &marker : control.markers)
      {
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f; // Ensure the marker remains visible
      }
    }
    server_->insert(int_marker);
    server_->applyChanges();

    RCLCPP_INFO(get_logger(), "Color changed for marker: %s", marker_name.c_str());
  }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_tutorials::BasicControlsPanel, rviz_common::Panel)

// start  tuseday lasssssssfgdfg