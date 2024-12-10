#include "interactive_marker_tutorials/basic_controls.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <sstream>

namespace interactive_marker_tutorials
{

    BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("basic_controls", options), menu_handler_()
    {
        // creates a unique pointer for an InteractiveMarkerServer class
        server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
            "basic_controls",
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_topics_interface(),
            get_node_services_interface());

        menu_handler_.insert("First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
        menu_handler_.insert("Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
        interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Submenu");
        menu_handler_.insert(sub_menu_handle, "First Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));
        menu_handler_.insert(sub_menu_handle, "Second Entry", std::bind(&BasicControlsNode::processFeedback, this, std::placeholders::_1));

        frame_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&BasicControlsNode::frameCallback, this));
    }
    void BasicControlsNode::frameCallback()
    {
        static uint32_t counter = 0;

        if (!tf_broadcaster_)
        {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
        }

        // Get current time using rclcpp::Time
        rclcpp::Time time_point = this->get_clock()->now();

        // Create a TransformStamped object directly
        geometry_msgs::msg::TransformStamped transform_msg; // In ROS 2, to use tf2::Stamped with transforms, you typically need to use the geometry_msgs::msg::TransformStamped directly
        transform_msg.header.stamp = time_point;            // Use rclcpp::Time
        transform_msg.header.frame_id = "base_link";
        transform_msg.child_frame_id = "moving_frame";
        transform_msg.transform.translation.x = 0.0;
        transform_msg.transform.translation.y = 0.0;
        transform_msg.transform.translation.z = sin(static_cast<double>(counter) / 140.0) * 2.0;
        transform_msg.transform.rotation.x = 0.0;
        transform_msg.transform.rotation.y = 0.0;
        transform_msg.transform.rotation.z = 0.0;
        transform_msg.transform.rotation.w = 1.0;

        // Send the transform for moving frame
        tf_broadcaster_->sendTransform(transform_msg);

        // Create another transform for rotating frame
        transform_msg.child_frame_id = "rotating_frame";
        transform_msg.transform.translation.x = 0.0;
        transform_msg.transform.translation.y = 0.0;
        transform_msg.transform.translation.z = 0.0;

        // Use tf2 to create a Quaternion for rotation
        tf2::Quaternion quat;
        quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);

        // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
        transform_msg.transform.rotation = tf2::toMsg(quat);
        // Send the transform for rotating frame
        tf_broadcaster_->sendTransform(transform_msg);

        counter++;
    }

    void BasicControlsNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        std::ostringstream ss;
        ss << "Feedback from marker '" << feedback->marker_name << "' "
           << "at pose (x=" << feedback->pose.position.x
           << ", y=" << feedback->pose.position.y
           << ", z=" << feedback->pose.position.z << ")";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    visualization_msgs::msg::Marker BasicControlsNode::makeBox(const visualization_msgs::msg::InteractiveMarker &msg)
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

    visualization_msgs::msg::InteractiveMarkerControl &BasicControlsNode::makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(makeBox(msg));
        msg.controls.push_back(control);
        return msg.controls.back();
    }

} // namespace interactive_marker_tutorials

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_tutorials::BasicControlsNode, rclcpp::Node)
