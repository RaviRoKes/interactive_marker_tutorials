#ifndef BASIC_CONTROLS_HPP
#define BASIC_CONTROLS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace interactive_marker_tutorials
{

    class BasicControlsNode : public rclcpp::Node
    {
    public:
        explicit BasicControlsNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~BasicControlsNode() = default;

    private:
        void frameCallback();
        void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
        void alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
        visualization_msgs::msg::Marker makeBox(const visualization_msgs::msg::InteractiveMarker &msg);
        visualization_msgs::msg::InteractiveMarkerControl &makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg);
        void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof);
        void makeRandomDofMarker(const tf2::Vector3 &position);
        void makeViewFacingMarker(const tf2::Vector3 &position);
        void makeQuadrocopterMarker(const tf2::Vector3 &position);
        void makeChessPieceMarker(const tf2::Vector3 &position);
        void makePanTiltMarker(const tf2::Vector3 &position);

        std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
        interactive_markers::MenuHandler menu_handler_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr frame_timer_;
    };

} // namespace interactive_marker_tutorials

#endif // BASIC_CONTROLS_HPP
