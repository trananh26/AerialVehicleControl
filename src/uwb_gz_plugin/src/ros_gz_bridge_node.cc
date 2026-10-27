// Standalone gz-transport -> ROS 2 bridge for the uwb_gz_plugin.
// Lives in uwb_gz_plugin so we can reuse the same build env.
//
// This node subscribes to the gz topics published by UwbSystem, decodes
// the custom Pose encoding, and republishes a normal
// geometry_msgs::msg::Pose on a ROS 2 topic. We need it because
// ros_gz_bridge 0.244 in this workspace has a factory-lookup failure
// when bridging gz.msgs.Pose -> geometry_msgs/msg/Pose.

#include <geometry_msgs/msg/pose.hpp>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class UwbGzToRosBridge : public rclcpp::Node
{
public:
  gz::transport::Node gz_node_;
  UwbGzToRosBridge()
  : rclcpp::Node("uwb_gz_to_ros_bridge")
  {
    declare_parameter("tag_id", std::string("0"));
    declare_parameter("gz_ranging_topic", std::string(""));
    declare_parameter("gz_tag_pose_topic", std::string(""));
    declare_parameter("ros_ranging_topic", std::string("/uwb/ranging_ros"));
    declare_parameter("ros_tag_pose_topic", std::string("/uwb/tag_pose_ros"));

    const std::string tag_id = get_parameter("tag_id").as_string();
    const std::string default_ranging = "/uwb/ranging/tag";
    const std::string default_tag_pose = "/uwb/tag_pose/tag";

    const std::string gz_ranging = get_parameter("gz_ranging_topic").as_string().empty()
      ? default_ranging
      : get_parameter("gz_ranging_topic").as_string();
    const std::string gz_tag_pose = get_parameter("gz_tag_pose_topic").as_string().empty()
      ? default_tag_pose
      : get_parameter("gz_tag_pose_topic").as_string();

    const std::string ros_ranging = get_parameter("ros_ranging_topic").as_string();
    const std::string ros_tag_pose = get_parameter("ros_tag_pose_topic").as_string();

    // Use sensor-friendly QoS (best-effort, volatile) so subscribers using
    // rclpy defaults or sensor_data profiles will not get dropped with
    // "incompatible QoS" warnings.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .best_effort()
                 .durability_volatile();
    pub_ranging_ = create_publisher<geometry_msgs::msg::Pose>(ros_ranging, qos);
    pub_tag_pose_ = create_publisher<geometry_msgs::msg::Pose>(ros_tag_pose, qos);

    if (!gz_node_.Subscribe(gz_ranging, &UwbGzToRosBridge::OnRanging, this)) {
      RCLCPP_ERROR(get_logger(),
        "Failed to subscribe to gz topic: %s", gz_ranging.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
        "Subscribed: gz=%s -> ros=%s",
        gz_ranging.c_str(), ros_ranging.c_str());
    }
    if (!gz_node_.Subscribe(gz_tag_pose, &UwbGzToRosBridge::OnTagPose, this)) {
      RCLCPP_ERROR(get_logger(),
        "Failed to subscribe to gz topic: %s", gz_tag_pose.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
        "Subscribed: gz=%s -> ros=%s",
        gz_tag_pose.c_str(), ros_tag_pose.c_str());
    }
  }

private:
  void OnRanging(const gz::msgs::Pose & msg)
  {
    pub_ranging_->publish(ToRosPose(msg));
  }

  void OnTagPose(const gz::msgs::Pose & msg)
  {
    pub_tag_pose_->publish(ToRosPose(msg));
  }

  static geometry_msgs::msg::Pose ToRosPose(const gz::msgs::Pose & msg)
  {
    geometry_msgs::msg::Pose out;
    out.position.x = msg.position().x();
    out.position.y = msg.position().y();
    out.position.z = msg.position().z();
    out.orientation.x = msg.orientation().x();
    out.orientation.y = msg.orientation().y();
    out.orientation.z = msg.orientation().z();
    out.orientation.w = msg.orientation().w();
    return out;
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_ranging_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_tag_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbGzToRosBridge>();
  // gz-transport spawns its own internal I/O thread when Subscribe() is
  // called, so the callbacks fire asynchronously with rclcpp::spin.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
