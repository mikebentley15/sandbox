#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <chrono>

using namespace std::chrono_literals;

int main(int argCount, char *argList[]) {
  rclcpp::init(argCount, argList);
  std::string base_frame = "map";
  std::string moving_frame = "camera";

  if (argCount > 1 &&
      (argList[1] == std::string("-h") || argList[1] == std::string("--help")))
  {
    std::cout
      << "Usage: " << argList[0] << " [base-frame] [moving-frame]\n"
         "\n"
         "Arguments:\n"
         "  base-frame    base frame name.\n"
         "                (defaults is '" << base_frame << "')\n"
         "  moving-frame  name of the frame moving relative to base-frame\n"
         "                (default is '" << moving_frame << "')\n";
  }

  if (argCount >= 2) { base_frame   = argList[1]; }
  if (argCount >= 3) { moving_frame = argList[2]; }

  auto node = rclcpp::Node::make_shared("moving_camera_publisher");

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped msg;
  geometry_msgs::msg::TransformStamped frame_transform;

  msg.transform.translation.x = 0;
  msg.transform.translation.y = 0;
  msg.transform.translation.z = 0;
  msg.transform.rotation.x = 0;
  msg.transform.rotation.y = 0;
  msg.transform.rotation.z = 0;
  msg.transform.rotation.w = 1;
  msg.header.frame_id = base_frame;
  msg.child_frame_id = moving_frame;

  tf2::Quaternion rotation(tf2::Vector3{0, 0, 1}, 0.02 * M_PI);
  frame_transform.transform.translation.x = 0;
  frame_transform.transform.translation.y = 0;
  frame_transform.transform.translation.z = 0;
  frame_transform.transform.rotation.x = rotation.x();
  frame_transform.transform.rotation.y = rotation.y();
  frame_transform.transform.rotation.z = rotation.z();
  frame_transform.transform.rotation.w = rotation.w();

  auto timer_callback = [&node, &msg, &frame_transform, &broadcaster, &clock]() -> void {
    geometry_msgs::msg::TransformStamped copy = msg;
    tf2::doTransform(copy, msg, frame_transform);
    msg.header.stamp = clock->now();
    msg.header.frame_id = copy.header.frame_id;
    msg.child_frame_id = copy.child_frame_id;
    //RCLCPP_INFO(node->get_logger(), "  Rotation: %f, %f, %f, %f",
    //    msg.transform.rotation.x,
    //    msg.transform.rotation.y,
    //    msg.transform.rotation.z,
    //    msg.transform.rotation.w);
    broadcaster.sendTransform(msg);
  };
  auto timer = node->create_wall_timer(33ms, timer_callback);

  RCLCPP_INFO(node->get_logger(), "Spinning camera, publishing tranform from %s to %s",
      msg.header.frame_id.c_str(), msg.child_frame_id.c_str());

  broadcaster.sendTransform(msg);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
