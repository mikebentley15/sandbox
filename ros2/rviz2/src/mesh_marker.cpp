#include "Mesh.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <memory>

#include <cmath>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
using builtin_interfaces::msg::Duration;

class MeshPublisher : public rclcpp::Node {
public:
  MeshPublisher(const std::string &filepath)
    : Node("mesh_publisher")
  {
    _publisher = this->create_publisher<Marker>("visualization_marker", 5);
    std::cout << "Loading file '" << filepath << "'\n";
    _mesh = Mesh::from_stl(filepath);
    init_marker();
    auto timer_callback = [this]() -> void {
      _marker.header.stamp = this->now();
      //std::cout << "calling publish\n";
      _publisher->publish(_marker);
    };
    _timer = this->create_wall_timer(1s, timer_callback);
  }

private:
  void init_marker() {
    _marker = _mesh.to_vis_marker();
    if (_marker.type != Marker::MESH_RESOURCE) {
      std::cerr << "ERROR: to_vis_marker() did not make a TRIANGLE_LIST\n";
    }
    //_marker = Marker();
    //_marker.header.frame_id = "/map";
    //_marker.ns = "mesh_markers";
    //_marker.id = 0;
    //_marker.type = Marker::MESH_RESOURCE;
    //_marker.action = Marker::ADD;
    //_marker.mesh_resource = "file:///home/bentley/ros_ws/src/ll4ma_scene_generator/data/meshes/mustard.stl";
    //_marker.scale.x = 1.0;
    //_marker.scale.y = 1.0;
    //_marker.scale.z = 1.0;
    //_marker.color.r = 1.0f;
    //_marker.color.g = 0.0f;
    //_marker.color.b = 0.0f;
    //_marker.color.a = 0.4f;
  }

private:
  rclcpp::Publisher<Marker>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  Marker _marker;
  Mesh _mesh;
};

int main(int argCount, char* argList[]) {
  std::string usage = "Usage: " + std::string(argList[0]) +
                      " [mesh-filepath]\n";
  rclcpp::init(argCount, argList);

  std::string filepath;
  if (argCount < 2) {
    filepath = "/home/bentley/ros_ws/src/ll4ma_scene_generator/data/meshes/"
               "mustard.stl";
  } else if (argCount == 2) {
    std::string arg(argList[1]);
    if (arg == "-h" || arg == "--help") {
      std::cout << usage;
      return 0;
    }
    filepath = std::string(argList[1]);
  } else {
    std::cerr << usage;
    return 1;
  }

  rclcpp::spin(std::make_shared<MeshPublisher>(filepath));
  rclcpp::shutdown();
  return 0;
}
