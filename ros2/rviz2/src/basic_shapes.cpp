#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <memory>
#include <type_traits> // for std::remove_cv<>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using Shape = std::remove_const<decltype(Marker::CUBE)>::type;

const char* shape_name(Shape shape) {
  switch(shape) {
    case Marker::CUBE:     return "cube";
    case Marker::SPHERE:   return "sphere";
    case Marker::ARROW:    return "arrow";
    case Marker::CYLINDER: return "cylinder";
  }
  return "UNKNOWN";
}

Shape next_shape(Shape shape) {
  switch(shape) {
    case Marker::CUBE:     return Marker::SPHERE;
    case Marker::SPHERE:   return Marker::ARROW;
    case Marker::ARROW:    return Marker::CYLINDER;
    case Marker::CYLINDER: return Marker::CUBE;
  }
  return Marker::CUBE;
}

class ShapePublisher : public rclcpp::Node {
public:
  ShapePublisher() : Node("basic_shapes") {
    _publisher = this->create_publisher<Marker>("visualization_marker", 1);
    init_marker();
    auto timer_callback = [this]() -> void {
      _marker.header.stamp = this->now();
      _marker.type = next_shape(_marker.type);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
                  shape_name(_marker.type));
      _publisher->publish(_marker);
    };
    _timer = this->create_wall_timer(1s, timer_callback);
  }

private:
  void init_marker() {
    _marker = Marker();
    _marker.header.frame_id = "/my_frame";
    _marker.ns = "basic_shapes";
    _marker.id = 0;
    _marker.type = Marker::CUBE;
    _marker.action = Marker::ADD;
    // _marker.pose.position.x = 0; // these are already the default values
    // _marker.pose.position.y = 0;
    // _marker.pose.position.z = 0;
    // _marker.pose.orientation.x = 0;
    // _marker.pose.orientation.y = 0;
    // _marker.pose.orientation.z = 0;
    // _marker.pose.orientation.w = 1;
    _marker.scale.x = 1.0;
    _marker.scale.y = 1.0;
    _marker.scale.z = 1.0;
    _marker.color.r = 0.0f;
    _marker.color.g = 1.0f;
    _marker.color.b = 0.0f;
    _marker.color.a = 1.0f;
    //_marker.lifetime = ; // what to put here?
  }

private:
  rclcpp::Publisher<Marker>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  Marker _marker;
};

int main( int argc, char** argv )
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShapePublisher>());
  rclcpp::shutdown();
  return 0;
}
