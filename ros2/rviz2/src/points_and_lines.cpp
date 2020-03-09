#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <memory>

#include <cmath>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;

class MovingCurvePublisher : public rclcpp::Node {
public:
  MovingCurvePublisher() : Node("points_and_lines") {
    _publisher = this->create_publisher<Marker>("visualization_marker", 1);
    init_marker();
    auto timer_callback = [this]() -> void {
      auto now = this->now();
      this->_points    .header.stamp = now;
      this->_line_strip.header.stamp = now;
      this->_line_list .header.stamp = now;

      this->populate_curve(this->f);
      this->f += 0.04;

      this->_publisher->publish(this->_points);
      this->_publisher->publish(this->_line_strip);
      this->_publisher->publish(this->_line_list);
    };
    _timer = this->create_wall_timer(33ms, timer_callback); // 30 hz
  }

private:
  void init_marker() {
    _points     = Marker();
    _line_strip = Marker();
    _line_list  = Marker();

    _points    .header.frame_id = "/my_frame";
    _line_strip.header.frame_id = "/my_frame";
    _line_list .header.frame_id = "/my_frame";

    _points    .ns = "points_and_lines";
    _line_strip.ns = "points_and_lines";
    _line_list .ns = "points_and_lines";

    _points    .id = 0;
    _line_strip.id = 1;
    _line_list .id = 2;

    _points    .type = Marker::POINTS;
    _line_strip.type = Marker::LINE_STRIP;
    _line_list .type = Marker::LINE_LIST;

    _points    .action = Marker::ADD;
    _line_strip.action = Marker::ADD;
    _line_list .action = Marker::ADD;

    // POINTS markers use x and y scale for width/height respectively
    _points.scale.x = 0.2;
    _points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers only use the x-component of scale, for the
    // line width
    _line_strip.scale.x = 0.1;
    _line_list .scale.x = 0.1;

    // Points are green
    _points.color.g = 1.0;
    _points.color.a = 1.0;

    // Line strip is blue
    _line_strip.color.b = 1.0;
    _line_strip.color.a = 1.0;

    // Line list is red
    _line_list.color.r = 1.0;
    _line_list.color.a = 1.0;
  }

  void populate_curve(float f) {
    _points    .points.clear();
    _line_strip.points.clear();
    _line_list .points.clear();
    for (uint32_t i = 0; i < 100; i++) {
      Point p;
      p.x = float(i) - 50.0f;
      p.y = 5.0f * std::sin(f + i / 100.0f * 2.0f * M_PI);
      p.z = 5.0f * std::cos(f + i / 100.0f * 2.0f * M_PI);

      _points    .points.push_back(p);
      _line_strip.points.push_back(p);

      // The line list needs two points for each line segment
      _line_list .points.push_back(p);
      p.z += 1.0;
      _line_list .points.push_back(p);
    }
  }

private:
  rclcpp::Publisher<Marker>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  Marker _points;
  Marker _line_strip;
  Marker _line_list;
  float f = 0.0f;
};

int main(int argCount, char* argList[]) {
  rclcpp::init(argCount, argList);
  rclcpp::spin(std::make_shared<MovingCurvePublisher>());
  rclcpp::shutdown();
  return 0;
}
