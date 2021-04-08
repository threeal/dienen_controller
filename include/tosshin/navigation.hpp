// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TOSSHIN__NAVIGATION_HPP_
#define TOSSHIN__NAVIGATION_HPP_

#include <housou/housou.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/msg/maneuver.hpp>
#include <tosshin_interfaces/msg/orientation.hpp>
#include <tosshin_interfaces/msg/position.hpp>
#include <tosshin_interfaces/srv/configure_maneuver.hpp>

#include <arpa/inet.h>

#include <memory>
#include <string>

namespace tosshin
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Orientation = tosshin_interfaces::msg::Orientation;
using Position = tosshin_interfaces::msg::Position;
using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;

class Navigation : public rclcpp::Node
{
public:
  struct BroadcastMessage
  {
    char headers[3] = {'i', 't', 's'};

    int16_t left_maneuver;
    int16_t forward_maneuver;
    int16_t yaw_maneuver;

    int16_t unused = 0;

    float yaw_offset;
    float y_offset;
    float x_offset;
  };

  Navigation(std::string node_name, int listener_port, int broadcaster_port);
  ~Navigation();

  bool connect();
  bool disconnect();

private:
  Maneuver configure_maneuver(const Maneuver & maneuver);

  void listen_process();
  void broadcast_process();

  rclcpp::Publisher<Position>::SharedPtr position_publisher;
  rclcpp::Publisher<Orientation>::SharedPtr orientation_publisher;

  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_event_publisher;
  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_input_subscription;

  rclcpp::Service<ConfigureManeuver>::SharedPtr configure_maneuver_service;

  rclcpp::TimerBase::SharedPtr update_timer;

  int calibrate_counter;

  std::shared_ptr<double> yaw_orientation_offset;
  std::shared_ptr<double> x_position_offset;
  std::shared_ptr<double> y_position_offset;

  double forward_maneuver;
  double left_maneuver;
  double yaw_maneuver;

  std::shared_ptr<housou::StringListener> listener;
  std::shared_ptr<housou::Broadcaster<BroadcastMessage>> broadcaster;
};

}  // namespace tosshin

#endif  // TOSSHIN__NAVIGATION_HPP__
