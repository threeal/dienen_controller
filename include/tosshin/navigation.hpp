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
  Navigation(
    std::string node_name, const char * server_ip, int server_port,
    const char * client_ip, int client_port
  );

  ~Navigation();

  bool connect();
  bool disconnect();

private:
  Maneuver configure_maneuver(const Maneuver & maneuver);

  void receive_process();
  void send_process();

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

  int sockfd;

  struct sockaddr_in server_addr;
  struct sockaddr_in client_addr;
};

}  // namespace tosshin

#endif  // TOSSHIN__NAVIGATION_HPP__
