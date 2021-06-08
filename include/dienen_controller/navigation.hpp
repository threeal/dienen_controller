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

#ifndef DIENEN_CONTROLLER__NAVIGATION_HPP_
#define DIENEN_CONTROLLER__NAVIGATION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <musen/musen.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>  // NOLINT
#include <string>

namespace dienen_controller
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class Navigation : public rclcpp::Node
{
public:
  struct Options : public rclcpp::NodeOptions
  {
    Options();

    std::string node_name;
    std::string target_host;
    int listen_port;
    int broadcast_port;
  };

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
  } __attribute__((packed, aligned(1)));

  explicit Navigation(const Options & options = Options());
  ~Navigation();

  bool connect();
  bool disconnect();

private:
  void listen_process();
  void broadcast_process();

  rclcpp::Subscription<Twist>::SharedPtr twist_subscription;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;

  rclcpp::TimerBase::SharedPtr update_timer;

  std::optional<double> initial_yaw;

  Pose current_pose;
  Twist current_twist;

  std::shared_ptr<musen::Listener> listener;
  std::shared_ptr<musen::Broadcaster> broadcaster;
};

}  // namespace dienen_controller

#endif  // DIENEN_CONTROLLER__NAVIGATION_HPP_
