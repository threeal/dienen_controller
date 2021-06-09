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

#include <dienen_controller/navigation.hpp>
#include <tf2/utils.h>

#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

namespace dienen_controller
{

using namespace std::chrono_literals;

Navigation::Options::Options()
: node_name("navigation"),
  target_host("169.254.183.100"),
  listen_port(8888),
  broadcast_port(44444),
  position_from_twist(false)
{
}

Navigation::Navigation(const Options & options)
: rclcpp::Node(options.node_name, options),
  options(options)
{
  // Initialize the velocity subscription
  twist_subscription = create_subscription<Twist>(
    "/cmd_vel", 10,
    [&](const Twist::SharedPtr msg) {
      current_twist = *msg;
    });

  // Initialize the odometry publisher
  odometry_publisher = create_publisher<Odometry>("/odom", 10);

  // Initialize the update timer
  {
    update_timer = create_wall_timer(
      10ms, [this]() {
        listen_process();
        broadcast_process();
      });

    update_timer->cancel();
  }

  // Initialize the listener
  {
    listener = std::make_shared<musen::Listener>(options.listen_port);

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Listener initialized on port " << listener->get_port() << "!");
  }

  // Initialize the broadcaster
  {
    broadcaster = std::make_shared<musen::Broadcaster>(
      options.broadcast_port, listener->get_udp_socket());

    broadcaster->add_target_host(options.target_host);
    broadcaster->enable_broadcast(false);

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Broadcaster initialized on port " << broadcaster->get_port() << "!");
  }
}

Navigation::~Navigation()
{
  disconnect();
}

bool Navigation::connect()
{
  if (!listener->connect()) {
    RCLCPP_ERROR(get_logger(), "Failed to connect the listener!");
    return false;
  }

  update_timer->reset();

  return true;
}

bool Navigation::disconnect()
{
  if (!listener->disconnect()) {
    RCLCPP_ERROR(get_logger(), "Failed to disconnect the listener!");
    return false;
  }

  update_timer->cancel();

  return true;
}

void Navigation::listen_process()
{
  auto message = listener->receive_strings(64, ",");

  if (message.size() > 0) {
    try {
      // Obtains orientation
      {
        // Orientation received as an inverted yaw in degree
        auto yaw = tf2Radians(stod(message[2]));

        // Shift yaw from the initial yaw
        if (initial_yaw.has_value()) {
          yaw -= initial_yaw.value();
        } else {
          initial_yaw = std::make_optional<double>(yaw);
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, tf2NormalizeAngle(yaw));

        tf2::convert(orientation, current_pose.orientation);
      }

      // Obtains position
      {
        if (options.position_from_twist) {
          double yaw, pitch, roll;
          tf2::getEulerYPR(current_pose.orientation, yaw, pitch, roll);

          auto forward = current_twist.linear.x;
          auto left = current_twist.linear.y;

          current_pose.position.x += (forward * cos(yaw) - left * sin(yaw)) / 10000;
          current_pose.position.y += (forward * sin(yaw) + left * cos(yaw)) / 10000;
        } else {
          // Position received as y, x in meter
          current_pose.position.y = stod(message[0]) * 0.01;
          current_pose.position.x = stod(message[1]) * 0.01;
        }
      }

      // Publish odometry
      {
        Odometry odometry;

        odometry.header.stamp = now();
        odometry.header.frame_id = "odom";

        odometry.pose.pose = current_pose;
        odometry.twist.twist = current_twist;

        odometry_publisher->publish(odometry);
      }
    } catch (const std::out_of_range & err) {
      RCLCPP_WARN_STREAM(get_logger(), "Not all values are received! " << err.what());
    }
  }
}

void Navigation::broadcast_process()
{
  BroadcastMessage message;

  message.left_maneuver = -current_twist.linear.y;
  message.forward_maneuver = current_twist.linear.x;
  message.yaw_maneuver = current_twist.angular.z * 13.5;

  message.yaw_offset = 0;
  message.x_offset = 0;
  message.y_offset = 0;

  broadcaster->send(message);
}

}  // namespace dienen_controller
