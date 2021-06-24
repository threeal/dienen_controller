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
  options(options),
  current_pos(keisan::Point2::zero()),
  current_yaw(keisan::make_degree(0.0))
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
        // Orientation received as a yaw in degree
        current_yaw = keisan::make_degree(stod(message[2])).normalize();

        // Shift yaw from the initial yaw if enabled
        if (!options.no_reset_odometry) {
          if (!initial_yaw.has_value()) {
            initial_yaw = std::make_optional(current_yaw);
          }

          current_yaw = initial_yaw.value().difference_to(current_yaw);
        }
      }

      // Obtains position
      {
        if (options.position_from_twist) {
          auto velocity = keisan::Point2(current_twist.linear.x, current_twist.linear.y);
          current_pos += velocity.scale(0.01).rotate(current_yaw);
        } else {
          // Position received as y, x in centimetre
          current_pos.y = stod(message[0]) * 0.01;
          current_pos.x = stod(message[1]) * 0.01;
        }

        // Shift pos from the initial pos if enabled
        if (!options.no_reset_odometry) {
          if (!initial_pos.has_value()) {
            initial_pos = std::make_optional(current_pos);
          }

          current_pos = current_pos.translate(-initial_pos.value()).rotate(-initial_yaw.value());
        }
      }

      // Publish odometry
      {
        Odometry odometry;

        odometry.header.stamp = now();
        odometry.header.frame_id = "odom";

        odometry.pose.pose.position.x = current_pos.x;
        odometry.pose.pose.position.y = current_pos.y;
        odometry.pose.pose.position.z = 0.0;

        auto quaternion = keisan::EulerAngles(
          keisan::make_degree(0.0), keisan::make_degree(0.0), current_yaw).quaternion();

        odometry.pose.pose.orientation.x = quaternion.x;
        odometry.pose.pose.orientation.y = quaternion.y;
        odometry.pose.pose.orientation.z = quaternion.z;
        odometry.pose.pose.orientation.w = quaternion.w;

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

  message.left_maneuver = -current_twist.linear.y * 70.0;
  message.forward_maneuver = current_twist.linear.x * 70.0;
  message.yaw_maneuver = current_twist.angular.z * 13.5;

  message.yaw_offset = 0;
  message.x_offset = 0;
  message.y_offset = 0;

  broadcaster->send(message);
}

}  // namespace dienen_controller
