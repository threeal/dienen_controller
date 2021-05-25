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

#include <keisan/keisan.hpp>

#include <tosshin_dienen_controller/navigation.hpp>

#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

namespace tosshin_dienen_controller
{

using namespace std::chrono_literals;

Navigation::Navigation(
  rclcpp::Node::SharedPtr node, std::string target_host,
  int listen_port, int broadcast_port)
: tosshin_cpp::NavigationProvider(node)
{
  // Initialize the update timer
  {
    update_timer = get_node()->create_wall_timer(
      10ms, [this]() {
        listen_process();
        broadcast_process();
      });

    update_timer->cancel();
  }

  // Initialize the listener
  {
    listener = std::make_shared<musen::StringListener>(listen_port);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Listener initialized on port " << listener->get_port() << "!");
  }

  // Initialize the broadcaster
  {
    using Broadcaster = musen::Broadcaster<BroadcastMessage>;
    broadcaster = std::make_shared<Broadcaster>(broadcast_port, listener->get_udp_socket());

    broadcaster->add_target_host(target_host);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
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
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to connect the listener!");
    return false;
  }

  update_timer->reset();

  return true;
}

bool Navigation::disconnect()
{
  if (!listener->disconnect()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to disconnect the listener!");
    return false;
  }

  update_timer->cancel();

  return true;
}

void Navigation::listen_process()
{
  auto message = listener->receive(64, ",");

  if (message.size() > 0) {
    try {
      tosshin_cpp::Odometry odometry;

      // Position received as y, x in centimetre
      odometry.position.y = stod(message[0]) * 0.01;
      odometry.position.x = stod(message[1]) * 0.01;

      // Orientation received as yaw in unwrapped degree
      odometry.orientation.yaw = keisan::wrap_deg(stod(message[2]));

      set_odometry(odometry);
    } catch (const std::out_of_range & err) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Not all values are received! " << err.what());
    }
  }
}

void Navigation::broadcast_process()
{
  BroadcastMessage message;

  auto maneuver = get_maneuver();

  message.left_maneuver = -maneuver.left / 5;
  message.forward_maneuver = maneuver.forward / 5;
  message.yaw_maneuver = maneuver.yaw / 5;

  message.yaw_offset = 0;
  message.x_offset = 0;
  message.y_offset = 0;

  broadcaster->send(message);
}

}  // namespace tosshin_dienen_controller
