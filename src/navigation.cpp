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

#include <tosshin/navigation.hpp>

#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

namespace tosshin
{

using namespace std::chrono_literals;

const double PI = atan(1) * 4;

Navigation::Navigation(
  std::string node_name, std::string target_host,
  int listener_port, int broadcaster_port)
: rclcpp::Node(node_name),
  calibrate_counter(30),
  yaw_orientation_offset(nullptr),
  x_position_offset(nullptr),
  y_position_offset(nullptr),
  forward_maneuver(0.0),
  left_maneuver(0.0),
  yaw_maneuver(0.0)
{
  // Initialize the node
  {
    RCLCPP_INFO_STREAM(get_logger(), "Node initialized with name " << get_name() << "!");

    // Initialize the position publisher
    {
      position_publisher = create_publisher<Position>(
        std::string(get_name()) + "/position", 10
      );

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Position publisher initialized on " <<
          position_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the orientation publisher
    {
      orientation_publisher = create_publisher<Orientation>(
        std::string(get_name()) + "/orientation", 10
      );

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Orientation publisher initialized on " <<
          orientation_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the maneuver event publisher
    {
      maneuver_event_publisher = create_publisher<Maneuver>(
        std::string(get_name()) + "/maneuver_event", 10
      );

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Maneuver event publisher initialized on " <<
          maneuver_event_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the maneuver input subscription
    {
      maneuver_input_subscription = create_subscription<Maneuver>(
        std::string(get_name()) + "/maneuver_input", 10,
        [this](const Maneuver::SharedPtr maneuver) {
          configure_maneuver(*maneuver);
        }
      );

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Maneuver input subscription initialized on " <<
          maneuver_input_subscription->get_topic_name() << "!"
      );
    }

    // Initialize the configure maneuver service
    {
      configure_maneuver_service = create_service<ConfigureManeuver>(
        std::string(get_name()) + "/configure_maneuver",
        [this](ConfigureManeuver::Request::SharedPtr request,
        ConfigureManeuver::Response::SharedPtr response) {
          response->maneuver = configure_maneuver(request->maneuver);
        }
      );

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Configure maneuver service initialized on " <<
          configure_maneuver_service->get_service_name() << "!"
      );
    }
  }

  // Initialize the listener
  {
    listener = std::make_shared<musen::StringListener>(listener_port);

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Listener initialized on port " <<
        listener->get_port() << "!");
  }

  // Initialize the broadcaster
  {
    using Broadcaster = musen::Broadcaster<BroadcastMessage>;
    broadcaster = std::make_shared<Broadcaster>(broadcaster_port);

    broadcaster->add_target_host(target_host);

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Broadcaster initialized on port " <<
        broadcaster->get_port() << "!");
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

  if (!broadcaster->connect()) {
    RCLCPP_ERROR(get_logger(), "Failed to connect the broadcaster!");
    return false;
  }

  // Initialize the update timer
  {
    update_timer = this->create_wall_timer(
      1ms, [this]() {
        listen_process();
        broadcast_process();
      }
    );
  }

  return true;
}

bool Navigation::disconnect()
{
  if (!listener->disconnect()) {
    RCLCPP_ERROR(get_logger(), "Failed to disconnect the listener!");
    return false;
  }

  if (!broadcaster->disconnect()) {
    RCLCPP_ERROR(get_logger(), "Failed to disconnect the broadcaster!");
    return false;
  }

  update_timer->cancel();

  return true;
}

Maneuver Navigation::configure_maneuver(const Maneuver & maneuver)
{
  Maneuver result;
  bool configured = false;

  if (maneuver.forward.size() > 0) {
    forward_maneuver = maneuver.forward.front();
    result.forward.push_back(forward_maneuver);

    configured = true;
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Forward maneuver configured into " <<
        forward_maneuver << "!"
    );
  }

  if (maneuver.left.size() > 0) {
    left_maneuver = maneuver.left.front();
    result.left.push_back(left_maneuver);

    configured = true;
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Left maneuver configured into " <<
        left_maneuver << "!"
    );
  }

  if (maneuver.yaw.size() > 0) {
    yaw_maneuver = maneuver.yaw.front();
    result.yaw.push_back(yaw_maneuver);

    configured = true;
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Yaw maneuver configured into " <<
        yaw_maneuver << "!"
    );
  }

  if (configured) {
    maneuver_event_publisher->publish(result);
  } else {
    result.forward.push_back(forward_maneuver);
    result.left.push_back(left_maneuver);
    result.yaw.push_back(yaw_maneuver);
  }

  return result;
}

void Navigation::listen_process()
{
  auto message = listener->receive(64);

  if (calibrate_counter > 0) {
    --calibrate_counter;
    return;
  }

  if (message.size() > 0) {
    std::vector<std::string> data;

    std::stringstream ss(message);
    while (ss.good()) {
      std::string substr;
      std::getline(ss, substr, ',');
      data.push_back(substr);
    }

    if (data.size() > 1) {
      double x_position = stod(data[1]) * 0.01;
      double y_position = stod(data[0]) * 0.01;

      RCLCPP_INFO_STREAM(get_logger(), x_position << " " << y_position);

      // Filter current position
      {
        if (x_position_offset == nullptr) {
          x_position_offset = std::make_shared<double>(x_position);
        }

        if (y_position_offset == nullptr) {
          y_position_offset = std::make_shared<double>(y_position);
        }
      }

      // Publish current position
      {
        Position position;
        position.x = x_position;
        position.y = y_position;

        position_publisher->publish(position);
      }
    }

    if (data.size() > 2) {
      double yaw_orientation = stod(data[2]);

      // Filter current orientation
      {
        if (yaw_orientation_offset == nullptr) {
          yaw_orientation_offset = std::make_shared<double>(yaw_orientation);
        }

        while (yaw_orientation > 180.0) {
          yaw_orientation -= 360.0;
        }

        while (yaw_orientation < -180.0) {
          yaw_orientation += 360.0;
        }
      }

      // Publish current orientation
      {
        Orientation orientation;
        orientation.yaw = yaw_orientation;

        orientation_publisher->publish(orientation);
      }
    }
  }
}

void Navigation::broadcast_process()
{
  BroadcastMessage message;

  message.left_maneuver = left_maneuver;
  message.forward_maneuver = forward_maneuver;
  message.yaw_maneuver = yaw_maneuver;

  message.yaw_offset = 0;
  if (yaw_orientation_offset != nullptr) {
    message.yaw_offset = *yaw_orientation_offset;
  }

  message.x_offset = 0;
  if (x_position_offset != nullptr) {
    message.x_offset = (*x_position_offset) * 100.0;
  }

  message.y_offset = 0;
  if (y_position_offset != nullptr) {
    message.y_offset = (*y_position_offset) * 100.0;
  }

  broadcaster->send(message);
}

}  // namespace tosshin