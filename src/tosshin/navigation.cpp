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

namespace tosshin
{

using namespace std::chrono_literals;

Navigation::Navigation(std::string node_name, const char * server_ip, int server_port, const char * client_ip, int client_port)
: rclcpp::Node(node_name),
  initial_x_position(nullptr),
  initial_y_position(nullptr),
  initial_yaw_orientation(nullptr),
  forward_maneuver(0.0),
  left_maneuver(0.0),
  yaw_maneuver(0.0),
  sockfd(-1)
{

  // Initialize the node
  {
    RCLCPP_INFO_STREAM(get_logger(), "Node initialized with name " << get_name() << "!");

    // Initialize the position publisher
    {
      position_publisher = create_publisher<Position>(std::string(get_name()) + "/position", 10);

      RCLCPP_INFO_STREAM(get_logger(),
        "Position publisher initialized on " << position_publisher->get_topic_name() << "!");
    }

    // Initialize the orientation publisher
    {
      orientation_publisher = create_publisher<Orientation>(std::string(get_name()) + "/orientation", 10);

      RCLCPP_INFO_STREAM(get_logger(),
        "Orientation publisher initialized on " << orientation_publisher->get_topic_name() << "!");
    }

    // Initialize the maneuver event publisher
    {
      maneuver_event_publisher = create_publisher<Maneuver>(std::string(get_name()) + "/maneuver_event", 10);

      RCLCPP_INFO_STREAM(get_logger(),
        "Maneuver event publisher initialized on " << maneuver_event_publisher->get_topic_name() << "!");
    }

    // Initialize the maneuver input subscription
    {
      maneuver_input_subscription = create_subscription<Maneuver>(
        std::string(get_name()) + "/maneuver_input", 10,
        [this](const Maneuver::SharedPtr maneuver) {
          configure_maneuver(*maneuver);
        }
      );

      RCLCPP_INFO_STREAM(get_logger(),
        "Maneuver input subscription initialized on " << maneuver_input_subscription->get_topic_name() << "!");
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

      RCLCPP_INFO_STREAM(get_logger(),
        "Configure maneuver service initialized on " << configure_maneuver_service->get_service_name() << "!");
    }
  }

  // Initialize the TCP addresses
  {
    memset((char *)&server_addr, 0, sizeof(server_addr));

    inet_aton(server_ip, &server_addr.sin_addr);

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);

    memset((char *)&client_addr, 0, sizeof(client_addr));

    inet_aton(client_ip, &client_addr.sin_addr);

    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(client_port);
  }
}

Navigation::~Navigation()
{
  disconnect();
}

bool Navigation::connect()
{
  if (sockfd >= 0) {
    return false;
  }

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    return false;
  }

  bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));

  RCLCPP_INFO_STREAM(get_logger(), "Connected to the tcp communication!");

  // Initialize the update timer
  {
    update_timer = this->create_wall_timer(
      1ms, [this]() {
        // Receive process
        {
          char buffer[512];
          socklen_t slen = sizeof(client_addr);

          int received = recvfrom(sockfd, buffer, 512, 0, (struct sockaddr *)&client_addr, &slen);

          if (received > 0) {
            if (buffer[0] == 'i' && buffer[1] == 't' && buffer[2] == 's') {
              float yaw_orientation;
              memcpy(&yaw_orientation, buffer + 3, 4);

              // Publish current orientation
              {
                Orientation orientation;
                orientation.yaw = 0.0;

                if (initial_yaw_orientation == nullptr) {
                  *initial_yaw_orientation = orientation.yaw;
                } else {
                  orientation.yaw = yaw_orientation - (*initial_yaw_orientation);
                }

                orientation_publisher->publish(orientation);
              }
            }

            if (buffer[0] == 'p' && buffer[1] == 's' && buffer[2] == 'i')
            {
              float x_position;
              memcpy(&x_position, buffer + 3, 4);

              float y_position;
              memcpy(&y_position, buffer + 7, 4);

              // Publish current position
              {
                Position position;
                position.x = 0.0;
                position.y = y_position;

                if (initial_x_position == nullptr) {
                  *initial_x_position = position.x;
                } else {
                  position.x = x_position - (*initial_x_position);
                }

                if (initial_y_position == nullptr) {
                  *initial_y_position = position.y;
                } else {
                  position.y = y_position - (*initial_y_position);
                }

                position_publisher->publish(position);
              }
            }
          }
        }

        // Send process
        {
          char buffer[512];

          buffer[0] = 'i';
          buffer[1] = 't';
          buffer[2] = 's';

          int forward = forward_maneuver;
          int left = left_maneuver;
          int yaw = yaw_maneuver;

          memcpy(buffer + 3, &forward, 2);
          memcpy(buffer + 5, &left, 2);
          memcpy(buffer + 7, &yaw, 2);

          socklen_t slen = sizeof(client_addr);
          sendto(sockfd, buffer, 24, 0, (struct sockaddr *)&client_addr, slen);
        }
      }
    );
  }

  return true;
}

bool Navigation::disconnect()
{
  if (sockfd < 0) {
    return false;
  }

  close(sockfd);
  sockfd = -1;

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
    RCLCPP_DEBUG_STREAM(get_logger(), "Forward maneuver configured into " << forward_maneuver << "!");
  }

  if (maneuver.left.size() > 0) {
    left_maneuver = maneuver.left.front();
    result.left.push_back(left_maneuver);

    configured = true;
    RCLCPP_DEBUG_STREAM(get_logger(), "Left maneuver configured into " << left_maneuver << "!");
  }

  if (maneuver.yaw.size() > 0) {
    yaw_maneuver = maneuver.yaw.front();
    result.yaw.push_back(yaw_maneuver);

    configured = true;
    RCLCPP_DEBUG_STREAM(get_logger(), "Yaw maneuver configured into " << yaw_maneuver << "!");
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

}  // namespace tosshin
