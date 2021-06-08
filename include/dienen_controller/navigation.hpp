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

#include <musen/musen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin_cpp/tosshin_cpp.hpp>

#include <memory>
#include <string>

namespace dienen_controller
{

class Navigation : public tosshin_cpp::NavigationProvider
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
  } __attribute__((packed, aligned(1)));

  Navigation(
    rclcpp::Node::SharedPtr node, std::string target_host,
    int listen_port = 8888, int broadcast_port = 44444);

  ~Navigation();

  bool connect();
  bool disconnect();

private:
  void listen_process();
  void broadcast_process();

  std::shared_ptr<tosshin_cpp::NavigationProvider> navigation_provider;

  rclcpp::TimerBase::SharedPtr update_timer;

  std::shared_ptr<musen::Listener> listener;
  std::shared_ptr<musen::Broadcaster> broadcaster;
};

}  // namespace dienen_controller

#endif  // DIENEN_CONTROLLER__NAVIGATION_HPP_
