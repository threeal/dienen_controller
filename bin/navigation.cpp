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

#include <rclcpp/rclcpp.hpp>
#include <tosshin_dienen_controller/navigation.hpp>

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("navigation");

  std::shared_ptr<tosshin_dienen_controller::Navigation> navigation;
  if (argc > 3) {
    navigation = std::make_shared<tosshin_dienen_controller::Navigation>(
      node, std::string(argv[1]), atoi(argv[2]), atoi(argv[3]));
  } else if (argc > 2) {
    navigation = std::make_shared<tosshin_dienen_controller::Navigation>(
      node, std::string(argv[1]), atoi(argv[2]));
  } else if (argc > 1) {
    navigation = std::make_shared<tosshin_dienen_controller::Navigation>(
      node, std::string(argv[1]));
  } else {
    std::cerr << "Usage: ros2 run tosshin_dienen_controller navigation " <<
      "<target_host> [listen_port] [broadcast_port]" << std::endl;
    return 1;
  }

  if (navigation->connect()) {
    rclcpp::spin(node);
  } else {
    std::cerr << "Failed to initialize the connection!" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
