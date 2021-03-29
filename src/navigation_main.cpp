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
#include <tosshin/navigation.hpp>

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cout << "Usage: ros2 run tosshin navigation" <<
      " <server_ip> <server_port> <client_ip> <client_port>" << std::endl;
    return 1;
  }

  const char * server_ip = argv[1];
  int server_port = atoi(argv[2]);

  const char * client_ip = argv[3];
  int client_port = atoi(argv[4]);

  rclcpp::init(argc, argv);

  auto navigation = std::make_shared<tosshin::Navigation>(
    "navigation", server_ip, server_port, client_ip, client_port
  );

  if (navigation->connect()) {
    rclcpp::spin(navigation);
  } else {
    std::cerr << "Failed to connect to the TCP communication!" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
