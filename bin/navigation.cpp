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

#include <argparse/argparse.hpp>
#include <dienen_controller/navigation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("navigation", "0.3.0");

  dienen_controller::Navigation::Options navigation_options;

  program.add_argument("target_host")
  .help("target host IP of the controller");

  program.add_argument("--listen-port", "-l")
  .help("listen port to be used")
  .default_value(navigation_options.listen_port)
  .action([](const std::string & value) {return std::stoi(value);});

  program.add_argument("--broadcast-port", "-b")
  .help("broadcast port to be used")
  .default_value(navigation_options.broadcast_port)
  .action([](const std::string & value) {return std::stoi(value);});

  program.add_argument("--position-from-twist")
  .help("obtains position from twist movement calculation")
  .default_value(false)
  .implicit_value(true);

  program.add_argument("--no-reset-odometry")
  .help("disable reset odometry on start")
  .default_value(false)
  .implicit_value(true);

  try {
    program.parse_args(argc, argv);

    navigation_options.target_host = program.get<std::string>("target_host");
    navigation_options.listen_port = program.get<int>("--listen-port");
    navigation_options.broadcast_port = program.get<int>("--broadcast-port");
    navigation_options.position_from_twist = program.get<bool>("--position-from-twist");
    navigation_options.no_reset_odometry = program.get<bool>("--no-reset-odometry");
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto navigation = std::make_shared<dienen_controller::Navigation>(navigation_options);

  if (navigation->connect()) {
    rclcpp::spin(navigation);
  } else {
    std::cerr << "Failed to initialize the connection!" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
