#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <balance_robot_msgs/msg/orientation.hpp>

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <balance_robot_rpi/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  auto node = rclcpp::Node::make_shared("serial_publisher_ow_right");

  auto publisher = node->create_publisher<balance_robot_msgs::msg::Orientation>("/balance/outerwheel/right", 10);

  char* sensor_port = "/dev/ttyACM0";
	int sensor = serialOpen(sensor_port, 9600);
	wiringPiSetup();

	std::string combined = "";

  while (rclcpp::ok()) {

    char buffer[200];
		ssize_t length = read(sensor, &buffer, sizeof(buffer));
		if (length == -1) {
			std::cerr << "Error reading from serial port" << std::endl;
			break;
		} else if (length == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		} else {
			buffer[length] = '\0';
			combined += std::string(buffer);
			if (combined.back() == '\n') {
				auto j = json::parse(combined, nullptr, false);
				if (!j.is_discarded()) {
					try {
            auto msg = std::make_unique<balance_robot_msgs::msg::Orientation>();
            msg->header.frame_id = "outer_wheel";
            msg->header.stamp = ros_clock.now();
            // Add differential information
						msg->pitch = j["P"].get<double>();
						msg->roll = j["R"].get<double>();
						msg->yaw = j["Y"].get<double>();
            if (j["f"] > 0) {
						  msg->dt = 1. / j["f"].get<double>();
            }
            publisher->publish(std::move(msg));
					} catch (const std::exception& e) {
						std::cout << combined << std::endl;
					}
				} else {
					std::cout << combined << std::endl;
				}
				combined = "";
			}
		}
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}