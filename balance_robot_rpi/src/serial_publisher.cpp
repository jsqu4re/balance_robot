#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <balance_robot_msgs/msg/orientation.hpp>

#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <balance_robot_rpi/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

int open_serial(std::string sensor_port) {
  try {
    return serialOpen(sensor_port.c_str(), 9600);
  } catch (const std::exception &e) {
	std::cerr << "Error opening serial port" << std::endl;
	sleep(1);
	return -1;
  }
}

int main(int argc, char *argv[]) {
  std::string device_name = argv[1];
  if (device_name == "") {
    device_name = "/dev/ttyACM0";
  }

  rclcpp::init(argc, argv);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  auto node = rclcpp::Node::make_shared("serial_publisher_ow");
  auto publisher_body = node->create_publisher<balance_robot_msgs::msg::Orientation>(
      "balance/orientation/imu", 10);
  auto publisher_wheel = node->create_publisher<balance_robot_msgs::msg::Orientation>(
      "balance/orientation/ow", 10);

  int sensor = -1;
  while(sensor < 0){
    sensor = open_serial(device_name);
    rclcpp::spin_some(node);
  };

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
            msg->header.frame_id = j["frame"].get<std::string>();
            msg->header.stamp = ros_clock.now();
            // Add differential information
            msg->pitch = j["P"].get<double>();
            msg->roll = j["R"].get<double>();
            msg->yaw = j["Y"].get<double>();
            msg->d_pitch = j["dP"].get<double>();
            msg->d_roll = j["dR"].get<double>();
            msg->d_yaw = j["dY"].get<double>();
            if (j["f"] > 0) {
              msg->dt = 1. / j["f"].get<double>();
            }
            if (msg->header.frame_id == "body") {
              publisher_body->publish(std::move(msg));
            } else {
              publisher_wheel->publish(std::move(msg));
            }
          } catch (const std::exception &e) {
            std::cout << combined << std::endl;
          }
        } else {
          std::cout << combined << std::endl;
        }
        combined = "";
      }
    }
    rclcpp::spin_some(node);
	sleep(0.1);
  }
  rclcpp::shutdown();
  return 0;
}