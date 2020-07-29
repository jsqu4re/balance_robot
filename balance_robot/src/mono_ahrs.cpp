#include <navio_vendor/Common/MPU9250.h>
#include <navio_vendor/Common/Util.h>
#include <navio_vendor/Navio2/LSM9DS1.h>

#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <navio_vendor/Examples/AHRS.hpp>

#include <balance_robot_msgs/msg/orientation.hpp>

static float main_loop = 0.02;

struct roll_dt {
  float roll;
  float pitch;
  float yaw;
  float dt;
};

std::unique_ptr<InertialSensor> get_inertial_sensor(std::string sensor_name) {
  if (sensor_name == "mpu") {
    printf("Selected: MPU9250\n");
    auto ptr = std::unique_ptr<InertialSensor>{new MPU9250()};
    return ptr;
  } else if (sensor_name == "lsm") {
    printf("Selected: LSM9DS1\n");
    auto ptr = std::unique_ptr<InertialSensor>{new LSM9DS1()};
    return ptr;
  } else {
    return NULL;
  }
}

void print_help() {
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf("If you want to visualize IMU data on another machine,\n");
  printf("add IP address and port number (by default 7000):\n");
  printf("-i [sensor name] ipaddress portnumber\n");
}

std::string get_sensor_name(int argc, char *argv[]) {
  if (get_navio_version() == NAVIO2) {

    if (argc < 2) {
      printf("Enter parameter\n");
      print_help();
      return std::string();
    }

    // prevent the error message
    opterr = 0;
    int parameter;

    while ((parameter = getopt(argc, argv, "i:h")) != -1) {
      switch (parameter) {
      case 'i':
        return optarg;
      case 'h':
        print_help();
        return "";
      case '?':
        printf("Wrong parameter.\n");
        print_help();
        return std::string();
      }
    }

  } else { // sensor on NAVIO+

    return "mpu";
  }
  return "";
}

//============================== Main loop ====================================

roll_dt imuLoop(AHRS *ahrs) {
  // Orientation data

  float roll, pitch, yaw;

  struct timeval tv;
  float dt;
  // Timing data

  static float maxdt;
  static float mindt = 0.01;
  static int isFirst = 1;
  static unsigned long previoustime, currenttime;

  //----------------------- Calculate delta time ----------------------------

  gettimeofday(&tv, NULL);
  previoustime = currenttime;
  currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;
  if (dt < 1 / 1300.0)
    usleep((1 / 1300.0 - dt) * 1000000);
  gettimeofday(&tv, NULL);
  currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;

  //-------- Read raw measurements from the MPU and update AHRS --------------

  ahrs->updateIMU(dt);

  //------------------------ Read Euler angles ------------------------------

  ahrs->getEuler(&roll, &pitch, &yaw);

  //------------------- Discard the time of the first cycle -----------------

  if (!isFirst) {
    if (dt > maxdt)
      maxdt = dt;
    if (dt < mindt)
      mindt = dt;
  }
  isFirst = 0;

  return roll_dt{roll, pitch, yaw, dt};
}

// using namespace Navio;

using std::placeholders::_1;

int main(int argc, char *argv[]) {
  // init ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("balance_robot_orientation");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  auto balance_pub = node->create_publisher<balance_robot_msgs::msg::Orientation>(
      "balance/orientation/imu", qos);

//  auto parameters_client =
//      std::make_shared<rclcpp::AsyncParametersClient>(node);

//  auto callback_handler =
//      parameters_client->on_parameter_event(param_change_callback);

  // Check to be the only user
  if (check_apm()) {
    return EXIT_FAILURE;
  }

  // Setup IMU
  auto sensor_name = get_sensor_name(argc, argv);

  if (sensor_name.empty())
    return EXIT_FAILURE;

  auto imu = get_inertial_sensor(sensor_name);

  if (!imu) {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }

  if (!imu->probe()) {
    printf("Sensor not enable\n");
    return EXIT_FAILURE;
  }

  auto ahrs = std::unique_ptr<AHRS>{new AHRS(move(imu))};

  // Setup gyroscope offset
  ahrs->setGyroOffset();

  {
    printf("Waiting...\n");
    roll_dt measurement{100, 100, 100, 0};
    while (measurement.roll > 3 || measurement.roll < -3) {
      measurement = imuLoop(ahrs.get());
      rclcpp::spin_some(node);
    }
    printf("ready! %+05.2f\n", measurement.roll);
  }

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  float roll, pitch, yaw;
  float dtsum = 0;

  while (rclcpp::ok()) {
    roll_dt measurement = imuLoop(ahrs.get());

    dtsum += measurement.dt;
    roll = measurement.roll;
    pitch = measurement.pitch;
    yaw = measurement.yaw;

    if (dtsum > main_loop) {

      auto msg = std::make_unique<balance_robot_msgs::msg::Orientation>();

      msg->header.frame_id = "balance_robot_imu";
      msg->header.stamp = ros_clock.now();

      msg->roll = roll;
      msg->pitch = pitch;
      msg->yaw = yaw;

      msg->dt = dtsum;

      balance_pub->publish(std::move(msg));

      dtsum = 0;

      rclcpp::spin_some(node);
    }
  }
}
