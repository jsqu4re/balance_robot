#include <navio_vendor/Common/MPU9250.h>
#include <navio_vendor/Common/Util.h>
// #include <navio_vendor/Navio+/RCOutput_Navio.h>
#include <navio_vendor/Navio2/LSM9DS1.h>
#include <navio_vendor/Navio2/PWM.h>
#include <navio_vendor/Navio2/RCOutput_Navio2.h>

#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <navio_vendor/Examples/AHRS.hpp>

#include <balance_robot/pid.h>

#include <balance_robot_msgs/msg/balance.hpp>

static float forward = 0;
static float turn = 0;

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

struct roll_dt {
  float roll;
  float dt;
};

//============================== Main loop ====================================

roll_dt imuLoop(AHRS *ahrs) {
  // Orientation data

  float roll, pitch, yaw;

  struct timeval tv;
  float dt;
  // Timing data

  static float maxdt;
  static float mindt = 0.01;
  static float dtsumm = 0;
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

  return roll_dt{roll, dt};
}

#define SERVO_MIN 1280 /*mS*/
#define SERVO_MID 1500 /*mS*/
#define SERVO_MAX 1720 /*mS*/

#define PWM_OUTPUT_WHEEL_LEFT 1
#define PWM_OUTPUT_WHEEL_RIGHT 2

// using namespace Navio;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
    forward = msg->axes[1];
    turn = msg->axes[0];
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

std::unique_ptr<RCOutput> get_rcout() {
  auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio2()};
  return ptr;
}

//=============================================================================

void stop_motors_handler(sig_atomic_t s) {
  printf("Terminating greacfully\n");
  auto pwm = get_rcout();
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, SERVO_MID);
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, SERVO_MID);
  rclcpp::shutdown();
  exit(0);
}

void listener() { rclcpp::spin(std::make_shared<MinimalSubscriber>()); }

int main(int argc, char *argv[]) {
  signal(SIGINT, stop_motors_handler);

  // init ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Balance");

  node->declare_parameter("pid_roll.p");
  node->declare_parameter("pid_roll.i");
  node->declare_parameter("pid_roll.d");

  std::thread listener_task(listener);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  auto balance_pub = node->create_publisher<balance_robot_msgs::msg::Balance>(
      "balance/control", qos);

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

  // Setup PWM Output
  auto pwm = get_rcout();

  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    return EXIT_FAILURE;
  }

  if (!(pwm->initialize(PWM_OUTPUT_WHEEL_LEFT))) {
    printf("Could not initialize left wheel\n");
    return EXIT_FAILURE;
  }

  if (!(pwm->initialize(PWM_OUTPUT_WHEEL_RIGHT))) {
    printf("Could not initialize right wheel\n");
    return EXIT_FAILURE;
  }

  pwm->set_frequency(PWM_OUTPUT_WHEEL_LEFT, 50);
  pwm->set_frequency(PWM_OUTPUT_WHEEL_RIGHT, 50);

  if (!(pwm->enable(PWM_OUTPUT_WHEEL_LEFT))) {
    return EXIT_FAILURE;
  }

  if (!(pwm->enable(PWM_OUTPUT_WHEEL_RIGHT))) {
    return EXIT_FAILURE;
  }

  auto ahrs = std::unique_ptr<AHRS>{new AHRS(move(imu))};

  // Setup gyroscope offset
  ahrs->setGyroOffset();

  // Set motors to off
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, SERVO_MID);
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, SERVO_MID);

  PID pid_v = PID(-3, 3, 0.0001, 0.0001, 0);

  PID pid_roll =
      PID(SERVO_MIN - SERVO_MID, SERVO_MAX - SERVO_MID, 11, 80, 0.01);

  float setpoint_roll = -1.44;
  float setpoint_v = 0;
  float dtsumm;

  {
    printf("Waiting...\n");
    roll_dt measurement{100, 0};
    while (measurement.roll > 3 || measurement.roll < -3) {
      measurement = imuLoop(ahrs.get());
    }
    printf("ready! %+05.2f\n", measurement.roll);
  }

  while (true) {
    roll_dt measurement = imuLoop(ahrs.get());
    float increment =
        pid_roll.calculate(setpoint_roll, measurement.roll, measurement.dt);

    setpoint_roll = pid_v.calculate(0, increment, measurement.dt);

    setpoint_roll += (forward * 10) - 1.44;

    float pwm_target = SERVO_MID + increment;

    dtsumm += measurement.dt;
    if (dtsumm > 0.025) {
      // Add turning
      float pwm_target_left = pwm_target + (turn * 70);
      float pwm_target_right = pwm_target - (turn * 70);

      if (pwm_target_left > SERVO_MAX) {
        float diff = pwm_target_left - SERVO_MAX;
        pwm_target_left = SERVO_MAX;
        pwm_target_right -= diff;
      }

      if (pwm_target_left < SERVO_MIN) {
        float diff = SERVO_MIN - pwm_target_left;
        pwm_target_left = SERVO_MIN;
        pwm_target_right += diff;
      }

      if (pwm_target_right > SERVO_MAX) {
        float diff = pwm_target_right - SERVO_MAX;
        pwm_target_right = SERVO_MAX;
        pwm_target_left -= diff;
      }

      if (pwm_target_right < SERVO_MIN) {
        float diff = SERVO_MIN - pwm_target_right;
        pwm_target_right = SERVO_MIN;
        pwm_target_left += diff;
      }

      pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, pwm_target_left);
      pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, pwm_target_right);
      // Console output
      printf(
          "SETPOINT: %+05.2f ROLL: %+05.2f INC: %+05.2f TARGET: %+05.2f PERIOD "
          "%.4fs RATE "
          "%dHz \n",
          setpoint_roll, measurement.roll, increment, pwm_target,
          measurement.dt, int(1 / measurement.dt));

      auto msg = std::make_unique<balance_robot_msgs::msg::Balance>();

      msg->setpoint = setpoint_roll;
      msg->roll = measurement.roll;
      msg->increment = increment;
      msg->target_pwm = pwm_target;
      msg->target_pwm_left = pwm_target_left;
      msg->target_pwm_right = pwm_target_right;

      balance_pub->publish(std::move(msg));

      dtsumm = 0;
    }
  }
}
