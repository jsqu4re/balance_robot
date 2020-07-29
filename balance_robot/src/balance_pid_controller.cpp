#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <string>

#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <balance_robot_msgs/msg/balance.hpp>
#include <balance_robot_msgs/msg/encoders.hpp>
#include <balance_robot_msgs/msg/motors.hpp>
#include <balance_robot_msgs/msg/orientation.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <balance_robot/pid.h>

struct vel_cmd {
  float forward;
  float turn;
  float forward_gain;
  float turn_gain;
};

struct orientation {
  double roll;
  double pitch;
  double yaw;
  double dt;
};

struct encoders {
  float position_left;
  float position_right;
  float velocity_left;
  float velocity_right;
};

struct wheel_position {
  float position_left;
  float position_right;
};

struct pid_param {
  float p;
  float i;
  float d;
  float offset;
};

static float main_loop = 0.02;

static vel_cmd velocity_cmd{0, 0, 40000, 5000};

static orientation orientation_measurement{.0, .0, .0, .2};
static encoders encoders_measurement{.0, .0, .0, .0};

static float vel_lowpass{20};
static pid_param pid_param_v{.0001, .000001, .0, .0};
static pid_param pid_param_roll{1100.0, 700.0, 90.0, 6.0};

static wheel_position motor_position{.0, .0};

void joy_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  velocity_cmd.forward = msg->axes[1];
  velocity_cmd.turn = msg->axes[0];
}

void orientation_topic_callback(
    const balance_robot_msgs::msg::Orientation::SharedPtr msg) {
  orientation_measurement.roll = msg->roll;
  orientation_measurement.pitch = msg->pitch;
  orientation_measurement.yaw = msg->yaw;
  orientation_measurement.dt = msg->dt;
}

void encoders_topic_callback(
    const balance_robot_msgs::msg::Encoders::SharedPtr msg) {
  encoders_measurement.position_left = msg->encoder1.position;
  encoders_measurement.position_right = msg->encoder0.position;
  encoders_measurement.velocity_left = msg->encoder1.velocity;
  encoders_measurement.velocity_right = msg->encoder0.velocity;
}

void param_change_callback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  for (auto parameter : event->changed_parameters) {
    if (parameter.name == "pid_roll.p")
      pid_param_roll.p = parameter.value.double_value;
    if (parameter.name == "pid_roll.i")
      pid_param_roll.i = parameter.value.double_value;
    if (parameter.name == "pid_roll.d")
      pid_param_roll.d = parameter.value.double_value;
    if (parameter.name == "pid_roll.offset")
      pid_param_roll.offset = parameter.value.double_value;

    if (parameter.name == "pid_velocity.p")
      pid_param_v.p = parameter.value.double_value;
    if (parameter.name == "pid_velocity.i")
      pid_param_v.i = parameter.value.double_value;
    if (parameter.name == "pid_velocity.d")
      pid_param_v.d = parameter.value.double_value;
    if (parameter.name == "pid_velocity.offset")
      pid_param_v.offset = parameter.value.double_value;

    if (parameter.name == "vel_cmd.forward_gain")
      velocity_cmd.forward_gain = parameter.value.double_value;
    if (parameter.name == "vel_cmd.turn_gain")
      velocity_cmd.turn_gain = parameter.value.double_value;

    if (parameter.name == "main_loop")
      main_loop = parameter.value.double_value;
  }
  printf("Updated PID_roll: P %+05.5f I %+05.5f D %+05.5f PID_velocity: P "
         "%+05.5f I %+05.5f D %+05.5f\n",
         pid_param_roll.p, pid_param_roll.i, pid_param_roll.d, pid_param_v.p,
         pid_param_v.i, pid_param_v.d);
}

int main(int argc, char *argv[]) {
  printf("balance controller starting\n");
  // init ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("balance_robot_controller");

  node->declare_parameter("pid_roll.p", pid_param_roll.p);
  node->declare_parameter("pid_roll.i", pid_param_roll.i);
  node->declare_parameter("pid_roll.d", pid_param_roll.d);
  node->declare_parameter("pid_roll.offset", pid_param_roll.offset);

  node->declare_parameter("pid_velocity.p", pid_param_v.p);
  node->declare_parameter("pid_velocity.i", pid_param_v.i);
  node->declare_parameter("pid_velocity.d", pid_param_v.d);
  node->declare_parameter("pid_velocity.offset", pid_param_v.offset);

  node->declare_parameter("vel_lowpass", vel_lowpass);

  node->declare_parameter("vel_cmd.forward_gain", velocity_cmd.forward_gain);
  node->declare_parameter("vel_cmd.turn_gain", velocity_cmd.turn_gain);

  node->declare_parameter("main_loop", main_loop);

  // std::thread listener_task(listener);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  auto balance_pub = node->create_publisher<balance_robot_msgs::msg::Balance>(
      "balance/controller", qos);

  auto motors_pub = node->create_publisher<balance_robot_msgs::msg::Motors>(
      "balance/motors", qos);

  auto joy_subscription = node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, joy_topic_callback);

  auto orientation_subscription =
      node->create_subscription<balance_robot_msgs::msg::Orientation>(
          "balance/orientation/imu", 10, orientation_topic_callback);

  auto encoders_subscription =
      node->create_subscription<balance_robot_msgs::msg::Encoders>(
          "balance/encoders", 10, encoders_topic_callback);

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node);

  auto callback_handler =
      parameters_client->on_parameter_event(param_change_callback);

  // FIXME: PID value ranges need to be aligned
  PID pid_v = PID(-30, 30, 0, 0, 0);
  PID pid_roll = PID(-30000, 30000, 0, 0, 0);

  float setpoint_roll = 0;
  float setpoint_roll_offset = pid_param_roll.offset;
  float setpoint_velocity = 0;
  float dtsum = 0;
  float roll = 0;
  float motor_increment = 0;
  float velocity_lp = 0;

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  while (rclcpp::ok()) {
    dtsum = orientation_measurement.dt;
    roll = orientation_measurement.roll;
    setpoint_velocity = velocity_cmd.forward * velocity_cmd.forward_gain;
    float measured_velocity = (encoders_measurement.velocity_right -
                               encoders_measurement.velocity_left) /
                              2;

    velocity_lp = (velocity_lp * vel_lowpass + measured_velocity) / (vel_lowpass + 1);

    // pid controllers
    setpoint_roll =
        pid_v.calculate(setpoint_velocity, velocity_lp, dtsum);
    setpoint_roll = setpoint_roll + setpoint_roll_offset;
    motor_increment = pid_roll.calculate(setpoint_roll, roll, dtsum);

    float pwm_target = motor_increment;

    // ddd turning capabilities
    float pwm_target_left =
        pwm_target + (velocity_cmd.turn * velocity_cmd.turn_gain);
    float pwm_target_right =
        pwm_target - (velocity_cmd.turn * velocity_cmd.turn_gain);

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::Balance>();

      msg->header.frame_id = "robot";
      msg->header.stamp = ros_clock.now();

      msg->velocity.setpoint = setpoint_velocity;
      msg->velocity.measurement = velocity_lp;
      msg->velocity.increment = setpoint_roll;

      msg->roll.setpoint = setpoint_roll;
      msg->roll.measurement = roll;
      msg->roll.increment = motor_increment;

      msg->motor = pwm_target;
      msg->motor_left = pwm_target_left;
      msg->motor_right = pwm_target_right;

      balance_pub->publish(std::move(msg));
    }

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::Motors>();

      msg->header.frame_id = "robot";
      msg->header.stamp = ros_clock.now();

      // motor_position.position_left += pwm_target_left;
      // motor_position.position_right -= pwm_target_right;

      msg->motor1.setpoint = pwm_target_left;
      msg->motor0.setpoint = pwm_target_right * -1;

      motors_pub->publish(std::move(msg));
    }

    rclcpp::spin_some(node);

    pid_roll.set(pid_param_roll.p, pid_param_roll.i, pid_param_roll.d);
    pid_v.set(pid_param_v.p, pid_param_v.i, pid_param_v.d);

    sleep(0.08); // FIXME: Just to get it running first
  }
}
