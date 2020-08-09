#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <arpa/inet.h>
#include <math.h>
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

#include <dlib/control.h>

using namespace dlib;

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
  double d_roll;
  double d_pitch;
  double d_yaw;
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

struct inner_wheel {
  float position;
  float velocity;
};

static float main_loop = 0.08;

static vel_cmd velocity_cmd{0, 0, 40000, 5000};

static orientation orientation_imu_measurement{.0, .0, .0, .2};
static orientation orientation_ow_measurement{.0, .0, .0, .2};
static encoders encoders_measurement{.0, .0, .0, .0};

static float vel_lowpass{20};

static wheel_position motor_position{.0, .0};
static inner_wheel combined_inner_wheel{.0, .0};

void joy_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  velocity_cmd.forward = msg->axes[1];
  velocity_cmd.turn = msg->axes[0];
}

void orientation_imu_topic_callback(
    const balance_robot_msgs::msg::Orientation::SharedPtr msg) {
  orientation_imu_measurement.roll = msg->roll;
  orientation_imu_measurement.pitch = msg->pitch;
  orientation_imu_measurement.yaw = msg->yaw;
  orientation_imu_measurement.d_roll = msg->d_roll;
  orientation_imu_measurement.d_pitch = msg->d_pitch;
  orientation_imu_measurement.d_yaw = msg->d_yaw;
  orientation_imu_measurement.dt = msg->dt;
}

void orientation_ow_topic_callback(
    const balance_robot_msgs::msg::Orientation::SharedPtr msg) {
  orientation_ow_measurement.roll = msg->roll;
  orientation_ow_measurement.pitch = msg->pitch;
  orientation_ow_measurement.yaw = msg->yaw;
  orientation_ow_measurement.d_roll = msg->d_roll;
  orientation_ow_measurement.d_pitch = msg->d_pitch;
  orientation_ow_measurement.d_yaw = msg->d_yaw;
  orientation_ow_measurement.dt = msg->dt;
}

double cprToRad(double cpr_value) {
  return cpr_value * 2 * M_PI / 8192;
}

double radToCpr(double rad_value) {
  return rad_value * 8192 / (2 * M_PI);
}

void encoders_topic_callback(
    const balance_robot_msgs::msg::Encoders::SharedPtr msg) {
  encoders_measurement.position_left = cprToRad(msg->encoder1.position * -1); // FIXME: This is not sufficient to change motor direction
  encoders_measurement.position_right = cprToRad(msg->encoder0.position);
  encoders_measurement.velocity_left = cprToRad(msg->encoder1.velocity * -1);
  encoders_measurement.velocity_right = cprToRad(msg->encoder0.velocity);
  combined_inner_wheel.position = encoders_measurement.position_right;
  combined_inner_wheel.velocity = (encoders_measurement.velocity_right + encoders_measurement.velocity_left) / 2;
}

void param_change_callback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  for (auto parameter : event->changed_parameters) {
    if (parameter.name == "vel_cmd.forward_gain")
      velocity_cmd.forward_gain = parameter.value.double_value;
    if (parameter.name == "vel_cmd.turn_gain")
      velocity_cmd.turn_gain = parameter.value.double_value;

    if (parameter.name == "main_loop")
      main_loop = parameter.value.double_value;
  }
}

int main(int argc, char *argv[]) {
  printf("balance controller starting\n");
  // init ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("balance_robot_controller");
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

  auto orientation_imu_subscription =
      node->create_subscription<balance_robot_msgs::msg::Orientation>(
          "balance/orientation/imu", 10, orientation_imu_topic_callback);

  auto orientation_ow_subscription =
      node->create_subscription<balance_robot_msgs::msg::Orientation>(
          "balance/orientation/ow", 10, orientation_ow_topic_callback);

  auto encoders_subscription =
      node->create_subscription<balance_robot_msgs::msg::Encoders>(
          "balance/encoders", 10, encoders_topic_callback);

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node);

  auto callback_handler =
      parameters_client->on_parameter_event(param_change_callback);

  float motor_increment = 0;
  float velocity_lp = 0;



  const int STATES = 6;
  const int CONTROLS = 1;

  matrix<double,STATES,1> current_state;
  current_state = 0,0,0,0,0,0;

  matrix<double,STATES,1> target_state;
  target_state = 0.1415,0,0,0,0,0;

  matrix<double,STATES,STATES> A;
  A =  1.62808335e+00,  1.12579382e-01, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       1.34683383e+01,  1.46065417e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       0.00000000e+00,  0.00000000e+00,  5.02317008e-01,  7.70948122e-02,  0.00000000e+00,  2.05294378e-05,
      -0.00000000e+00, -0.00000000e+00, -8.79733272e+00,  4.05398123e-01, -0.00000000e+00,  3.40513655e-04,
       0.00000000e+00,  0.00000000e+00, -1.74678326e-04,  4.25539709e-05,  1.00000000e+00,  8.64210138e-02,
       0.00000000e+00,  0.00000000e+00, -4.85585774e-03,  7.05825866e-04,  0.00000000e+00,  7.41307296e-01;

  matrix<double,STATES,CONTROLS> B;
  B =  -0.70508113,
      -15.11944431,
        0.95705824,
       16.92134030,
        2.32133538,
       44.22661332;

  matrix<double,STATES,1> C;
  C = 0,
      0,
      0,
      0,
      0,
      0;


  const int HORIZON = 5;

  matrix<double,STATES,1> Q;
  Q = 5, 1, 0.1, 4, 2, 1;

  matrix<double,CONTROLS,1> R, lower, upper;
  R = 0.1;
  lower = -10.0;
  upper =  10.0;

  mpc<STATES,CONTROLS,HORIZON> controller(A,B,C,Q,R,lower,upper);
  controller.set_target(target_state);

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  while (rclcpp::ok()) {
    current_state(0) = orientation_imu_measurement.roll;
    current_state(1) = orientation_imu_measurement.d_roll;
    current_state(2) = -1 * orientation_ow_measurement.pitch;
    current_state(3) = -1 * orientation_ow_measurement.d_pitch;
    current_state(4) = combined_inner_wheel.position;
    current_state(5) = combined_inner_wheel.velocity;

    matrix<double,CONTROLS,1> action = controller(current_state);

    auto current_stamp = ros_clock.now();
    // float pwm_target = current_state(5) + action(0);
    float pwm_target = action(0);

    float pwm_target_left =
        pwm_target + (velocity_cmd.turn * velocity_cmd.turn_gain);
    float pwm_target_right =
        pwm_target - (velocity_cmd.turn * velocity_cmd.turn_gain);

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::Balance>();

      msg->header.frame_id = "robot";
      msg->header.stamp = current_stamp;

      msg->velocity.setpoint = 0;
      msg->velocity.measurement = 0;
      msg->velocity.increment = 0;

      msg->roll.setpoint = 0;
      msg->roll.measurement = 0;
      msg->roll.increment = 0;

      msg->motor = pwm_target;
      msg->motor_left = pwm_target_left;
      msg->motor_right = pwm_target_right;

      balance_pub->publish(std::move(msg));
    }

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::Motors>();

      msg->header.frame_id = "robot";
      msg->header.stamp = current_stamp;

      msg->motor1.setpoint = radToCpr(pwm_target_left);
      msg->motor0.setpoint = radToCpr(pwm_target_right * -1);

      motors_pub->publish(std::move(msg));
    }
    rclcpp::spin_some(node);
    sleep(main_loop);
  }
}
