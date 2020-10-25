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

#include <balance_robot_msgs/msg/encoders.hpp>
#include <balance_robot_msgs/msg/gains.hpp>
#include <balance_robot_msgs/msg/motors.hpp>
#include <balance_robot_msgs/msg/orientation.hpp>
#include <balance_robot_msgs/msg/states.hpp>
#include <sensor_msgs/msg/joy.hpp>

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

static std::array<double, 6> control_k = {-467.79730132, -42.88097678, 15.94971425, -6.14649532, -4.47213596, -4.3158712};
// −311.753349248838, −28.670137694416, 46.020894063272, −0.69701467630523, −4.47213596519255, −2.94092932828381
static std::array<double, 6> calibration = {0, 0, 0, 0, 0, 0};

static vel_cmd velocity_cmd{0, 0, 0.05, 3};

static orientation orientation_imu_measurement{.0, .0, .0, .0, .0, .0, .2};
static orientation orientation_ow_measurement{.0, .0, .0, .0, .0, .0, .2};
static encoders encoders_measurement{.0, .0, .0, .0};

static float vel_lowpass{20};

static wheel_position motor_position{.0, .0};
static inner_wheel combined_inner_wheel{.0, .0};

void joy_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  velocity_cmd.forward = msg->axes[1];
  velocity_cmd.turn = msg->axes[0];
}

void gains_topic_callback(const balance_robot_msgs::msg::Gains::SharedPtr msg) {
  control_k = msg->k;
}

void calibration_topic_callback(const balance_robot_msgs::msg::Gains::SharedPtr msg) {
  calibration = msg->k;
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
  auto states_pub = node->create_publisher<balance_robot_msgs::msg::States>(
      "balance/states", qos);

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

  auto gains_subscription = node->create_subscription<balance_robot_msgs::msg::Gains>(
      "balance/gains", 10, gains_topic_callback);

  auto calibration_subscription = node->create_subscription<balance_robot_msgs::msg::Gains>(
      "balance/calibration", 10, calibration_topic_callback);

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node);

  auto callback_handler =
      parameters_client->on_parameter_event(param_change_callback);

  double motor_increment = 0;
  double velocity_lp = 0;

  std::array<double, 6> state_x = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> target_w = {0, 0, 0, 0, 0, 0};

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  while (rclcpp::ok()) {
    state_x[0] = -1 * orientation_imu_measurement.pitch + calibration[0];
    state_x[1] = -1 * orientation_imu_measurement.d_pitch + calibration[1];
    state_x[2] = orientation_ow_measurement.pitch + calibration[2];
    state_x[3] = orientation_ow_measurement.d_pitch + calibration[3];
    state_x[4] = combined_inner_wheel.position + calibration[4];
    state_x[5] = combined_inner_wheel.velocity + calibration[5];

    target_w[4] = target_w[4] + (velocity_cmd.forward * velocity_cmd.forward_gain);
    target_w[5] = velocity_cmd.forward * velocity_cmd.forward_gain;

    // Limit max delta input
    if ( target_w[4] > state_x[4] + M_PI * 5 ) {
      target_w[4] = state_x[4] + M_PI * 5;
    }

    if ( target_w[4] < state_x[4] - M_PI * 5 ) {
      target_w[4] = state_x[4] - M_PI * 5;
    }

    motor_increment = 0;
    for (int i = 0; i < 6; ++i){
      motor_increment -= (state_x[i] - target_w[i]) * control_k[i];
    }

    velocity_lp = (velocity_lp + state_x[5]) / 2;
    auto current_stamp = ros_clock.now();
    float pwm_target = velocity_lp + motor_increment * main_loop; // v = v_measurement + a * t

    float pwm_target_left =
        pwm_target - (velocity_cmd.turn * velocity_cmd.turn_gain);
    float pwm_target_right =
        pwm_target + (velocity_cmd.turn * velocity_cmd.turn_gain);

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::States>();

      msg->header.frame_id = "robot";
      msg->header.stamp = current_stamp;

      msg->x = state_x;
      msg->w = target_w;
      msg->k = control_k;
      msg->u[0] = pwm_target;

      states_pub->publish(std::move(msg));
    }

    {
      auto msg = std::make_unique<balance_robot_msgs::msg::Motors>();

      msg->header.frame_id = "robot";
      msg->header.stamp = current_stamp;

      msg->motor1.setpoint = radToCpr(pwm_target_left * -1);
      msg->motor0.setpoint = radToCpr(pwm_target_right);

      motors_pub->publish(std::move(msg));
    }
    rclcpp::spin_some(node);
    sleep(main_loop);
  }
}
