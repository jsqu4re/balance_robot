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
#include <balance_robot_msgs/msg/motor_control.hpp>
#include <balance_robot_msgs/msg/orientation.hpp>
#include <balance_robot_msgs/msg/states.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

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
  bool reset;
};

static float main_loop = 0.08;

// static std::array<double, 6> control_k = {-467.79730132, -42.88097678, 15.94971425, -6.14649532, -4.47213596, -4.3158712};
static std::array<double, 6> control_k = {-596.90351136, -51.49636883, 48.51258556, -5.80937142, -6.32455534, -3.22630397};
// static std::array<double, 6> control_k = {-467.79730132, -42.88097678, 15.94971425, -6.14649532, -4.47213596, -4.3158712};
// static std::array<double, 6> control_k = {-308.73601156, -28.39590112, 46.91495775, -1.79074614, -4.47213595, -2.43355035};
static std::array<double, 6> calibration = {-0.065, 0, 0, 0, 0, 0};

static vel_cmd velocity_cmd{0, 0, 0.05, 3};

static orientation orientation_imu_measurement{.0, .0, .0, .0, .0, .0, .2};
static orientation orientation_ow_measurement{.0, .0, .0, .0, .0, .0, .2};
static encoders encoders_measurement{.0, .0, .0, .0};

static float vel_lowpass{20};

static inner_wheel combined_inner_wheel{.0, .0, true};

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
  if (combined_inner_wheel.reset) {
    encoders_measurement.position_left = cprToRad(msg->encoder1.position * -1); // FIXME: This is not sufficient to change motor direction
    encoders_measurement.position_right = cprToRad(msg->encoder0.position);
    encoders_measurement.velocity_left = cprToRad(msg->encoder1.velocity * -1);
    encoders_measurement.velocity_right = cprToRad(msg->encoder0.velocity);
    combined_inner_wheel.reset = false;
  } else {
    encoders_measurement.position_left = (encoders_measurement.position_left + cprToRad(msg->encoder1.position * -1)) / 2; // FIXME: This is not sufficient to change motor direction
    encoders_measurement.position_right = (encoders_measurement.position_right + cprToRad(msg->encoder0.position)) / 2;
    encoders_measurement.velocity_left = (encoders_measurement.velocity_left + cprToRad(msg->encoder1.velocity * -1)) / 2;
    encoders_measurement.velocity_right = (encoders_measurement.velocity_right + cprToRad(msg->encoder0.velocity)) / 2;
  }
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

class LqrControllerNode : public rclcpp::Node
{
public:
  LqrControllerNode()
  : Node("LqrControllerNode") {
    RCLCPP_INFO(this->get_logger(), "lqr controller startup");

    setup_parameters();
    setup_subscribers();
    setup_publisher();

    RCLCPP_INFO(this->get_logger(), "setup communication complete");

    auto timer_callback =
    [this]() -> void {
      update_states();
      auto acceleration = compute_lqr_control();
      auto turn_velocity = get_turn_velocity();
      auto current_stamp = this->now();
      publish_internal_states(acceleration, current_stamp);
      publish_motor_control(acceleration, turn_velocity, current_stamp);
    };
    timer_ = this->create_wall_timer(5ms, timer_callback);

    RCLCPP_INFO(this->get_logger(), "initialized lqr timer callback");
  }
private:
  std::array<double, 6> state_x_ = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> target_w_ = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> state_x_lp_ = {0, 0, 0, 0, 0, 0};

  rclcpp::TimerBase::SharedPtr timer_;

  const static int NUMBER_SUBS = 6;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_[NUMBER_SUBS];
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<balance_robot_msgs::msg::Orientation>::SharedPtr orientation_imu_subscription_;
  rclcpp::Subscription<balance_robot_msgs::msg::Orientation>::SharedPtr orientation_ow_subscription_;
  rclcpp::Subscription<balance_robot_msgs::msg::Encoders>::SharedPtr encoders_subscription_;
  rclcpp::Subscription<balance_robot_msgs::msg::Gains>::SharedPtr gains_subscription_;
  rclcpp::Subscription<balance_robot_msgs::msg::Gains>::SharedPtr calibration_subscription_;

  rclcpp::Publisher<balance_robot_msgs::msg::States>::SharedPtr states_publisher_;
  rclcpp::Publisher<balance_robot_msgs::msg::MotorControl>::SharedPtr motors_publisher_;

  void setup_parameters() {
    this->declare_parameter("vel_lowpass", vel_lowpass);
    this->declare_parameter("vel_cmd.forward_gain", velocity_cmd.forward_gain);
    this->declare_parameter("vel_cmd.turn_gain", velocity_cmd.turn_gain);
    this->declare_parameter("main_loop", main_loop);

    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(this);

    parameters_client->on_parameter_event(param_change_callback);
  }

  void setup_subscribers() {
    for (auto& callback_group_subscriber : callback_group_subscribers_) {
      callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    rclcpp::SubscriptionOptions sub_opts[NUMBER_SUBS];
    for (auto& sub_opt : sub_opts) {
      sub_opt = rclcpp::SubscriptionOptions();
    }

    for (int i = 0; i < NUMBER_SUBS; ++i) {
      sub_opts[i].callback_group = callback_group_subscribers_[i];
    }

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10), joy_topic_callback, sub_opts[0]);

    orientation_imu_subscription_ = this->create_subscription<balance_robot_msgs::msg::Orientation>(
            "balance/orientation/imu", rclcpp::QoS(10), orientation_imu_topic_callback, sub_opts[1]);

    orientation_ow_subscription_ = this->create_subscription<balance_robot_msgs::msg::Orientation>(
            "balance/orientation/ow", rclcpp::QoS(10), orientation_ow_topic_callback, sub_opts[2]);

    encoders_subscription_ = this->create_subscription<balance_robot_msgs::msg::Encoders>(
            "balance/encoders", rclcpp::QoS(10), encoders_topic_callback, sub_opts[3]);

    gains_subscription_ = this->create_subscription<balance_robot_msgs::msg::Gains>(
        "balance/gains", rclcpp::QoS(10), gains_topic_callback, sub_opts[4]);

    calibration_subscription_ = this->create_subscription<balance_robot_msgs::msg::Gains>(
        "balance/calibration", rclcpp::QoS(10), calibration_topic_callback, sub_opts[5]);
  }

  void setup_publisher() {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    states_publisher_ = this->create_publisher<balance_robot_msgs::msg::States>(
        "balance/states", qos);
    motors_publisher_ = this->create_publisher<balance_robot_msgs::msg::MotorControl>(
        "balance/motors", qos);
  }

  void update_states() {
    state_x_[0] = -1 * orientation_imu_measurement.pitch + calibration[0];
    state_x_[1] = -1 * orientation_imu_measurement.d_pitch + calibration[1];
    state_x_[2] = -1 * orientation_ow_measurement.pitch + calibration[2];
    state_x_[3] = -1 * orientation_ow_measurement.d_pitch + calibration[3];
    state_x_[4] = combined_inner_wheel.position + calibration[4];
    state_x_[5] = combined_inner_wheel.velocity + calibration[5];
    combined_inner_wheel.reset = true;

    target_w_[4] = target_w_[4] + (velocity_cmd.forward * velocity_cmd.forward_gain);
    target_w_[5] = velocity_cmd.forward * velocity_cmd.forward_gain;

    // Limit max delta input
    if ( target_w_[4] > state_x_[4] + M_PI * 5 ) {
      target_w_[4] = state_x_[4] + M_PI * 5;
    }

    if ( target_w_[4] < state_x_[4] - M_PI * 5 ) {
      target_w_[4] = state_x_[4] - M_PI * 5;
    }

    for (int i = 0; i < 6; ++i){
      state_x_lp_[i] = (2 * state_x_lp_[i] + state_x_[i]) / 3;
    }
  }

  double compute_lqr_control() {
    double acceleration = 0;
    for (int i = 0; i < 6; ++i){
      acceleration -= (state_x_[i] - target_w_[i]) * control_k[i];
    }
    return acceleration;
  }

  double get_turn_velocity() {
    return velocity_cmd.turn * velocity_cmd.turn_gain;
  }

  void publish_internal_states(const double acceleration, const rclcpp::Time& stamp) {
    auto msg = std::make_unique<balance_robot_msgs::msg::States>();

    msg->header.frame_id = "robot";
    msg->header.stamp = stamp;

    msg->x = state_x_;
    msg->lp_x = state_x_lp_;
    msg->c = calibration;
    msg->w = target_w_;
    msg->k = control_k;
    msg->u[0] = acceleration;

    states_publisher_->publish(std::move(msg));
  }

  void publish_motor_control(const double acceleration, const double turn_velocity, const rclcpp::Time& stamp) {
    auto msg = std::make_unique<balance_robot_msgs::msg::MotorControl>();

    msg->header.frame_id = "robot";
    msg->header.stamp = stamp;

    msg->acceleration = radToCpr(acceleration);
    msg->turn_velocity = radToCpr(turn_velocity);

    motors_publisher_->publish(std::move(msg));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<LqrControllerNode>();

  // executor.add_node(node);
  // executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}