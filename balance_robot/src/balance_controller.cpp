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

struct pid_param {
  float p;
  float i;
  float d;
};

static float main_loop = 0.02;

static vel_cmd velocity_cmd{0, 0, 10000, 70};
static std::vector<orientation> orientation_measurements {};
static std::vector<encoders> encoders_measurements {};

static pid_param pid_param_v{0.001, 0.0001, 0};
static pid_param pid_param_roll{14.5, 44, 0.016};

void joy_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  velocity_cmd.forward = msg->axes[1];
  velocity_cmd.turn = msg->axes[0];
}

void orientation_topic_callback(const balance_robot_msgs::msg::Orientation::SharedPtr msg) {
  orientation measurement{msg->roll,msg->pitch,msg->yaw,msg->dt};
  orientation_measurements.push_back(measurement);
}

void encoders_topic_callback(const balance_robot_msgs::msg::Encoders::SharedPtr msg) {
  encoders measurement{};
  measurement.position_left = msg->encoder1.position;
  measurement.position_right = msg->encoder0.position;
  measurement.velocity_left = msg->encoder1.velocity;
  measurement.velocity_right = msg->encoder0.velocity;
  encoders_measurements.push_back(measurement);
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

    if (parameter.name == "pid_v.p")
      pid_param_v.p = parameter.value.double_value;
    if (parameter.name == "pid_v.i")
      pid_param_v.i = parameter.value.double_value;
    if (parameter.name == "pid_v.d")
      pid_param_v.d = parameter.value.double_value;

    if (parameter.name == "vel_cmd.forward_gain")
      velocity_cmd.forward_gain = parameter.value.double_value;
    if (parameter.name == "vel_cmd.turn_gain")
      velocity_cmd.turn_gain = parameter.value.double_value;

    if (parameter.name == "main_loop")
      main_loop = parameter.value.double_value;
  }
  printf("updated parameters\n");
}

int main(int argc, char *argv[]) {
  printf("balance controller starting\n");
  // init ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("balance_robot_controller");

  node->declare_parameter("pid_roll.p", pid_param_roll.p);
  node->declare_parameter("pid_roll.i", pid_param_roll.i);
  node->declare_parameter("pid_roll.d", pid_param_roll.d);

  node->declare_parameter("pid_velocity.p", pid_param_v.p);
  node->declare_parameter("pid_velocity.i", pid_param_v.i);
  node->declare_parameter("pid_velocity.d", pid_param_v.d);

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

  auto orientation_subscription = node->create_subscription<balance_robot_msgs::msg::Orientation>(
      "balance/orientation", 10, orientation_topic_callback);

  auto encoders_subscription = node->create_subscription<balance_robot_msgs::msg::Encoders>(
      "balance/encoders", 10, encoders_topic_callback);

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node);

  auto callback_handler =
      parameters_client->on_parameter_event(param_change_callback);

  // FIXME: PID values need to be proper
  PID pid_v = PID(-20, 20, 0.0001, 0.0001, 0);
  PID pid_roll = PID(0,0, 15, 50, 0.016);

  float setpoint_roll = 0.2;
  float setpoint_velocity = 0;
  float dtsum = 0;
  float roll = 0;
  float motor_increment = 0;

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  bool missing_measurements = false;

  while (rclcpp::ok()) {
    if (orientation_measurements.size() == 0 && encoders_measurements.size() == 0) {
        if (!missing_measurements) 
          printf("measurements missing\n");
        missing_measurements = true;
        continue;
    }
    else {
        if (missing_measurements)
          printf("recoverd\n");
        missing_measurements = false;
    }

    orientation orientation_measurement{0,0,0,0};
    {
        int number_measurements = 0;
        while (!orientation_measurements.empty()) {
            orientation popped_orientation_measurement = orientation_measurements.back();
            orientation_measurements.pop_back();
            // Could use pitch and yaw here, too
            orientation_measurement.roll = (orientation_measurement.roll * 2 * number_measurements + popped_orientation_measurement.roll) / (2 * number_measurements + 1);
            orientation_measurement.pitch = (orientation_measurement.pitch * 2 * number_measurements + popped_orientation_measurement.pitch) / (2 * number_measurements + 1);
            orientation_measurement.yaw = (orientation_measurement.yaw * 2 * number_measurements + popped_orientation_measurement.yaw) / (2 * number_measurements + 1);
            orientation_measurement.dt += popped_orientation_measurement.dt;
            number_measurements ++;
        }
    }
    encoders encoders_measurement{0,0,0,0};
    {
        int number_measurements = 0;
        while (!orientation_measurements.empty()) {
            encoders popped_encoders_measurement = encoders_measurements.back();
            encoders_measurements.pop_back();

            encoders_measurement.velocity_left = (encoders_measurement.velocity_left * 2 * number_measurements + popped_encoders_measurement.velocity_left) / (2 * number_measurements + 1);
            encoders_measurement.velocity_right = (encoders_measurement.velocity_right * 2 * number_measurements + popped_encoders_measurement.velocity_right) / (2 * number_measurements + 1);
            if (number_measurements == 0) {
                encoders_measurement.position_left = popped_encoders_measurement.position_left;
                encoders_measurement.position_right = popped_encoders_measurement.position_right;
            }
            number_measurements ++;
        }
    }
    
    dtsum = orientation_measurement.dt;
    roll = orientation_measurement.roll;
    setpoint_velocity = velocity_cmd.forward * velocity_cmd.forward_gain;
    float measured_velocity = (encoders_measurement.velocity_left + encoders_measurement.velocity_right) / 2;

    // pid controllers
    setpoint_roll = pid_v.calculate(setpoint_velocity, measured_velocity, dtsum);
    motor_increment = pid_roll.calculate(setpoint_roll, roll, dtsum);

    float pwm_target = motor_increment;

    // ddd turning capabilities
    float pwm_target_left =
        pwm_target + (velocity_cmd.turn * velocity_cmd.turn_gain);
    float pwm_target_right =
        pwm_target - (velocity_cmd.turn * velocity_cmd.turn_gain);

    auto msg = std::make_unique<balance_robot_msgs::msg::Balance>();

    msg->header.frame_id = "robot";
    msg->header.stamp = ros_clock.now();

    msg->velocity.setpoint = setpoint_velocity;
    msg->velocity.measurement = measured_velocity;
    msg->velocity.increment = setpoint_roll;

    msg->roll.setpoint = setpoint_roll;
    msg->roll.measurement = roll;
    msg->roll.increment = motor_increment;

    msg->motor = pwm_target;
    msg->motor_left = pwm_target_left;
    msg->motor_right = pwm_target_right;

    balance_pub->publish(std::move(msg));

    rclcpp::spin_some(node);

    pid_roll.set(pid_param_roll.p, pid_param_roll.i, pid_param_roll.d);
    pid_v.set(pid_param_v.p, pid_param_v.i, pid_param_v.d);

    sleep(0.02); //FIXME: Just to get it running first
  }
}
