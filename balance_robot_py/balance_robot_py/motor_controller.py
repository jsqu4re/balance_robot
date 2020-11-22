from enum import IntEnum
import time

import odrive
from odrive.enums import *

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor

from balance_robot_msgs.msg import Encoders
from balance_robot_msgs.msg import MotorControl

class State(IntEnum):
    Init = 0
    Ready = 1
    Calibrated = 2
    Armed = 3
    Control = 4

def get_percentage(voltage: float, cells: int) -> float:
    # Battery percentage:
    # 3.7=0%
    # 3.8=20
    # 3.9=40
    # 4.0=60
    # 4.1=80
    # 4.2=100%
    cell_voltage = voltage / cells
    return (cell_voltage - 3.7) * 100 / (4.2 - 3.7)

class OdriveMotorManager(Node):
    def __init__(self):
        super().__init__('odrive_motor_manager')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter("target_state", State.Ready)
        self.declare_parameter("current_state", State.Init)
        self.current_state = State.Init
        self.target_state = State.Init

    def init_odrive(self):
        find_timeout = 20
        self.get_logger().info("try find an odrive .. timeout is " + str(find_timeout) + " seconds")
        try:
            self.balance_odrive = odrive.find_any(timeout=find_timeout)
        except KeyboardInterrupt:
            raise
        except Exception as err:
            self.get_logger().error("failed to find balance odrive .. sleep for " + str(find_timeout) + " seconds and retry")
            self.get_logger().error(str(err))
            time.sleep(find_timeout)

    def timer_callback(self):
        if self.current_state != self.target_state:
            self.get_logger().info("transition from " + str(self.current_state) + " to " + str(self.target_state))

        if self.current_state == State.Init and self.target_state >= State.Ready:
            try:
                self.init_odrive()
                while self.balance_odrive.axis0.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
                while self.balance_odrive.axis1.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
                self.current_state = State.Ready
            except Exception as err:
                self.get_logger().error("failed to initialize odrive: " + str(err) + " .. retry")
                self.current_state = State.Init

        if self.current_state >= State.Ready:
            try:
                # TODO: Should be published
                voltage = self.balance_odrive.vbus_voltage
                self.get_logger().info("bus voltage is " + "{:2.4f}".format(voltage) + "V -> " + "{:3.1f}".format(get_percentage(voltage, 3)) + "%")
            except Exception as err:
                self.get_logger().error("failed to receive data from balance odrive: " + str(err) + " .. restarting odrive")
                self.target_state = State.Init

        if self.current_state == State.Ready and self.target_state >= State.Calibrated:
            try:
                self.get_logger().info("starting calibration motor 0 ...")
                self.balance_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.balance_odrive.axis0.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
                self.get_logger().info("starting calibration motor 1 ...")
                self.balance_odrive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.balance_odrive.axis1.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
                self.get_logger().info("done")
                self.current_state = State.Calibrated
            except:
                self.current_state = State.Ready
        
        if self.current_state == State.Calibrated and self.target_state >= State.Armed:
            try:
                self.get_logger().info("starting closed loop control motor 0 ...")
                self.balance_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                while self.balance_odrive.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    time.sleep(0.1)
                self.get_logger().info("starting closed loop control motor 1 ...")
                self.balance_odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                while self.balance_odrive.axis1.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    time.sleep(0.1)
                self.get_logger().info("done")
                self.current_state = State.Armed
            except:
                self.current_state = State.Calibrated

        if self.current_state == State.Armed and self.target_state >= State.Control:
            self.current_state = State.Control

        if self.target_state < self.current_state:
            if self.target_state <= State.Calibrated:
                try:
                    self.balance_odrive.axis0.requested_state = AXIS_STATE_IDLE
                    self.balance_odrive.axis1.requested_state = AXIS_STATE_IDLE
                except Exception as err:
                    self.get_logger().error("unable to set AXIS_STATE_IDLE on balance odrive: " + str(err) + " .. restarting odrive")
                    self.target_state = State.Init
                
            if self.target_state <= State.Init:
                try:
                    self.balance_odrive.reboot()
                    self.target_state = State.Ready
                except Exception as err:
                    self.get_logger().error("unable to restart balance odrive: " + str(err) + " .. search for odrive")
                    self.target_state = State.Init
            self.current_state = self.target_state

        self.target_state = self.get_parameter("target_state").get_parameter_value().integer_value

        current_state_param = rclpy.parameter.Parameter(
            "current_state",
            rclpy.Parameter.Type.INTEGER,
            self.current_state
        )

        all_new_parameters = [current_state_param]
        self.set_parameters(all_new_parameters)

class OdriveMotorController(Node):
    def __init__(self, manager):
        super().__init__('odrive_motor_controller')
        self.manager = manager
        self.sub = self.create_subscription(MotorControl, 'balance/motors', self.motors_callback, 1)
        self.pub = self.create_publisher(Encoders, 'balance/encoders', 10)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.motor_acc = .0
        self.motor_acc_updated = False
        self.motor_vel_turn = .0
        self.tar_vel_0 = .0
        self.tar_vel_1 = .0

    def motors_callback(self, msg):
        self.motor_acc = msg.acceleration
        self.motor_vel_turn = msg.turn_velocity
        self.motor_acc_updated = True

    def timer_callback(self):
        if self.manager.current_state >= State.Ready:
            try:
                start_time = self.get_clock().now()

                pos_0 = self.manager.balance_odrive.axis0.encoder.pos_estimate
                pos_1 = self.manager.balance_odrive.axis1.encoder.pos_estimate
                vel_0 = self.manager.balance_odrive.axis0.encoder.vel_estimate
                vel_1 = self.manager.balance_odrive.axis1.encoder.vel_estimate

                vel_diff_0 = self.tar_vel_0 - vel_0
                vel_diff_1 = self.tar_vel_1 - vel_1

                if self.motor_acc_updated:
                    self.tar_vel_0 = vel_0 + self.motor_acc * self.timer_period # + self.motor_vel_turn
                    self.tar_vel_1 = vel_1 - self.motor_acc * self.timer_period # + self.motor_vel_turn
                    self.motor_acc_updated = False

                if self.manager.current_state == State.Control:
                    self.manager.balance_odrive.axis0.controller.vel_setpoint = self.tar_vel_0
                    self.manager.balance_odrive.axis1.controller.vel_setpoint = self.tar_vel_1
                else:
                    self.manager.balance_odrive.axis0.controller.vel_setpoint = .0
                    self.manager.balance_odrive.axis1.controller.vel_setpoint = .0

                msg = Encoders()
                msg.header.frame_id = "robot_base_frame"
                msg.header.stamp = start_time.to_msg()
                msg.encoder0.position = pos_0
                msg.encoder1.position = pos_1
                msg.encoder0.velocity = vel_0
                msg.encoder1.velocity = vel_1
                msg.encoder0.target_velocity = self.tar_vel_0
                msg.encoder1.target_velocity = self.tar_vel_1
                msg.encoder0.diff_velocity = vel_diff_0
                msg.encoder1.diff_velocity = vel_diff_1
                msg.dt = (self.get_clock().now() - start_time).nanoseconds / 1000000.0
                self.pub.publish(msg)

            except Exception as err:
                self.get_logger().error("failed to send or receive data from balance odrive: " + str(err) + " .. restarting odrive")
                self.manager.target_state = State.Init

def main(args=None):
    rclpy.init(args=args)

    try:
        manager_node = OdriveMotorManager()
        controller_node = OdriveMotorController(manager_node)
    
        executor = MultiThreadedExecutor()
        executor.add_node(manager_node)
        executor.add_node(controller_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            manager_node.destroy_node()
            controller_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
