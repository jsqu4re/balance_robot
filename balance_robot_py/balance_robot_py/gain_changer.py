import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor

from balance_robot_msgs.msg import Gains
from balance_robot_msgs.msg import States

import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.multioutput import MultiOutputRegressor

import pickle

class GainChanger(Node):
    def __init__(self):
        super().__init__('gain_changer')
        self.sub = self.create_subscription(States, 'balance/states', self.states_callback, 10)
        self.pub = self.create_publisher(Gains, 'balance/gains', 10)

    def states_callback(self, states_msg):
        gains_msg = Gains()
        gains_msg.header.frame_id = "robot_base_frame"
        gains_msg.header.stamp = self.get_clock().now().to_msg()
        state_x = [
            states_msg.x[0], # Calibration data
            states_msg.x[1],
            states_msg.x[2],
            states_msg.x[3],
            states_msg.x[4],
            states_msg.x[5],
            0  # abs(states_msg.x[2]) * 200
        ]
        gains_k = self.regressor.predict(np.array([state_x]))
        gains_msg.k = gains_k[0]
        self.pub.publish(gains_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        gain_changer_node = GainChanger()
        # fix absolute path
        pkl_filename = "/home/jsqu4re/workspace/videte_robot/simple_videte_wheel/regressor_gains.pkl"
        with open(pkl_filename, 'rb') as file:
            gain_changer_node.regressor = pickle.load(file)

        executor = SingleThreadedExecutor()
        executor.add_node(gain_changer_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            gain_changer_node.destroy_node()

    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()
