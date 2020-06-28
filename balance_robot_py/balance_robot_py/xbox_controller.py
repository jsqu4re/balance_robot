import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor

from balance_robot_msgs.msg import Motors
from sensor_msgs.msg import Joy

class XboxController(Node):
    def __init__(self):
        super().__init__('xbox_controller')
        self.sub = self.create_subscription(Joy, 'joy', self.xbox_callback, 10)
        self.pub = self.create_publisher(Motors, 'balance/motors', 10)

    def xbox_callback(self, joy_msg):
        self.get_logger().info("hello world")

        motors_msg = Motors()
        motors_msg.header.frame_id = "robot_base_frame"
        motors_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.pub.publish(motors_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        controller_node = XboxController()
    
        executor = SingleThreadedExecutor()
        executor.add_node(controller_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller_node.destroy_node()

    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()
