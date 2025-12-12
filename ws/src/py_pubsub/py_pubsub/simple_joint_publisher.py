import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Joint State Publisher started')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [0.0, 0.0]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
