import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomPublisher(Node):
    """
    Dead Reckoning Odometry Publisher for JetBot.
    
    This node subscribes to the velocity commands sent to the robot (/jetbot/cmd_vel)
    and integrates them over time to estimate the robot's position.
    
    This is a "blind" estimation (Dead Reckoning) and will drift over time,
    but it provides a continuous high-frequency pose estimate that is essential
    for sensor fusion (EKF) when combined with the lower-frequency, non-continuous
    ArUco localization.
    """
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()

        # Subscribers
        # We listen to the same topic the motors listen to
        self.subscription = self.create_subscription(
            Twist,
            '/jetbot/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for updating pose (20 Hz)
        self.create_timer(0.05, self.update_pose)
        
        self.get_logger().info('Dead Reckoning Odom Publisher Started')

    def cmd_vel_callback(self, msg):
        """Update current target velocity from cmd_vel"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_pose(self):
        """Integrate velocity to get position"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Compute odometry in a typical differential drive way
        # delta_x = (vx * cos(th)) * dt
        # delta_y = (vx * sin(th)) * dt
        delta_x = (self.vx * math.cos(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create Quaternion from Yaw
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. Publish Transform (odom -> base_link)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        # Set covariance (optional but good for EKF)
        # We are fairly confident in x, y, yaw, but less so as time goes on.
        # For now, we leave it zero or set small values.
        # robot_localization usually handles 0 covariance by ignoring it or using defaults,
        # but explicit covariance is better.
        
        # Twist
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
