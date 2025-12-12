import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
import math

# Constants
MAXVELDIST = 0.20
VMAX = 0.10
VMIN = 0.07
MINDIST = 0.10 # Distance to stop from the marker
Kp_linear = 1.0

OMAXANGLE = 25 * math.pi/180.
OMAX = 1.00
OMINANGLE = 5 * math.pi/180.
OMIN = 0.50
Kp_angular = 1.0

SEARCH_ROTATION_SPEED = 1.2

class PatrolMarkers(Node):
    def __init__(self):
        super().__init__('patrol_markers')
        self.sub = self.create_subscription(ArucoDetection, '/aruco_detections', self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        
        self.state = "SEARCHING" # SEARCHING or APPROACHING
        self.last_visited_id = -1
        self.target_id = -1
        
        # Search state variables
        self.search_state = "ROTATING"
        self.search_start_time = self.get_clock().now()
        self.rotation_duration = 1.0 # seconds to rotate
        self.stop_duration = 1.0 # seconds to stop and scan
                # Target tracking
        self.last_target_seen_time = self.get_clock().now()
        self.target_lost_threshold = 2.0 # seconds before giving up on target
                # Data storage
        self.latest_msg = None
        self.last_msg_time = self.get_clock().now()
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Patrol Markers Node Started. State: SEARCHING")

    def new_twist(self, linear_vel, ang_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = ang_vel
        return twist

    def on_aruco_detection(self, msg):
        self.latest_msg = msg
        self.last_msg_time = self.get_clock().now()

    def control_loop(self):
        # Check if data is stale (older than 1.0s)
        now = self.get_clock().now()
        time_since_last_msg = (now - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > 1.0:
            # Safety stop if no camera data
            # self.get_logger().warning("No camera data received for > 1s")
            self.pub.publish(self.new_twist(0.0, 0.0))
            return

        if self.latest_msg is None:
            return

        msg = self.latest_msg

        if self.state == "SEARCHING":
            # Check for new candidates
            candidates = []
            for m in msg.markers:
                if m.marker_id != self.last_visited_id:
                    candidates.append(m)
            
            if candidates:
                # Found a new marker!
                closest_marker = min(candidates, key=lambda m: m.pose.position.z)
                self.target_id = closest_marker.marker_id
                self.state = "APPROACHING"
                self.get_logger().info(f"Found new target: Marker {self.target_id}. Switching to APPROACHING.")
                self.pub.publish(self.new_twist(0.0, 0.0)) # Stop rotation immediately
            else:
                # No new markers, execute Rotate-Stop-Scan pattern
                elapsed = (now - self.search_start_time).nanoseconds / 1e9
                
                if self.search_state == "ROTATING":
                    if elapsed > self.rotation_duration:
                        self.search_state = "STOPPED"
                        self.search_start_time = now
                        self.pub.publish(self.new_twist(0.0, 0.0))
                    else:
                        self.pub.publish(self.new_twist(0.0, SEARCH_ROTATION_SPEED))
                elif self.search_state == "STOPPED":
                    if elapsed > self.stop_duration:
                        self.search_state = "ROTATING"
                        self.search_start_time = now
                        self.pub.publish(self.new_twist(0.0, SEARCH_ROTATION_SPEED))
                    else:
                        self.pub.publish(self.new_twist(0.0, 0.0))
                
        elif self.state == "APPROACHING":
            # Find the target marker in current frame
            target_marker = None
            for m in msg.markers:
                if m.marker_id == self.target_id:
                    target_marker = m
                    break
            
            if target_marker:
                self.last_target_seen_time = now
                z = target_marker.pose.position.z
                x = target_marker.pose.position.x
                
                self.get_logger().info(f"Approaching Marker {self.target_id}. Dist: {z:.3f}")
                
                if z < MINDIST:
                    # We arrived
                    self.get_logger().info(f"Arrived at Marker {self.target_id}. Switching to SEARCHING.")
                    self.last_visited_id = self.target_id
                    self.target_id = -1
                    self.state = "SEARCHING"
                    self.search_state = "ROTATING" # Reset search state
                    self.search_start_time = now
                    self.pub.publish(self.new_twist(0.0, 0.0))
                else:
                    # Drive towards it
                    angle = math.atan2(x, z)
                    
                    # Linear velocity control
                    linear_vel = (VMIN + Kp_linear * (VMAX - VMIN) * (z - MINDIST) / (MAXVELDIST - MINDIST))
                    
                    linear_vel_clipped = 0.0
                    if abs(linear_vel) > VMAX:
                        linear_vel_clipped = VMAX
                    elif abs(linear_vel) < VMIN:
                        if z > MINDIST:
                             linear_vel_clipped = VMIN
                        else:
                             linear_vel_clipped = 0.0
                    else:
                        linear_vel_clipped = linear_vel

                    # Angular velocity control
                    # Fix: Use abs(angle) for symmetric control
                    abs_angle = abs(angle)
                    mag_ang_vel = (OMIN + Kp_angular * (OMAX - OMIN) * (abs_angle - OMINANGLE) / (OMAXANGLE - OMINANGLE))
                    
                    # Apply sign
                    ang_vel = math.copysign(mag_ang_vel, angle)
                    
                    ang_vel_clipped = 0.0
                    if abs(ang_vel) > OMAX:
                        ang_vel_clipped = math.copysign(OMAX, ang_vel)
                    elif abs(ang_vel) < OMIN:
                         if abs(angle) < OMINANGLE:
                             ang_vel_clipped = 0.0
                         else:
                             ang_vel_clipped = math.copysign(OMIN, ang_vel)
                    else:
                        ang_vel_clipped = ang_vel
                        
                    self.pub.publish(self.new_twist(-linear_vel_clipped, ang_vel_clipped))
            else:
                # Target lost!
                # Check how long it has been lost
                time_since_seen = (now - self.last_target_seen_time).nanoseconds / 1e9
                
                if time_since_seen > self.target_lost_threshold:
                    self.get_logger().warning(f"Target Marker {self.target_id} lost for {time_since_seen:.1f}s! Switching back to SEARCHING.")
                    self.state = "SEARCHING"
                    self.search_state = "ROTATING"
                    self.search_start_time = now
                    self.target_id = -1
                    self.pub.publish(self.new_twist(0.0, 0.0))
                else:
                    # Wait for it to reappear
                    # self.get_logger().info(f"Target momentarily lost... waiting")
                    self.pub.publish(self.new_twist(0.0, 0.0))

def main(args=None):
    rclpy.init(args=args)
    node = PatrolMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
