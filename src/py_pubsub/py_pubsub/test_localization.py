#!/usr/bin/env python3
"""
Localization System Monitor

This script monitors the ArUco localization system and displays:
- Detection rate (Hz)
- Current robot pose
- Detected markers
- TF tree status

Usage:
    ros2 run py_pubsub test_localization
    
Or run with Python directly:
    python3 test_localization.py
"""

import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import time
import sys


class LocalizationMonitor(Node):
    """Monitor and display localization system status."""
    
    def __init__(self):
        super().__init__('localization_monitor')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to detections and pose
        self.detection_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.detection_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # Status tracking
        self.detection_times = []
        self.pose_times = []
        self.last_markers = []
        self.last_pose = None
        self.start_time = time.time()
        
        # Status display timer
        self.timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info('Localization Monitor started')
        print("\n" + "="*60)
        print("ARUCO LOCALIZATION SYSTEM MONITOR")
        print("="*60)
        print("Monitoring topics:")
        print("  - /aruco_detections")
        print("  - /robot_pose")
        print("="*60 + "\n")
    
    def detection_callback(self, msg):
        """Track detection rate."""
        current_time = time.time()
        self.detection_times.append(current_time)
        
        # Keep only last 100 detections
        if len(self.detection_times) > 100:
            self.detection_times.pop(0)
        
        # Store detected markers
        self.last_markers = [m.marker_id for m in msg.markers]
    
    def pose_callback(self, msg):
        """Track pose updates."""
        current_time = time.time()
        self.pose_times.append(current_time)
        
        # Keep only last 100 poses
        if len(self.pose_times) > 100:
            self.pose_times.pop(0)
        
        self.last_pose = msg.pose.pose
    
    def calculate_rate(self, time_list):
        """Calculate rate from timestamp list."""
        if len(time_list) < 2:
            return 0.0
        
        time_span = time_list[-1] - time_list[0]
        if time_span < 0.01:  # Avoid division by zero
            return 0.0
        
        return (len(time_list) - 1) / time_span
    
    def display_status(self):
        """Display system status."""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Calculate rates
        detection_rate = self.calculate_rate(self.detection_times)
        pose_rate = self.calculate_rate(self.pose_times)
        
        # Clear terminal (works on Linux/Mac)
        print("\033[2J\033[H", end="")
        
        print("="*60)
        print(f"LOCALIZATION SYSTEM STATUS (Running: {elapsed:.1f}s)")
        print("="*60)
        
        print(f"\n📡 DETECTION RATE: {detection_rate:.1f} Hz")
        if detection_rate < 1.0:
            print("   ⚠️  WARNING: Detection rate is low!")
        elif detection_rate < 5.0:
            print("   ⚠️  Below target (5 Hz)")
        else:
            print("   ✅ Good detection rate")
        
        print(f"\n📍 POSE UPDATE RATE: {pose_rate:.1f} Hz")
        if pose_rate < 1.0:
            print("   ⚠️  WARNING: Pose update rate is low!")
        elif pose_rate < 5.0:
            print("   ⚠️  Below target (5 Hz)")
        else:
            print("   ✅ Good pose rate")
        
        print(f"\n🎯 DETECTED MARKERS: {self.last_markers if self.last_markers else 'None'}")
        if not self.last_markers:
            print("   ⚠️  No markers detected")
        
        if self.last_pose:
            print(f"\n🤖 ROBOT POSE:")
            print(f"   Position: ({self.last_pose.position.x:.3f}, "
                  f"{self.last_pose.position.y:.3f}, "
                  f"{self.last_pose.position.z:.3f})")
            print(f"   Orientation: ({self.last_pose.orientation.x:.3f}, "
                  f"{self.last_pose.orientation.y:.3f}, "
                  f"{self.last_pose.orientation.z:.3f}, "
                  f"{self.last_pose.orientation.w:.3f})")
        else:
            print(f"\n🤖 ROBOT POSE: Not available")
            print("   ⚠️  No pose estimates received")
        
        # Check TF tree
        print(f"\n🌳 TF TREE STATUS:")
        try:
            # Check map -> base_link
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            print("   ✅ map -> base_link transform available")
        except Exception as e:
            print(f"   ❌ map -> base_link: {str(e)}")
        
        try:
            # Check base_link -> camera_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            print("   ✅ base_link -> camera_link transform available")
        except Exception as e:
            print(f"   ❌ base_link -> camera_link: {str(e)}")
        
        print("\n" + "="*60)
        print("Press Ctrl+C to exit")
        print("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    monitor = LocalizationMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
