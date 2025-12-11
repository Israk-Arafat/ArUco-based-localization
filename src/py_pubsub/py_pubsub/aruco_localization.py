#!/usr/bin/env python3
"""
ArUco-based Localization Node

This node subscribes to ArUco marker detections and publishes the robot's
pose in the map frame using TF transforms. It computes the robot's position
by inverting the camera-to-marker transform and using known marker positions.
"""

import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math


# Quaternion and rotation utilities (replaces scipy dependency)
def quat_multiply(q1, q2):
    """Multiply two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quat_conjugate(q):
    """Conjugate of quaternion [x, y, z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    # Convert to [w, x, y, z] for calculation
    qw, qx, qy, qz = q[3], q[0], q[1], q[2]
    vx, vy, vz = v[0], v[1], v[2]
    
    # Quaternion rotation: q * v * q^-1
    # Optimized version
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    
    return np.array([
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx
    ])


class ArucoLocalizer(Node):
    """
    Localizes robot using ArUco markers with known positions.
    
    Subscribes to: /aruco_detections (ArucoDetection)
    Publishes: /robot_pose (PoseWithCovarianceStamped)
    TF: map -> base_link
    """
    
    def __init__(self):
        super().__init__('aruco_localizer')
        
        print("=" * 60)
        print("ARUCO LOCALIZER STARTING UP")
        print("=" * 60)
        
        # Declare parameters
        self.declare_parameter('marker_map_file', 'marker_map.yaml')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('use_closest_marker', True)
        self.declare_parameter('max_marker_distance', 2.0)  # meters
        
        # Get parameters
        marker_map_file = self.get_parameter('marker_map_file').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.use_closest = self.get_parameter('use_closest_marker').value
        self.max_distance = self.get_parameter('max_marker_distance').value
        
        print(f"[INIT] Parameters loaded:")
        print(f"  - marker_map_file: {marker_map_file}")
        print(f"  - camera_frame: {self.camera_frame}")
        print(f"  - base_frame: {self.base_frame}")
        print(f"  - map_frame: {self.map_frame}")
        print(f"  - use_closest: {self.use_closest}")
        print(f"  - max_distance: {self.max_distance}")
        
        # Load marker map
        print(f"[INIT] Loading marker map...")
        self.marker_map = self.load_marker_map(marker_map_file)
        self.get_logger().info(f'Loaded {len(self.marker_map)} markers from map')
        print(f"[INIT] Marker map: {self.marker_map}")
        
        # TF setup
        print(f"[INIT] Setting up TF broadcaster and listener...")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print(f"[INIT] TF setup complete")
        
        # Subscribe to ArUco detections
        print(f"[INIT] Subscribing to /aruco_detections...")
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
        print(f"[INIT] Subscribed to /aruco_detections")
        
        # Publish pose with covariance
        print(f"[INIT] Creating publisher for /robot_pose...")
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot_pose',
            10
        )
        print(f"[INIT] Publisher created for /robot_pose")
        
        # Store latest robot pose
        self.latest_pose = None
        self.last_detection_time = None
        self.detection_count = 0
        
        self.get_logger().info('ArUco Localizer initialized')
        print("=" * 60)
        print("ARUCO LOCALIZER READY - Waiting for detections...")
        print("=" * 60)
    
    def load_marker_map(self, filename):
        """Load marker positions from YAML file."""
        marker_map = {}
        
        # Try to find file in package share directory
        try:
            pkg_share = get_package_share_directory('py_pubsub')
            config_path = os.path.join(pkg_share, 'config', filename)
            if not os.path.exists(config_path):
                # Try relative to workspace
                config_path = os.path.join(
                    os.path.dirname(__file__),
                    '../config',
                    filename
                )
        except:
            # Fallback to relative path
            config_path = os.path.join(
                os.path.dirname(__file__),
                '../config',
                filename
            )
        
        if not os.path.exists(config_path):
            self.get_logger().warn(f'Marker map file not found: {config_path}')
            self.get_logger().warn('Using default marker at origin')
            # Default: single marker at origin
            marker_map[0] = {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0]  # quaternion [x,y,z,w]
            }
            return marker_map
        
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'markers' in data:
                    for marker in data['markers']:
                        marker_id = marker['id']
                        marker_map[marker_id] = {
                            'position': marker['position'],
                            'orientation': marker.get('orientation', [0.0, 0.0, 0.0, 1.0])
                        }
            self.get_logger().info(f'Loaded marker map from: {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load marker map: {e}')
            # Use default
            marker_map[0] = {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0]
            }
        
        return marker_map
    
    def aruco_callback(self, msg):
        """Process ArUco detections and compute robot pose."""
        self.detection_count += 1
        
        # Only print every 50th empty detection to reduce spam
        if not msg.markers:
            if self.detection_count % 50 == 0:
                print(f"\n[CALLBACK #{self.detection_count}] No markers detected (printed every 50 callbacks)")
            return
        
        print(f"\n[CALLBACK #{self.detection_count}] Received ArUco detection message")
        print(f"  - Number of markers: {len(msg.markers)}")
        print(f"[CALLBACK #{self.detection_count}] Marker IDs detected: {[m.marker_id for m in msg.markers]}")
        
        self.last_detection_time = self.get_clock().now()
        
        # Filter markers to only those in our map
        valid_markers = [m for m in msg.markers if m.marker_id in self.marker_map]
        print(f"[CALLBACK #{self.detection_count}] Valid markers (in map): {[m.marker_id for m in valid_markers]}")
        
        if not valid_markers:
            self.get_logger().warn('No known markers detected - check marker_map.yaml')
            print(f"[CALLBACK #{self.detection_count}] WARNING: No markers from map detected!")
            print(f"  - Detected IDs: {[m.marker_id for m in msg.markers]}")
            print(f"  - Map has IDs: {list(self.marker_map.keys())}")
            return
        
        # Get camera to base_link transform
        print(f"[CALLBACK #{self.detection_count}] Looking up TF: {self.base_frame} -> {self.camera_frame}")
        try:
            camera_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            print(f"[CALLBACK #{self.detection_count}] TF lookup successful")
        except Exception as e:
            self.get_logger().warn(f'Could not get camera to base transform: {e}')
            print(f"[CALLBACK #{self.detection_count}] TF lookup failed, using identity")
            # Use identity if not available
            camera_to_base = TransformStamped()
            camera_to_base.transform.rotation.w = 1.0
        
        # Compute robot pose from each visible marker
        poses = []
        for marker in valid_markers:
            print(f"[CALLBACK #{self.detection_count}] Processing marker {marker.marker_id}")
            print(f"  - Position: [{marker.pose.position.x:.3f}, {marker.pose.position.y:.3f}, {marker.pose.position.z:.3f}]")
            try:
                robot_pose = self.compute_robot_pose_from_marker(
                    marker,
                    camera_to_base
                )
                if robot_pose is not None:
                    # Add distance as weight (closer = higher weight)
                    distance = np.linalg.norm([
                        marker.pose.position.x,
                        marker.pose.position.y,
                        marker.pose.position.z
                    ])
                    print(f"  - Distance: {distance:.3f} m")
                    if distance < self.max_distance:
                        poses.append((robot_pose, 1.0 / (distance + 0.1), marker.marker_id))
                        print(f"  - Added to poses (weight: {1.0 / (distance + 0.1):.3f})")
                    else:
                        print(f"  - Too far (>{self.max_distance}m), skipping")
            except Exception as e:
                self.get_logger().error(f'Error processing marker {marker.marker_id}: {e}')
                print(f"[CALLBACK #{self.detection_count}] ERROR processing marker {marker.marker_id}: {e}")
                import traceback
                traceback.print_exc()
        
        if not poses:
            print(f"[CALLBACK #{self.detection_count}] No valid poses computed")
            return
        
        print(f"[CALLBACK #{self.detection_count}] Computed {len(poses)} valid poses")
        
        # Choose pose estimation strategy
        if self.use_closest:
            # Use closest marker only
            poses.sort(key=lambda x: x[1], reverse=True)
            final_pose, weight, marker_id = poses[0]
            self.get_logger().info(f'Using marker {marker_id} for localization')
            print(f"[CALLBACK #{self.detection_count}] Using closest marker: {marker_id}")
        else:
            # Weighted average of all visible markers
            final_pose = self.weighted_average_poses(poses)
            print(f"[CALLBACK #{self.detection_count}] Using weighted average of {len(poses)} markers")
        
        print(f"[CALLBACK #{self.detection_count}] Final robot pose:")
        print(f"  - Position: {final_pose['position']}")
        print(f"  - Orientation: {final_pose['orientation']}")
        
        self.latest_pose = final_pose
        
        # Publish transform
        print(f"[CALLBACK #{self.detection_count}] Publishing TF: {self.map_frame} -> {self.base_frame}")
        self.publish_transform(final_pose, msg.header.stamp)
        
        # Publish pose with covariance
        print(f"[CALLBACK #{self.detection_count}] Publishing /robot_pose")
        self.publish_pose(final_pose, msg.header.stamp)
        print(f"[CALLBACK #{self.detection_count}] SUCCESS - Pose published!")
    
    def compute_robot_pose_from_marker(self, marker, camera_to_base):
        """
        Compute robot pose in map frame from marker detection.
        
        Transform chain:
        map -> marker (known)
        marker -> camera (detected, need to invert)
        camera -> base_link (known from TF or static)
        
        Result: map -> base_link
        """
        marker_id = marker.marker_id
        marker_info = self.marker_map[marker_id]
        
        # Get marker position in map frame (known)
        marker_pos_map = np.array(marker_info['position'])
        marker_quat_map = np.array(marker_info['orientation'])  # [x,y,z,w]
        
        # Get camera position relative to marker (detected)
        camera_pos_marker = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z
        ])
        camera_quat_marker = np.array([
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ])
        
        # Invert: get marker position relative to camera
        # marker_rot_camera = camera_rot_marker^-1 (conjugate for unit quaternion)
        marker_quat_camera = quat_conjugate(camera_quat_marker)
        marker_pos_camera = quat_rotate_vector(marker_quat_camera, -camera_pos_marker)
        
        # Transform marker position to map frame
        camera_pos_map = quat_rotate_vector(marker_quat_map, marker_pos_camera) + marker_pos_map
        camera_quat_map = quat_multiply(marker_quat_map, marker_quat_camera)
        
        # Apply camera to base_link transform
        base_offset = np.array([
            camera_to_base.transform.translation.x,
            camera_to_base.transform.translation.y,
            camera_to_base.transform.translation.z
        ])
        base_quat = np.array([
            camera_to_base.transform.rotation.x,
            camera_to_base.transform.rotation.y,
            camera_to_base.transform.rotation.z,
            camera_to_base.transform.rotation.w
        ])
        
        # Final robot position in map frame
        robot_pos_map = camera_pos_map + quat_rotate_vector(camera_quat_map, base_offset)
        robot_quat_map = quat_multiply(camera_quat_map, base_quat)
        
        return {
            'position': robot_pos_map,
            'orientation': robot_quat_map
        }
    
    def weighted_average_poses(self, poses):
        """Average multiple pose estimates with weights."""
        total_weight = sum(w for _, w, _ in poses)
        
        # Average positions
        avg_pos = np.zeros(3)
        for pose, weight, _ in poses:
            avg_pos += np.array(pose['position']) * weight
        avg_pos /= total_weight
        
        # Average quaternions (simplified - just use highest weight for now)
        # Proper quaternion averaging requires more complex math
        poses.sort(key=lambda x: x[1], reverse=True)
        avg_quat = poses[0][0]['orientation']
        
        return {
            'position': avg_pos,
            'orientation': avg_quat
        }
    
    def publish_transform(self, pose, stamp):
        """Publish map -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = float(pose['position'][0])
        t.transform.translation.y = float(pose['position'][1])
        t.transform.translation.z = float(pose['position'][2])
        
        t.transform.rotation.x = float(pose['orientation'][0])
        t.transform.rotation.y = float(pose['orientation'][1])
        t.transform.rotation.z = float(pose['orientation'][2])
        t.transform.rotation.w = float(pose['orientation'][3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_pose(self, pose, stamp):
        """Publish robot pose with covariance."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        
        pose_msg.pose.pose.position.x = float(pose['position'][0])
        pose_msg.pose.pose.position.y = float(pose['position'][1])
        pose_msg.pose.pose.position.z = float(pose['position'][2])
        
        pose_msg.pose.pose.orientation.x = float(pose['orientation'][0])
        pose_msg.pose.pose.orientation.y = float(pose['orientation'][1])
        pose_msg.pose.pose.orientation.z = float(pose['orientation'][2])
        pose_msg.pose.pose.orientation.w = float(pose['orientation'][3])
        
        # Set covariance (diagonal matrix, higher values = more uncertainty)
        # Order: [x, y, z, rotation_x, rotation_y, rotation_z]
        covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        pose_msg.pose.covariance = covariance
        
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()