#!/usr/bin/env python3
"""
Dynamic ArUco Marker Map Builder

This node dynamically discovers ArUco markers and builds a map of their positions.
The first marker detected becomes the origin of the map frame.
Subsequent markers are positioned relative to the origin marker based on the
robot's estimated pose when they are first detected.

Subscribes to: /aruco_detections (ArucoDetection)
Publishes: Static TF transforms for discovered markers
Saves: marker_map.yaml with discovered marker positions
"""

import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
import numpy as np
import yaml
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


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
    qw, qx, qy, qz = q[3], q[0], q[1], q[2]
    vx, vy, vz = v[0], v[1], v[2]
    
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    
    return np.array([
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx
    ])


class DynamicMapBuilder(Node):
    """
    Builds a map of ArUco markers dynamically as they are discovered.
    
    The first marker becomes the map origin. Subsequent markers are positioned
    based on the robot's pose estimate when they are first seen.
    """
    
    def __init__(self):
        super().__init__('dynamic_map_builder')
        
        self.get_logger().info('-' * 60)
        self.get_logger().info('DYNAMIC MAP BUILDER STARTING')
        
        # Declare parameters
        self.declare_parameter('marker_map_file', 'marker_map.yaml')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('auto_save', True)
        self.declare_parameter('min_observations', 3)  # Require multiple observations before adding marker
        self.declare_parameter('reset_map', False)     # Clear map on startup
        
        # Get parameters
        self.marker_map_file = self.get_parameter('marker_map_file').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.auto_save = self.get_parameter('auto_save').value
        self.min_observations = self.get_parameter('min_observations').value
        self.reset_map = self.get_parameter('reset_map').value
        
        # Force boolean conversion if it's a string (fixes launch file argument issues)
        if isinstance(self.reset_map, str):
            self.reset_map = self.reset_map.lower() == 'true'
            
        self.get_logger().info(f'DEBUG: reset_map parameter value is: {self.reset_map} (type: {type(self.reset_map)})')
        
        # Initialize map
        self.marker_map = {}  # {marker_id: {'position': [...], 'orientation': [...]}}
        self.origin_marker_id = None
        self.pending_markers = {}  # {marker_id: [(pos, orient), ...]} - observations before confirming
        
        # TF setup
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Visualization publisher
        self.viz_pub = self.create_publisher(MarkerArray, 'marker_map_viz', 10)
        self.viz_timer = self.create_timer(1.0, self.publish_map_visualization)
        
        # Load existing map if available
        self.load_existing_map()
        
        # Subscribe to ArUco detections
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
        
        # Keep track of current robot pose estimate (if available from localization)
        self.current_robot_pose = None
        
        self.get_logger().info('Dynamic Map Builder initialized')
        self.get_logger().info(f'Origin marker: {self.origin_marker_id}')
        self.get_logger().info(f'Known markers: {list(self.marker_map.keys())}')
        self.get_logger().info('=' * 60)
    
    def clear_visualization(self):
        """Clear all markers from RViz."""
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.viz_pub.publish(marker_array)

    def load_existing_map(self):
        """Load existing marker map if available."""
        config_path = self.get_config_path()
        
        if self.reset_map:
            self.get_logger().warn('RESET_MAP is True - Clearing existing map!')
            self.clear_visualization() # Clear RViz
            if os.path.exists(config_path):
                try:
                    # Overwrite with empty map
                    with open(config_path, 'w') as f:
                        f.write('# ArUco Marker Map Configuration - DYNAMIC SLAM MODE\n')
                        f.write('markers: []\n')
                    self.get_logger().info('Map file cleared.')
                except Exception as e:
                    self.get_logger().error(f'Failed to clear map file: {e}')
            return

        if not os.path.exists(config_path):
            self.get_logger().info('No existing map found - will create new map')
            return
        
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'markers' in data and data['markers']:
                    for marker in data['markers']:
                        marker_id = marker['id']
                        self.marker_map[marker_id] = {
                            'position': marker['position'],
                            'orientation': marker.get('orientation', [0.0, 0.0, 0.0, 1.0]),
                            'description': marker.get('description', ''),
                            'discovery_timestamp': marker.get('discovery_timestamp', '')
                        }
                    
                    # First marker in file is origin
                    self.origin_marker_id = data['markers'][0]['id']
                    
                    # Publish static transforms for loaded markers
                    self.publish_all_marker_transforms()
                    self.publish_map_visualization()
                    
                    self.get_logger().info(f'Loaded existing map with {len(self.marker_map)} markers')
                else:
                    self.get_logger().info('Map file exists but is empty - starting fresh')
        except Exception as e:
            self.get_logger().error(f'Error loading existing map: {e}')
    
    def get_config_path(self):
        """Get path to marker map configuration file."""
        try:
            pkg_share = get_package_share_directory('py_pubsub')
            config_path = os.path.join(pkg_share, 'config', self.marker_map_file)
            if os.path.exists(config_path):
                return config_path
        except:
            pass
        
        # Fallback to relative path
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../config',
            self.marker_map_file
        )
        return config_path
    
    def aruco_callback(self, msg):
        """Process ArUco detections and update map."""
        # self.get_logger().info(f'[MAP_BUILDER] Received message with {len(msg.markers)} markers')
        
        if not msg.markers:
            return
        
        # self.get_logger().info(f'[MAP_BUILDER] Current state: origin={self.origin_marker_id}, map has {len(self.marker_map)} markers')
        
        # Get camera to base transform
        try:
            camera_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            # Use identity if not available
            camera_to_base = TransformStamped()
            camera_to_base.transform.rotation.w = 1.0
        
        for marker in msg.markers:
            marker_id = marker.marker_id
            
            # Check if marker is already in map
            if marker_id in self.marker_map:
                # self.get_logger().info(f'[MAP_BUILDER] Marker {marker_id} already in map, skipping')
                continue
            
            self.get_logger().info(f'[MAP_BUILDER] Processing new marker {marker_id}')
            
            # Extract marker pose relative to camera
            marker_pos_camera = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
            marker_quat_camera = np.array([
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ])
            
            # First marker becomes origin
            if self.origin_marker_id is None:
                self.get_logger().info(f'[MAP_BUILDER] No origin yet - setting marker {marker_id} as origin!')
                self.add_origin_marker(marker_id, marker_pos_camera, marker_quat_camera)
            else:
                self.get_logger().info(f'[MAP_BUILDER] Marker {marker_id} is new, adding observation...')
                # Add to pending observations
                self.add_marker_observation(marker_id, marker_pos_camera, marker_quat_camera, camera_to_base)
    
    def add_origin_marker(self, marker_id, pos_camera, quat_camera):
        """Add the first marker as the origin of the map frame."""
        self.origin_marker_id = marker_id
        
        # Origin is at [0, 0, 0] with identity orientation
        self.marker_map[marker_id] = {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],
            'description': f'Origin marker (first detected)',
            'discovery_timestamp': datetime.now().isoformat()
        }
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓✓✓ Marker {marker_id} set as MAP ORIGIN ✓✓✓')
        self.get_logger().info('=' * 60)
        
        # Publish static transform
        self.publish_marker_transform(marker_id)
        self.publish_map_visualization()
        self.get_logger().info(f'Published static TF: map → marker_{marker_id}')
        
        # Save map
        if self.auto_save:
            self.save_map()
            self.get_logger().info(f'Saved origin marker to {self.get_config_path()}')
    
    def add_marker_observation(self, marker_id, pos_camera, quat_camera, camera_to_base):
        """Add observation of a new marker."""
        # If no origin marker yet, this should become the origin (handled by caller)
        # This function is only called for non-origin markers
        
        # Try to get current robot pose in map frame
        try:
            base_to_map = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Compute marker position in map frame
            # Chain: map -> base -> camera -> marker
            
            # Camera pose in base frame
            cam_offset_base = np.array([
                camera_to_base.transform.translation.x,
                camera_to_base.transform.translation.y,
                camera_to_base.transform.translation.z
            ])
            cam_quat_base = np.array([
                camera_to_base.transform.rotation.x,
                camera_to_base.transform.rotation.y,
                camera_to_base.transform.rotation.z,
                camera_to_base.transform.rotation.w
            ])
            
            # Base pose in map frame
            base_pos_map = np.array([
                base_to_map.transform.translation.x,
                base_to_map.transform.translation.y,
                base_to_map.transform.translation.z
            ])
            base_quat_map = np.array([
                base_to_map.transform.rotation.x,
                base_to_map.transform.rotation.y,
                base_to_map.transform.rotation.z,
                base_to_map.transform.rotation.w
            ])
            
            # Camera pose in map frame
            cam_pos_map = base_pos_map + quat_rotate_vector(base_quat_map, cam_offset_base)
            cam_quat_map = quat_multiply(base_quat_map, cam_quat_base)
            
            # Marker pose in map frame
            marker_pos_map = cam_pos_map + quat_rotate_vector(cam_quat_map, pos_camera)
            marker_quat_map = quat_multiply(cam_quat_map, quat_camera)
            
            # Add to pending observations
            if marker_id not in self.pending_markers:
                self.pending_markers[marker_id] = []
            
            self.pending_markers[marker_id].append((marker_pos_map, marker_quat_map))
            
            # Check if we have enough observations
            if len(self.pending_markers[marker_id]) >= self.min_observations:
                self.confirm_marker(marker_id)
            else:
                self.get_logger().info(
                    f'Marker {marker_id}: {len(self.pending_markers[marker_id])}/{self.min_observations} observations'
                )
        
        except Exception as e:
            # This is normal when we don't have localization yet (before origin marker is set)
            if self.origin_marker_id is None:
                # Silently ignore - waiting for origin marker
                pass
            else:
                # We have origin but can't get robot pose - this is a problem
                self.get_logger().warn(f'Cannot compute marker position: {e}')
    
    def confirm_marker(self, marker_id):
        """Confirm a marker after sufficient observations and add to map."""
        observations = self.pending_markers[marker_id]
        
        # Average the position observations
        positions = np.array([obs[0] for obs in observations])
        avg_position = np.mean(positions, axis=0)
        
        # Use first orientation (proper quaternion averaging is complex)
        avg_orientation = observations[0][1]
        
        # Compute standard deviation to check consistency
        std_dev = np.std(positions, axis=0)
        max_std = np.max(std_dev)
        
        if max_std > 0.1:  # 10cm threshold
            self.get_logger().warn(
                f'Marker {marker_id} observations inconsistent (std: {max_std:.3f}m) - need more data'
            )
            return
        
        # Add to map
        self.marker_map[marker_id] = {
            'position': avg_position.tolist(),
            'orientation': avg_orientation.tolist(),
            'description': f'Auto-discovered marker',
            'discovery_timestamp': datetime.now().isoformat()
        }
        
        self.get_logger().info(f'✓ Marker {marker_id} CONFIRMED and added to map')
        self.get_logger().info(f'  Position: [{avg_position[0]:.3f}, {avg_position[1]:.3f}, {avg_position[2]:.3f}]')
        
        # Remove from pending
        del self.pending_markers[marker_id]
        
        # Publish static transform
        self.publish_marker_transform(marker_id)
        self.publish_map_visualization()
        
        # Save map
        if self.auto_save:
            self.save_map()
    
    def publish_marker_transform(self, marker_id):
        """Publish static transform for a marker."""
        marker_info = self.marker_map[marker_id]
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = f'marker_{marker_id}'
        
        t.transform.translation.x = float(marker_info['position'][0])
        t.transform.translation.y = float(marker_info['position'][1])
        t.transform.translation.z = float(marker_info['position'][2])
        
        t.transform.rotation.x = float(marker_info['orientation'][0])
        t.transform.rotation.y = float(marker_info['orientation'][1])
        t.transform.rotation.z = float(marker_info['orientation'][2])
        t.transform.rotation.w = float(marker_info['orientation'][3])
        
        self.tf_static_broadcaster.sendTransform(t)
    
    def publish_all_marker_transforms(self):
        """Publish static transforms for all known markers."""
        transforms = []
        for marker_id in self.marker_map.keys():
            marker_info = self.marker_map[marker_id]
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = f'marker_{marker_id}'
            
            t.transform.translation.x = float(marker_info['position'][0])
            t.transform.translation.y = float(marker_info['position'][1])
            t.transform.translation.z = float(marker_info['position'][2])
            
            t.transform.rotation.x = float(marker_info['orientation'][0])
            t.transform.rotation.y = float(marker_info['orientation'][1])
            t.transform.rotation.z = float(marker_info['orientation'][2])
            t.transform.rotation.w = float(marker_info['orientation'][3])
            
            transforms.append(t)
        
        if transforms:
            self.tf_static_broadcaster.sendTransform(transforms)
            
    def publish_map_visualization(self):
        """Publish visualization markers for RViz."""
        marker_array = MarkerArray()
        
        for marker_id, marker_info in self.marker_map.items():
            # 1. White border
            border = Marker()
            border.header.frame_id = self.map_frame
            border.header.stamp = self.get_clock().now().to_msg()
            border.ns = "aruco_borders"
            border.id = int(marker_id)
            border.type = Marker.CUBE
            border.action = Marker.ADD
            
            border.pose.position.x = float(marker_info['position'][0])
            border.pose.position.y = float(marker_info['position'][1])
            border.pose.position.z = float(marker_info['position'][2])
            
            border.pose.orientation.x = float(marker_info['orientation'][0])
            border.pose.orientation.y = float(marker_info['orientation'][1])
            border.pose.orientation.z = float(marker_info['orientation'][2])
            border.pose.orientation.w = float(marker_info['orientation'][3])
            
            # Make it look like a flat tile
            border.scale.x = 0.07  # 7cm (slightly larger than 5cm marker)
            border.scale.y = 0.07
            border.scale.z = 0.002 # Very thin
            
            border.color.r = 1.0
            border.color.g = 1.0
            border.color.b = 1.0
            border.color.a = 1.0
            border.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(border)

            # 2. Green inner square (the "marker")
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "aruco_markers"
            marker.id = int(marker_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(marker_info['position'][0])
            marker.pose.position.y = float(marker_info['position'][1])
            marker.pose.position.z = float(marker_info['position'][2])
            
            marker.pose.orientation.x = float(marker_info['orientation'][0])
            marker.pose.orientation.y = float(marker_info['orientation'][1])
            marker.pose.orientation.z = float(marker_info['orientation'][2])
            marker.pose.orientation.w = float(marker_info['orientation'][3])
            
            marker.scale.x = 0.05  # 5cm marker
            marker.scale.y = 0.05
            marker.scale.z = 0.004 # Slightly thicker than border to pop out
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            
            # 3. Text ID
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "aruco_ids"
            text_marker.id = int(marker_id)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(marker_info['position'][0])
            text_marker.pose.position.y = float(marker_info['position'][1])
            text_marker.pose.position.z = float(marker_info['position'][2]) + 0.1
            
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.05
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"ID: {marker_id}"
            text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(text_marker)
            
        self.viz_pub.publish(marker_array)
    
    def save_map(self):
        """Save current marker map to YAML file."""
        config_path = self.get_config_path()
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        
        # Create YAML data
        data = {
            'markers': []
        }
        
        # Add origin marker first
        if self.origin_marker_id is not None:
            marker_info = self.marker_map[self.origin_marker_id]
            data['markers'].append({
                'id': self.origin_marker_id,
                'position': marker_info['position'],
                'orientation': marker_info['orientation'],
                'description': marker_info['description'],
                'discovery_timestamp': marker_info['discovery_timestamp']
            })
        
        # Add other markers
        for marker_id, marker_info in self.marker_map.items():
            if marker_id != self.origin_marker_id:
                data['markers'].append({
                    'id': marker_id,
                    'position': marker_info['position'],
                    'orientation': marker_info['orientation'],
                    'description': marker_info['description'],
                    'discovery_timestamp': marker_info['discovery_timestamp']
                })
        
        try:
            with open(config_path, 'w') as f:
                f.write('# ArUco Marker Map Configuration - DYNAMIC SLAM MODE\n')
                f.write('# This file is automatically updated by the map_builder node\n')
                f.write('# \n')
                f.write(f'# Last updated: {datetime.now().isoformat()}\n')
                f.write(f'# Origin marker: {self.origin_marker_id}\n')
                f.write(f'# Total markers: {len(self.marker_map)}\n')
                f.write('\n')
                yaml.dump(data, f, default_flow_style=False)
            
            self.get_logger().info(f'Map saved to {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DynamicMapBuilder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save map on exit
        if node.auto_save and node.marker_map:
            node.save_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
