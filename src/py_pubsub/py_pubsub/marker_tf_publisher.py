#!/usr/bin/env python3
"""
Static Marker Transform Publisher

Reads marker positions from marker_map.yaml and publishes static transforms
from the map frame to each marker frame. This establishes the known marker
positions in the world.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class MarkerStaticTFPublisher(Node):
    """
    Publishes static TF transforms for known ArUco marker positions.
    
    TF: map -> marker_X for each marker in marker_map.yaml
    """
    
    def __init__(self):
        super().__init__('marker_static_tf_publisher')
        
        # Declare parameters
        self.declare_parameter('marker_map_file', 'marker_map.yaml')
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        marker_map_file = self.get_parameter('marker_map_file').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Static TF broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Load and publish marker transforms
        self.load_and_publish_markers(marker_map_file)
        
        self.get_logger().info('Marker Static TF Publisher initialized')
    
    def load_and_publish_markers(self, filename):
        """Load marker map and publish static transforms."""
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
            self.get_logger().error(f'Marker map file not found: {config_path}')
            self.get_logger().error('Cannot publish marker transforms without map file')
            return
        
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'markers' in data:
                    transforms = []
                    
                    for marker in data['markers']:
                        marker_id = marker['id']
                        position = marker['position']
                        orientation = marker.get('orientation', [0.0, 0.0, 0.0, 1.0])
                        
                        # Create transform
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = self.map_frame
                        t.child_frame_id = f'marker_{marker_id}'
                        
                        t.transform.translation.x = float(position[0])
                        t.transform.translation.y = float(position[1])
                        t.transform.translation.z = float(position[2])
                        
                        t.transform.rotation.x = float(orientation[0])
                        t.transform.rotation.y = float(orientation[1])
                        t.transform.rotation.z = float(orientation[2])
                        t.transform.rotation.w = float(orientation[3])
                        
                        transforms.append(t)
                        
                        self.get_logger().info(
                            f'Marker {marker_id}: pos=[{position[0]:.2f}, '
                            f'{position[1]:.2f}, {position[2]:.2f}]'
                        )
                    
                    # Publish all static transforms
                    self.tf_static_broadcaster.sendTransform(transforms)
                    self.get_logger().info(f'Published {len(transforms)} marker transforms')
                else:
                    self.get_logger().error('Invalid marker map format')
        except Exception as e:
            self.get_logger().error(f'Failed to load marker map: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MarkerStaticTFPublisher()
    
    try:
        # Keep node alive to maintain static transforms
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()