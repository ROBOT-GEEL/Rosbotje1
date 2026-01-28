#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import yaml
import cv2
import numpy as np
import os

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')

        # Launch argument
        map_yaml_path = self.declare_parameter(
            'map_yaml',
            '/home/wheeltec/wheeltec_ros2/src/test_rt_package/map/WHEELTEC.yaml'
        ).get_parameter_value().string_value

        # YAML inlezen
        with open(map_yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # PGM/PNG pad bepalen
        map_dir = os.path.dirname(map_yaml_path)
        image_path = os.path.join(map_dir, cfg['image'])

        # Afbeelding inlezen via OpenCV (veilig!)
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Kon map niet laden: {image_path}")

        height, width = img.shape

        # OccupancyGrid data genereren
        data = []
        for v in img.flatten():
            if v == 255:
                data.append(0)      # free
            elif v == 0:
                data.append(100)    # occupied
            else:
                data.append(-1)     # unknown

        # QoS: Transient Local (latched)
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)

        # OccupancyGrid bericht opbouwen
        msg = OccupancyGrid()
        msg.header = Header(frame_id='map')
        msg.info.resolution = float(cfg['resolution'])
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(cfg['origin'][0])
        msg.info.origin.position.y = float(cfg['origin'][1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data

        self.msg = msg

        # Publiceer 1x per seconde (latched, dus blijft beschikbaar)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info(f"Static map publisher actief. Map: {image_path}")

    def publish_map(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

