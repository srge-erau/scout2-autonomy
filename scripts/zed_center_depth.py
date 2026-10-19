#!/usr/bin/env python3

import math
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CenterDepth(Node):
    def __init__(self):
        super().__init__('zed_center_depth')
        self.last_print = 0.0
        self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Waiting for ZED X depth images...')

    def depth_callback(self, msg):
        now = time.monotonic()
        if now - self.last_print < 1.0:
            return
        self.last_print = now

        if msg.encoding != '32FC1':
            self.get_logger().warning('Unexpected encoding: ' + msg.encoding)
            return

        cx = msg.width // 2
        cy = msg.height // 2
        fmt = '>f' if msg.is_bigendian else '<f'
        distances = []

        for y in range(cy - 5, cy + 6):
            for x in range(cx - 5, cx + 6):
                offset = y * msg.step + x * 4
                distance = struct.unpack_from(fmt, msg.data, offset)[0]

                if math.isfinite(distance) and 0.3 <= distance <= 10.0:
                    distances.append(distance)

        if distances:
            measured = statistics.median(distances)
            self.get_logger().info(
                f'Center obstacle distance: {measured:.2f} meters'
            )
        else:
            self.get_logger().warning('No valid depth at image center')


def main():
    rclpy.init()
    node = CenterDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
