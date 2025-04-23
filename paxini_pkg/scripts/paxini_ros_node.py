#!/usr/bin/env python3
import numpy as np
import rclpy
from paxini_driver import Paxini
from paxini_pkg.msg import SensingData
from rclpy.node import Node


class PaxiniRos(Node):
    """
    A ROS Node to publish the tactile sensing values
    """

    def __init__(self):
        # hyper-parameters
        self.sensing_frequency = 60

        super().__init__("paxini_ros_node")
        self.device = Paxini(con_ids=[10])
        self.device.sensors_initialize()

        self.sensing_data_pub = self.create_publisher(SensingData, "paxini/sensing_data", 10)
        self.timer = self.create_timer(1.0 / self.sensing_frequency, self.timer_callback)

    def timer_callback(self):
        sensing_data = self.device.get_all_module_sensing_data().astype(np.float32)
        msg = SensingData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.shape = sensing_data.shape
        msg.data = sensing_data.flatten().tolist()

        self.sensing_data_pub.publish(msg)
        self.get_logger().info("Publishing sensing data")


def main(args=None):
    rclpy.init(args=args)
    paxini_ros = PaxiniRos()
    rclpy.spin(paxini_ros)


if __name__ == "__main__":
    main()
