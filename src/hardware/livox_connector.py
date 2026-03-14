import rclpy
from rclpy.node import Node

import threading
import queue
import logging

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


logging.basicConfig(
    filename='log/lidar.log',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
)


class LivoxConnection(Node):
    """
    ROS2 Livox Mid-360 connection.

    Example
    ```
    lidar = LivoxConnection()

    lidar.start()

    try:
        while True:
            points = lidar.get_scan()
            if points is not None:
                print(f"Got {len(points)} points")
    finally:
        lidar.stop()
    ```
    """

    def __init__(self, topic="/livox/lidar"):

        logging.info("Connecting to Livox lidar...")

        rclpy.init()

        super().__init__("livox_reader")

        self.topic = topic

        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self._callback,
            10
        )

        self.data_queue = queue.Queue(maxsize=10)

        self._stop_event = threading.Event()
        self._thread = None

        logging.info("Livox connection ready")

    def _callback(self, msg):

        points = []

        for p in pc2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        ):
            points.append(p)

        if self.data_queue.full():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                pass

        self.data_queue.put(points)

    def _spin(self):

        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def start(self):

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._spin,
            daemon=True
        )

        self._thread.start()

        logging.info("Livox scanning started")

    def stop(self):

        self._stop_event.set()

        if self._thread:
            self._thread.join()

        self.destroy_node()
        rclpy.shutdown()

        logging.info("Livox scanning stopped")

    def get_scan(self, timeout=1.0):

        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            return None
        