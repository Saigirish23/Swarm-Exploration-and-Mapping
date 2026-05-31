import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import SubmapQuery
import gzip
import numpy as np
from collections import Counter

class Checker(Node):
    def __init__(self):
        super().__init__('checker')
        self.cli = self.create_client(SubmapQuery, '/robot1/submap_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            pass
        req = SubmapQuery.Request()
        req.trajectory_id = 0
        req.submap_index = 0
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp.textures:
            print("No textures")
            return
        tex = resp.textures[0]
        cells = bytes(tex.cells)
        if len(cells) >= 2 and cells[0] == 0x1f and cells[1] == 0x8b:
            cells = gzip.decompress(cells)
        pixels = np.frombuffer(cells, dtype=np.uint8).reshape((-1, 2))
        counts = Counter(map(tuple, pixels))
        print("Unique (intensity, alpha) pairs:")
        for pair, count in counts.items():
            print(f"  {pair}: {count}")

rclpy.init()
c = Checker()
c.destroy_node()
rclpy.shutdown()
