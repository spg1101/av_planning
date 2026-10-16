"""World model node (Member A, task P1.4 / P2.2).

In:  TF map->base_link, velocity topic (Odometry), /map, /tracking/objects
Out: /world/ego_state (Odometry in map, 50 Hz), /world/costmap (inflated grid),
     /world/obstacles (tracked objects + constant-velocity predictions)
"""

import numpy as np
import rclpy
from av_msgs.msg import TrackedObjectArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def inflate(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Mark every cell within radius of an occupied cell as occupied (100)."""
    occupied = grid >= 50
    out = grid.copy()
    h, w = grid.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            shifted = np.zeros_like(occupied)
            ys, yd = slice(max(dy, 0), h + min(dy, 0)), slice(max(-dy, 0), h + min(-dy, 0))
            xs, xd = slice(max(dx, 0), w + min(dx, 0)), slice(max(-dx, 0), w + min(-dx, 0))
            shifted[ys, xs] = occupied[yd, xd]
            out[shifted] = 100
    return out


class WorldModelNode(Node):
    def __init__(self):
        super().__init__("world_model")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("velocity_topic", "/odometry/filtered")
        self.declare_parameter("inflation_radius", 0.3)
        self.declare_parameter("prediction_horizon", 2.0)
        self.declare_parameter("prediction_dt", 0.2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Odometry, self.get_parameter("velocity_topic").value, lambda m: setattr(self, "velocity", m), 10
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(TrackedObjectArray, "/tracking/objects", self.on_objects, 10)

        self.ego_pub = self.create_publisher(Odometry, "/world/ego_state", 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/world/costmap", latched)
        self.obstacles_pub = self.create_publisher(TrackedObjectArray, "/world/obstacles", 10)
        self.create_timer(0.02, self.publish_ego)

    def publish_ego(self):
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except TransformException:
            return
        msg = Odometry()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = map_frame
        msg.child_frame_id = base_frame
        t = tf.transform
        msg.pose.pose.position.x = t.translation.x
        msg.pose.pose.position.y = t.translation.y
        msg.pose.pose.orientation = t.rotation
        if self.velocity is not None:
            msg.twist = self.velocity.twist  # body-frame twist, same in any parent frame
        self.ego_pub.publish(msg)

    def on_map(self, msg: OccupancyGrid):
        """TODO(member A): also mark live /sensors/scan hits and use a rolling local window."""
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        radius = int(np.ceil(self.get_parameter("inflation_radius").value / msg.info.resolution))
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = inflate(grid, radius).astype(np.int8).flatten().tolist()
        self.costmap_pub.publish(out)

    def on_objects(self, msg: TrackedObjectArray):
        horizon = self.get_parameter("prediction_horizon").value
        dt = self.get_parameter("prediction_dt").value
        steps = int(horizon / dt)
        for o in msg.objects:
            o.prediction_dt = float(dt)
            o.predicted_poses = []
            for k in range(1, steps + 1):
                p = Pose()
                p.position.x = o.pose.position.x + o.velocity.x * k * dt
                p.position.y = o.pose.position.y + o.velocity.y * k * dt
                p.orientation = o.pose.orientation
                o.predicted_poses.append(p)
        self.obstacles_pub.publish(msg)


def main():
    rclpy.init()
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
