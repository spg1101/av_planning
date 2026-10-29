"""World model node (Member A, task P1.4 / P2.2).

In:  TF map->base_link, velocity topic (Odometry), /sensors/scan, /tracking/objects
Out: /world/ego_state (Odometry in map, 50 Hz),
     /world/costmap (rolling grid around the car from the live scan, inflated by the car's radius),
     /world/obstacles (tracked objects + constant-velocity predictions)

The grid math lives in planning_core.world.costmap (unit-tested).
TODO(member A): ray-trace free space, merge the static SLAM map, keep hits for a short time.
"""

import numpy as np
import rclpy
from av_msgs.msg import TrackedObjectArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from planning_core.world.costmap import inflate, rolling_grid, scan_to_points, transform_points
from planning_nodes.conversions import quaternion_to_yaw


class WorldModelNode(Node):
    def __init__(self):
        super().__init__("world_model")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("velocity_topic", "/odometry/filtered")
        self.declare_parameter("inflation_radius", 0.3)  # m, about half the car's width
        self.declare_parameter("costmap_size", 10.0)  # m, side of the square window
        self.declare_parameter("costmap_resolution", 0.05)  # m per cell
        self.declare_parameter("costmap_period", 0.1)  # s between costmap updates
        self.declare_parameter("prediction_horizon", 2.0)
        self.declare_parameter("prediction_dt", 0.2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity = None

        self.last_costmap_time = None
        self.create_subscription(
            Odometry, self.get_parameter("velocity_topic").value, lambda m: setattr(self, "velocity", m), 10
        )
        self.create_subscription(LaserScan, "/sensors/scan", self.on_scan, 5)
        self.create_subscription(TrackedObjectArray, "/tracking/objects", self.on_objects, 10)

        self.ego_pub = self.create_publisher(Odometry, "/world/ego_state", 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/world/costmap", 1)
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

    def on_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        period = self.get_parameter("costmap_period").value
        if self.last_costmap_time is not None and (now - self.last_costmap_time).nanoseconds * 1e-9 < period:
            return
        map_frame = self.get_parameter("map_frame").value
        try:
            to_scan = self.tf_buffer.lookup_transform(map_frame, msg.header.frame_id, Time())
            to_base = self.tf_buffer.lookup_transform(
                map_frame, self.get_parameter("base_frame").value, Time()
            )
        except TransformException:
            return
        self.last_costmap_time = now
        t = to_scan.transform
        local = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        pts = transform_points(local, t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))
        res = self.get_parameter("costmap_resolution").value
        centre = (to_base.transform.translation.x, to_base.transform.translation.y)
        grid, (ox, oy) = rolling_grid(pts, centre, self.get_parameter("costmap_size").value, res)
        grid = inflate(grid, int(np.ceil(self.get_parameter("inflation_radius").value / res)))

        out = OccupancyGrid()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = map_frame
        out.info.resolution = float(res)
        out.info.width = out.info.height = grid.shape[0]
        out.info.origin.position.x, out.info.origin.position.y = float(ox), float(oy)
        out.info.origin.orientation.w = 1.0
        out.data = grid.flatten().tolist()
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
