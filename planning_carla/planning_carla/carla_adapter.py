"""CARLA platform adapter: the only node that imports `carla`.

It turns CARLA into the same topics every other platform provides, so the planning stack
doesn't know (or care) that it is running in CARLA.

Owns the simulation clock: CARLA runs in synchronous mode and this node is the ONLY client
that calls world.tick(). Each tick it:
  1. applies the latest /drive command (speed + steering -> throttle/brake/steer, PID)
  2. ticks the world by fixed_delta_seconds
  3. waits for every sensor reading of that exact frame
  4. publishes /clock, sensors, odometry and ground truth, all stamped with simulation time

Publishes:
  /clock                      rosgraph_msgs/Clock
  /sensors/lidar/points       sensor_msgs/PointCloud2   (frame lidar)
  /sensors/imu                sensor_msgs/Imu           (frame imu)
  /sensors/gnss               sensor_msgs/NavSatFix     (frame gnss)
  /sensors/camera/image       sensor_msgs/Image         (frame camera, if enabled)
  /vehicle/odom               nav_msgs/Odometry         (twist only, noisy: "wheel odometry")
  /ground_truth/odom          nav_msgs/Odometry         (map frame; evaluation + ground_truth mode)
  /ground_truth/objects       av_msgs/TrackedObjectArray (other vehicles/walkers in range)
  /ground_truth/collision     geometry_msgs/Vector3Stamped (impulse; frame_id = other actor type)
  /goal_pose                  geometry_msgs/PoseStamped (latched, from the scenario's goal)
  /set_pose, /initialpose     geometry_msgs/PoseWithCovarianceStamped (initial pose for EKF/SLAM)
  /tf_static                  base_link -> each sensor
Subscribes:
  /drive                      ackermann_msgs/AckermannDriveStamped

Frames: map = CARLA world with y flipped (ROS convention). base_link = rear axle centre.
"""

import math
import queue
import threading
import time

import carla
import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from av_msgs.msg import TrackedObject, TrackedObjectArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from planning_carla.control import AckermannToCarla
from planning_carla.transforms import (
    euler_to_quaternion,
    imu_to_ros,
    lidar_to_ros,
    location_to_ros,
    rear_axle_pose,
    rotation_to_ros,
    world_velocity_to_body,
    yaw_rate_to_ros,
)

SENSOR_BLUEPRINTS = {
    "lidar": "sensor.lidar.ray_cast",
    "imu": "sensor.other.imu",
    "gnss": "sensor.other.gnss",
    "camera": "sensor.camera.rgb",
}


def stamp_from_seconds(t: float):
    from builtin_interfaces.msg import Time

    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class CarlaAdapter(Node):
    def __init__(self):
        super().__init__("carla_adapter")
        p = self.declare_parameter
        p("host", "localhost")
        p("port", 2000)
        p("tm_port", 8000)
        p("client_timeout", 30.0)  # s to wait for the CARLA server
        p("town", "")  # e.g. Town03; empty = keep the loaded map
        p("fixed_delta_seconds", 0.05)  # 20 Hz simulation step
        p("realtime_factor", 1.0)  # 1.0 = real time; 0 = as fast as possible
        p("no_rendering", False)
        p("ego_blueprint", "vehicle.lincoln.mkz_2020")
        p("spawn_index", 0)
        p("goal_index", -1)  # spawn point used as the route goal; -1 = none
        p("seed", 0)
        p("num_vehicles", 0)  # background traffic driven by the Traffic Manager
        p("parked_vehicle_indices", [-1])  # spawn points to put parked (static) cars on; -1 = none
        p("sensors_file", "")
        p("object_range", 80.0)
        p("odom_speed_noise", 0.05)
        p("odom_yaw_rate_noise", 0.005)
        p("drive_timeout", 0.5)  # s of simulation time without /drive -> brake
        p("follow_spectator", True)
        p("wait_for_localization", 5.0)  # s to wait for an EKF subscriber before ticking
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.rng = np.random.default_rng(gp("seed"))
        self.dt = float(gp("fixed_delta_seconds"))
        self.actors = []
        self.sensor_queues = {}
        self.latest_drive = None
        self.drive_rx_sim_time = -1.0
        self.sim_time = 0.0

        # --- connect and configure the world -------------------------------------------
        self.client = carla.Client(gp("host"), int(gp("port")))
        self.client.set_timeout(float(gp("client_timeout")))
        self.world = self.client.get_world()
        town = gp("town")
        if town and not self.world.get_map().name.endswith(town):
            self.get_logger().info(f"loading {town} ...")
            self.world = self.client.load_world(town)
        self.map = self.world.get_map()

        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        settings.no_rendering_mode = bool(gp("no_rendering"))
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = max(1, int(math.ceil(self.dt / 0.01)))
        self.world.apply_settings(settings)

        self.tm = self.client.get_trafficmanager(int(gp("tm_port")))
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(int(gp("seed")))

        # --- ego vehicle ---------------------------------------------------------------
        spawn_points = self.map.get_spawn_points()
        bp = self.world.get_blueprint_library().find(gp("ego_blueprint"))
        bp.set_attribute("role_name", "ego")
        self.ego = self.world.spawn_actor(bp, spawn_points[int(gp("spawn_index"))])
        self.actors.append(self.ego)
        self.world.tick()  # wheel positions in the physics control are valid after a tick
        self.wheelbase, self.rear_offset, max_steer = self.vehicle_geometry()
        self.get_logger().info(
            f"ego {gp('ego_blueprint')}: wheelbase {self.wheelbase:.3f} m, "
            f"max steer {math.degrees(max_steer):.1f} deg, rear axle {self.rear_offset:.3f} m from origin"
        )
        self.controller = AckermannToCarla(max_steer)

        # --- publishers / subscribers ----------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self.objects_pub = self.create_publisher(TrackedObjectArray, "/ground_truth/objects", 10)
        self.collision_pub = self.create_publisher(Vector3Stamped, "/ground_truth/collision", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", latched)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/sensors/lidar/points", 10)
        self.imu_pub = self.create_publisher(Imu, "/sensors/imu", 10)
        self.gnss_pub = self.create_publisher(NavSatFix, "/sensors/gnss", 10)
        self.image_pub = self.create_publisher(Image, "/sensors/camera/image", 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.static_tf = StaticTransformBroadcaster(self)

        self.goal_index = int(gp("goal_index"))
        self.parked_ids = set()
        self.spawn_sensors(gp("sensors_file"))
        self.spawn_parked([int(i) for i in gp("parked_vehicle_indices") if int(i) >= 0], spawn_points)
        self.spawn_traffic(int(gp("num_vehicles")), spawn_points, int(gp("spawn_index")))
        self.spawn_points = spawn_points

    # ------------------------------------------------------------------------------------
    def vehicle_geometry(self):
        """Wheelbase, rear-axle offset from the actor origin and max steer, from CARLA physics."""
        physics = self.ego.get_physics_control()
        inv = np.array(self.ego.get_transform().get_inverse_matrix())
        local = []
        for w in physics.wheels:  # positions are world coordinates in centimetres
            pos = np.array([w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0, 1.0])
            local.append(inv @ pos)
        front_x = 0.5 * (local[0][0] + local[1][0])
        rear_x = 0.5 * (local[2][0] + local[3][0])
        return front_x - rear_x, rear_x, math.radians(physics.wheels[0].max_steer_angle)

    def spawn_sensors(self, sensors_file: str):
        if not sensors_file:
            raise RuntimeError("parameter sensors_file is required")
        with open(sensors_file) as f:
            cfg = yaml.safe_load(f)
        library = self.world.get_blueprint_library()
        static = []
        for name, bp_id in SENSOR_BLUEPRINTS.items():
            sc = cfg.get(name, {})
            if not sc.get("enabled", False):
                continue
            bp = library.find(bp_id)
            attrs = dict(sc.get("attributes", {}))
            if name == "lidar":
                attrs["rotation_frequency"] = str(1.0 / self.dt)  # one full sweep per tick
            for k, v in attrs.items():
                bp.set_attribute(k, str(v))
            bp.set_attribute("sensor_tick", "0.0")  # every simulation step
            # base_link (ROS) -> actor origin (CARLA): shift by the rear-axle offset, flip y
            tf = carla.Transform(carla.Location(x=sc["x"] + self.rear_offset, y=-sc["y"], z=sc["z"]))
            sensor = self.world.spawn_actor(bp, tf, attach_to=self.ego)
            q = queue.Queue()
            sensor.listen(q.put)
            self.sensor_queues[name] = (q, sc["frame_id"])
            self.actors.append(sensor)
            static.append(self.static_transform(sc))

        if cfg.get("collision", {}).get("enabled", False):
            bp = library.find("sensor.other.collision")
            sensor = self.world.spawn_actor(bp, carla.Transform(), attach_to=self.ego)
            sensor.listen(self.on_collision)
            self.actors.append(sensor)
        self.static_tf.sendTransform(static)

    def static_transform(self, sc) -> TransformStamped:
        t = TransformStamped()
        t.header.frame_id = "base_link"
        t.child_frame_id = sc["frame_id"]
        t.transform.translation.x = float(sc["x"])
        t.transform.translation.y = float(sc["y"])
        t.transform.translation.z = float(sc["z"])
        t.transform.rotation.w = 1.0
        return t

    def spawn_parked(self, indices, spawn_points):
        """Static obstacles: cars on the given spawn points, handbrake on, no autopilot."""
        bp = self.world.get_blueprint_library().find("vehicle.audi.a2")
        for i in indices:
            car = self.world.try_spawn_actor(bp, spawn_points[i])
            if car is None:
                self.get_logger().warn(f"spawn point {i} is occupied; parked car skipped")
                continue
            car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
            self.actors.append(car)
            self.parked_ids.add(car.id)

    def spawn_traffic(self, n: int, spawn_points, ego_index: int):
        if n <= 0:
            return
        library = self.world.get_blueprint_library().filter("vehicle.*")
        library = [b for b in library if int(b.get_attribute("number_of_wheels")) == 4]
        used = {ego_index, *[int(i) for i in self.get_parameter("parked_vehicle_indices").value]}
        if self.goal_index >= 0:
            used.add(self.goal_index)
        candidates = [sp for i, sp in enumerate(spawn_points) if i not in used]
        self.rng.shuffle(candidates)
        batch = []
        for sp in candidates[:n]:
            bp = library[int(self.rng.integers(len(library)))]
            bp.set_attribute("role_name", "autopilot")
            batch.append(
                carla.command.SpawnActor(bp, sp).then(
                    carla.command.SetAutopilot(carla.command.FutureActor, True, self.tm.get_port())
                )
            )
        ids = [r.actor_id for r in self.client.apply_batch_sync(batch, True) if not r.error]
        self.actors.extend(self.world.get_actors(ids))
        self.get_logger().info(f"spawned {len(ids)} traffic vehicles")

    # ------------------------------------------------------------------------------------
    def on_drive(self, msg: AckermannDriveStamped):
        self.latest_drive = msg.drive
        self.drive_rx_sim_time = self.sim_time

    def on_collision(self, event):
        msg = Vector3Stamped()
        msg.header.stamp = stamp_from_seconds(event.timestamp)
        msg.header.frame_id = event.other_actor.type_id
        i = event.normal_impulse
        msg.vector.x, msg.vector.y, msg.vector.z = i.x, -i.y, i.z
        self.collision_pub.publish(msg)

    def ego_state(self):
        tf = self.ego.get_transform()
        x, y, z = location_to_ros(tf.location.x, tf.location.y, tf.location.z)
        roll, pitch, yaw = rotation_to_ros(tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw)
        x, y = rear_axle_pose(x, y, yaw, self.rear_offset)
        v = self.ego.get_velocity()
        forward, left = world_velocity_to_body(v.x, v.y, yaw)
        yaw_rate = yaw_rate_to_ros(self.ego.get_angular_velocity().z)
        return x, y, z, roll, pitch, yaw, forward, left, yaw_rate

    def apply_control(self):
        stale = self.latest_drive is None or self.sim_time - self.drive_rx_sim_time > float(
            self.get_parameter("drive_timeout").value
        )
        _, _, _, _, _, _, forward, _, _ = self.ego_state()
        if stale:
            self.controller.reset()
            self.ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            return
        throttle, brake, steer = self.controller(
            self.latest_drive.speed, self.latest_drive.steering_angle, forward, self.dt
        )
        self.ego.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))

    # ------------------------------------------------------------------------------------
    def publish_initial_pose(self, stamp):
        x, y, z, roll, pitch, yaw, *_ = self.ego_state()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        q = euler_to_quaternion(0.0, 0.0, yaw)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01
        cov[35] = 0.001
        msg.pose.covariance = cov
        self.set_pose_pub.publish(msg)  # robot_localization: start odom frame at the true pose
        self.initialpose_pub.publish(msg)  # slam_toolbox localization mode

    def publish_goal(self, stamp):
        if self.goal_index < 0:
            return
        tf = self.spawn_points[self.goal_index]
        x, y, _ = location_to_ros(tf.location.x, tf.location.y, tf.location.z)
        _, _, yaw = rotation_to_ros(0.0, 0.0, tf.rotation.yaw)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = x, y
        q = euler_to_quaternion(0.0, 0.0, yaw)
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = q
        self.goal_pub.publish(msg)
        self.get_logger().info(f"goal: spawn point {self.goal_index} at ({x:.1f}, {y:.1f})")

    def publish_vehicle(self, stamp):
        x, y, z, roll, pitch, yaw, forward, left, yaw_rate = self.ego_state()
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = "map"
        gt.child_frame_id = "base_link"
        gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = x, y, z
        q = euler_to_quaternion(roll, pitch, yaw)
        o = gt.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        gt.twist.twist.linear.x, gt.twist.twist.linear.y = forward, left
        gt.twist.twist.angular.z = yaw_rate
        self.gt_pub.publish(gt)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = forward + self.rng.normal(
            0.0, self.get_parameter("odom_speed_noise").value
        )
        odom.twist.twist.angular.z = yaw_rate + self.rng.normal(
            0.0, self.get_parameter("odom_yaw_rate_noise").value
        )
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.01, 0.01, 0.001
        odom.twist.covariance = cov
        self.odom_pub.publish(odom)

        if self.get_parameter("follow_spectator").value:
            tf = self.ego.get_transform()
            fwd = tf.get_forward_vector()
            v = tf.location - fwd * 10.0  # Location - Vector3D gives a Vector3D
            loc = carla.Location(x=v.x, y=v.y, z=v.z + 5.0)
            self.world.get_spectator().set_transform(
                carla.Transform(loc, carla.Rotation(pitch=-20.0, yaw=tf.rotation.yaw))
            )

    def publish_objects(self, stamp):
        ego_loc = self.ego.get_location()
        rng = float(self.get_parameter("object_range").value)
        out = TrackedObjectArray()
        out.header.stamp = stamp
        out.header.frame_id = "map"
        for actor in self.world.get_actors():
            tid = actor.type_id
            if actor.id == self.ego.id or not (tid.startswith("vehicle.") or tid.startswith("walker.")):
                continue
            loc = actor.get_location()
            if loc.distance(ego_loc) > rng:
                continue
            tf = actor.get_transform()
            centre = tf.transform(actor.bounding_box.location)
            o = TrackedObject()
            o.id = actor.id
            o.classification = (
                TrackedObject.CLASS_PEDESTRIAN if tid.startswith("walker.") else TrackedObject.CLASS_CAR
            )
            o.confidence = 1.0
            o.pose.position.x, o.pose.position.y, o.pose.position.z = location_to_ros(
                centre.x, centre.y, centre.z
            )
            _, _, yaw = rotation_to_ros(0.0, 0.0, tf.rotation.yaw)
            q = euler_to_quaternion(0.0, 0.0, yaw)
            o.pose.orientation.x, o.pose.orientation.y, o.pose.orientation.z, o.pose.orientation.w = q
            v = actor.get_velocity()
            o.velocity.x, o.velocity.y = v.x, -v.y
            e = actor.bounding_box.extent
            o.size.x, o.size.y, o.size.z = 2 * e.x, 2 * e.y, 2 * e.z
            out.objects.append(o)
        self.objects_pub.publish(out)

    def publish_sensors(self, frame: int, stamp):
        for name, (q, frame_id) in self.sensor_queues.items():
            data = None
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    item = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if item.frame == frame:
                    data = item
                    break
            if data is None:
                self.get_logger().warn(f"{name}: no data for frame {frame}", throttle_duration_sec=5.0)
                continue
            getattr(self, f"publish_{name}")(data, stamp, frame_id)

    def publish_lidar(self, data, stamp, frame_id):
        pts = lidar_to_ros(np.frombuffer(data.raw_data, dtype=np.float32))
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.height, msg.width = 1, pts.shape[0]
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z", "intensity"))
        ]
        msg.is_bigendian, msg.point_step = False, 16
        msg.row_step, msg.is_dense = 16 * pts.shape[0], True
        msg.data = pts.tobytes()
        self.lidar_pub.publish(msg)

    def publish_imu(self, data, stamp, frame_id):
        a, g = imu_to_ros(
            (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z),
            (data.gyroscope.x, data.gyroscope.y, data.gyroscope.z),
        )
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
        r = data.transform.rotation
        q = euler_to_quaternion(*rotation_to_ros(r.roll, r.pitch, r.yaw))
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.imu_pub.publish(msg)

    def publish_gnss(self, data, stamp, frame_id):
        msg = NavSatFix()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.status.status, msg.status.service = NavSatStatus.STATUS_FIX, NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude, msg.altitude = data.latitude, data.longitude, data.altitude
        msg.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gnss_pub.publish(msg)

    def publish_camera(self, data, stamp, frame_id):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.height, msg.width, msg.encoding = data.height, data.width, "bgra8"
        msg.is_bigendian, msg.step = 0, data.width * 4
        msg.data = bytes(data.raw_data)
        self.image_pub.publish(msg)

    # ------------------------------------------------------------------------------------
    def wait_for_localization(self):
        """Give the EKF time to start, then set its initial pose BEFORE the first sensor data.

        This makes the odom (and SLAM map) frame line up with CARLA's world frame, so the
        exported lane graph and the goal pose are valid in the localization frame too.
        """
        deadline = time.monotonic() + float(self.get_parameter("wait_for_localization").value)
        while time.monotonic() < deadline and self.set_pose_pub.get_subscription_count() == 0:
            time.sleep(0.1)
        stamp = stamp_from_seconds(self.world.get_snapshot().timestamp.elapsed_seconds)
        self.clock_pub.publish(Clock(clock=stamp))
        if self.set_pose_pub.get_subscription_count() > 0:
            for _ in range(3):
                self.publish_initial_pose(stamp)
                time.sleep(0.2)
            self.get_logger().info("initial pose sent to localization")

    def run(self):
        self.wait_for_localization()
        first = True
        period = self.dt / max(float(self.get_parameter("realtime_factor").value), 1e-6)
        if float(self.get_parameter("realtime_factor").value) <= 0.0:
            period = 0.0
        while rclpy.ok():
            t0 = time.monotonic()
            self.apply_control()
            frame = self.world.tick()
            self.sim_time = self.world.get_snapshot().timestamp.elapsed_seconds
            stamp = stamp_from_seconds(self.sim_time)
            self.clock_pub.publish(Clock(clock=stamp))
            self.publish_vehicle(stamp)
            self.publish_sensors(frame, stamp)
            self.publish_objects(stamp)
            if first:
                self.publish_goal(stamp)
                first = False
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def shutdown(self):
        """Always leave the server usable: back to asynchronous mode, all our actors removed."""
        try:
            for a in self.actors:
                if hasattr(a, "stop") and a.type_id.startswith("sensor."):
                    a.stop()
            self.client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in self.actors])
            self.tm.set_synchronous_mode(False)
            self.world.apply_settings(self.original_settings)
            self.get_logger().info("CARLA restored to asynchronous mode, actors destroyed")
        except RuntimeError as exc:  # server already gone
            self.get_logger().warn(f"cleanup failed: {exc}")


def main():
    rclpy.init()
    node = CarlaAdapter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
