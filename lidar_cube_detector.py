#!/usr/bin/env python3
"""
Detect a cube-like cluster from Isaac Sim /point_cloud.

This is a standalone ROS2 diagnostic node for the next perception step.
It does NOT command the robot yet.  It subscribes to:
  - /point_cloud (sensor_msgs/PointCloud2, fields x/y/z float32)
  - /tf          (tf2_msgs/TFMessage, direct world -> sensor transform)

It publishes/prints the estimated cube centroid in world coordinates.

Run:
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 "/home/user/Desktop/260527 KMS/lidar_cube_detector.py"
"""

import math
import json
import os
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


POINT_TOPIC = "/point_cloud"
TF_TOPIC = "/tf"
OUT_TOPIC = "/detected_cube"
OUT_JSON = "/tmp/lidar_cube_latest.json"

# Tune these after looking at the diagnostic printouts.
MIN_WORLD_Z = 0.32
MAX_WORLD_Z = 0.58
MAX_SENSOR_RANGE = 3.0
MIN_FORWARD = 0.35
MAX_FORWARD = 2.0
MAX_LATERAL_ABS = 0.40
SELF_FILTER_FORWARD = 0.55
SELF_FILTER_LATERAL_ABS = 0.32

# Grid clustering in world XY.  Small enough to split cube/stand from walls.
GRID_RESOLUTION = 0.06
MIN_CLUSTER_POINTS = 12
MAX_CLUSTER_POINTS = 2500
MIN_CLUSTER_Z_SIZE = 0.025
MIN_CLUSTER_XY_SIZE = 0.08

# Prefer objects in front of the sensor/Husky and not too far away.
MAX_CANDIDATE_XY_FROM_SENSOR = 2.2


def quat_to_rot_xyzw(x, y, z, w):
    """Return 3x3 rotation matrix from quaternion x,y,z,w."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def pointcloud2_xyz(msg):
    """Fast path for PointCloud2 with float32 x/y/z fields."""
    if msg.point_step < 12:
        return np.empty((0, 3), dtype=np.float32)
    count = msg.width * msg.height
    if count == 0:
        return np.empty((0, 3), dtype=np.float32)

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape((count, msg.point_step))
    xyz = np.empty((count, 3), dtype=np.float32)
    xyz[:, 0] = np.frombuffer(rows[:, 0:4].copy().tobytes(), dtype=np.float32)
    xyz[:, 1] = np.frombuffer(rows[:, 4:8].copy().tobytes(), dtype=np.float32)
    xyz[:, 2] = np.frombuffer(rows[:, 8:12].copy().tobytes(), dtype=np.float32)
    return xyz


def connected_components_grid(points_xy, resolution):
    """Simple 8-connected clustering on occupied XY grid cells."""
    cells = np.floor(points_xy / resolution).astype(np.int32)
    cell_to_indices = {}
    for i, c in enumerate(cells):
        key = (int(c[0]), int(c[1]))
        cell_to_indices.setdefault(key, []).append(i)

    visited = set()
    components = []
    for start in cell_to_indices:
        if start in visited:
            continue
        q = deque([start])
        visited.add(start)
        comp_cells = []
        while q:
            cur = q.popleft()
            comp_cells.append(cur)
            cx, cy = cur
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nxt = (cx + dx, cy + dy)
                    if nxt in cell_to_indices and nxt not in visited:
                        visited.add(nxt)
                        q.append(nxt)
        idxs = []
        for cell in comp_cells:
            idxs.extend(cell_to_indices[cell])
        components.append(np.array(idxs, dtype=np.int32))
    return components


class LidarCubeDetector(Node):
    def __init__(self):
        super().__init__("lidar_cube_detector")
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud_sub = self.create_subscription(PointCloud2, POINT_TOPIC, self.on_cloud, sensor_qos)
        self.tf_sub = self.create_subscription(TFMessage, TF_TOPIC, self.on_tf, 10)
        self.pub = self.create_publisher(PointStamped, OUT_TOPIC, 10)

        self.sensor_t_world = None
        self.sensor_R_world = None
        self.last_print = 0.0
        self.frame_count = 0
        self.get_logger().info(
            f"listening: {POINT_TOPIC}, {TF_TOPIC}; publishing estimated centroid: {OUT_TOPIC}"
        )

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "world" and tr.child_frame_id == "sensor":
                t = tr.transform.translation
                q = tr.transform.rotation
                self.sensor_t_world = np.array([t.x, t.y, t.z], dtype=np.float64)
                self.sensor_R_world = quat_to_rot_xyzw(q.x, q.y, q.z, q.w)

    def on_cloud(self, msg):
        self.frame_count += 1
        if self.sensor_t_world is None or self.sensor_R_world is None:
            if self.frame_count % 60 == 1:
                self.get_logger().warn("waiting for world -> sensor TF")
            return

        pts_sensor = pointcloud2_xyz(msg).astype(np.float64)
        if pts_sensor.size == 0:
            return
        finite = np.isfinite(pts_sensor).all(axis=1)
        pts_sensor = pts_sensor[finite]
        ranges = np.linalg.norm(pts_sensor, axis=1)
        pts_sensor = pts_sensor[ranges < MAX_SENSOR_RANGE]

        pts_world = (self.sensor_R_world @ pts_sensor.T).T + self.sensor_t_world
        z_mask = (pts_world[:, 2] >= MIN_WORLD_Z) & (pts_world[:, 2] <= MAX_WORLD_Z)
        near_mask = np.linalg.norm(pts_world[:, :2] - self.sensor_t_world[:2], axis=1) < MAX_CANDIDATE_XY_FROM_SENSOR
        forward_mask = (pts_sensor[:, 0] >= MIN_FORWARD) & (pts_sensor[:, 0] <= MAX_FORWARD)
        lateral_mask = np.abs(pts_sensor[:, 1]) <= MAX_LATERAL_ABS
        # Points this close and this far to the side are usually self-returns
        # from the robot/lidar mount, not the external cube/stand.
        self_mask = (pts_sensor[:, 0] < SELF_FILTER_FORWARD) & (
            np.abs(pts_sensor[:, 1]) > SELF_FILTER_LATERAL_ABS
        )
        candidate_mask = z_mask & near_mask & forward_mask & lateral_mask & (~self_mask)
        candidates = pts_world[candidate_mask]
        candidates_sensor = pts_sensor[candidate_mask]

        now = time.time()
        if len(candidates) < MIN_CLUSTER_POINTS:
            if now - self.last_print > 1.0:
                self.last_print = now
                print(
                    f"[lidar] frame={self.frame_count} raw={len(pts_world)} "
                    f"candidates={len(candidates)} no cluster"
                )
            return

        comps = connected_components_grid(candidates[:, :2], GRID_RESOLUTION)
        clusters = []
        for idxs in comps:
            n = len(idxs)
            if n < MIN_CLUSTER_POINTS or n > MAX_CLUSTER_POINTS:
                continue
            cluster = candidates[idxs]
            cluster_sensor = candidates_sensor[idxs]
            mins = cluster.min(axis=0)
            maxs = cluster.max(axis=0)
            size = maxs - mins
            if size[2] < MIN_CLUSTER_Z_SIZE and max(size[0], size[1]) < MIN_CLUSTER_XY_SIZE:
                continue
            centroid = cluster.mean(axis=0)
            centroid_sensor = cluster_sensor.mean(axis=0)
            xy_dist = float(np.linalg.norm(centroid[:2] - self.sensor_t_world[:2]))
            # Prefer compact object-like clusters at cube/stand top height.
            score = (
                0.15 * xy_dist
                + 2.0 * abs(float(centroid[2]) - 0.43)
                + 1.5 * abs(float(centroid_sensor[1]))
                + 0.3 * float(size[0] + size[1])
            )
            clusters.append((score, n, centroid, size, mins, maxs, centroid_sensor))

        if not clusters:
            if now - self.last_print > 1.0:
                self.last_print = now
                print(
                    f"[lidar] frame={self.frame_count} raw={len(pts_world)} "
                    f"candidates={len(candidates)} clusters=0"
                )
            return

        clusters.sort(key=lambda item: item[0])
        score, n, centroid, size, mins, maxs, centroid_sensor = clusters[0]

        msg_out = PointStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "world"
        grasp_point = np.array([centroid[0], centroid[1], maxs[2]], dtype=np.float64)
        msg_out.point.x = float(grasp_point[0])
        msg_out.point.y = float(grasp_point[1])
        msg_out.point.z = float(grasp_point[2])
        self.pub.publish(msg_out)
        self.write_json(grasp_point, n, size)

        if now - self.last_print > 0.5:
            self.last_print = now
            print(
                "[lidar] "
                f"raw={len(pts_world)} cand={len(candidates)} clusters={len(clusters)} "
                f"centroid=({centroid[0]:+.3f},{centroid[1]:+.3f},{centroid[2]:+.3f}) "
                f"target=({grasp_point[0]:+.3f},{grasp_point[1]:+.3f},{grasp_point[2]:+.3f}) "
                f"sensor=({centroid_sensor[0]:+.3f},{centroid_sensor[1]:+.3f},{centroid_sensor[2]:+.3f}) "
                f"size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f}) n={n} score={score:.3f}"
            )

    def write_json(self, centroid, n, size):
        payload = {
            "stamp": time.time(),
            "frame_id": "world",
            "x": float(centroid[0]),
            "y": float(centroid[1]),
            "z": float(centroid[2]),
            "points": int(n),
            "size_x": float(size[0]),
            "size_y": float(size[1]),
            "size_z": float(size[2]),
        }
        tmp = OUT_JSON + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        os.replace(tmp, OUT_JSON)


def main():
    rclpy.init()
    node = LidarCubeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
