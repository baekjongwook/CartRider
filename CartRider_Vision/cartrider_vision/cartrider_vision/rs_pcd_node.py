#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener, TransformException


class RSPCDNode(Node):
    def __init__(self):
        super().__init__("rs_pcd_node")

        self.declare_parameter("input_cloud_topic", "camera/depth/color/points")
        self.declare_parameter("output_cloud_topic", "camera/depth/voxel_cloud")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("voxel_size", 0.05)

        self.declare_parameter("crop_min_x", 0.2)
        self.declare_parameter("crop_max_x", 2.5)
        self.declare_parameter("crop_min_y", -1.0)
        self.declare_parameter("crop_max_y", 1.0)
        self.declare_parameter("crop_min_z", 0.05)
        self.declare_parameter("crop_max_z", 1.5)

        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.output_cloud_topic = self.get_parameter("output_cloud_topic").value

        self.target_frame = self.get_parameter("target_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)

        self.crop_min_x = float(self.get_parameter("crop_min_x").value)
        self.crop_max_x = float(self.get_parameter("crop_max_x").value)
        self.crop_min_y = float(self.get_parameter("crop_min_y").value)
        self.crop_max_y = float(self.get_parameter("crop_max_y").value)
        self.crop_min_z = float(self.get_parameter("crop_min_z").value)
        self.crop_max_z = float(self.get_parameter("crop_max_z").value)

        self.crop_min_x, self.crop_max_x = sorted([self.crop_min_x, self.crop_max_x])
        self.crop_min_y, self.crop_max_y = sorted([self.crop_min_y, self.crop_max_y])
        self.crop_min_z, self.crop_max_z = sorted([self.crop_min_z, self.crop_max_z])

        if self.voxel_size <= 0.0:
            self.voxel_size = 0.05

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_cloud_topic,
            10,
        )

        self.get_logger().info("rs_pcd_node started.")
        self.get_logger().info(f"Input cloud topic  : {self.input_cloud_topic}")
        self.get_logger().info(f"Output cloud topic : {self.output_cloud_topic}")
        self.get_logger().info(f"Target frame       : {self.target_frame}")
        self.get_logger().info(f"Voxel size         : {self.voxel_size:.3f}")
        self.get_logger().info(
            "Crop box           : "
            f"x[{self.crop_min_x:.2f}, {self.crop_max_x:.2f}], "
            f"y[{self.crop_min_y:.2f}, {self.crop_max_y:.2f}], "
            f"z[{self.crop_min_z:.2f}, {self.crop_max_z:.2f}]"
        )

    def cloud_callback(self, msg: PointCloud2):
        if not msg.header.frame_id:
            self.get_logger().warn("Input PointCloud2 frame_id is empty.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF failed: {msg.header.frame_id} -> {self.target_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return

        points = self.pointcloud2_to_xyz_array(msg)

        if points is None or points.shape[0] == 0:
            return

        points = self.transform_points(points, transform)
        points = self.crop_points(points)

        if points.shape[0] == 0:
            return

        points = self.voxel_downsample(points)

        if points.shape[0] == 0:
            return

        out_msg = point_cloud2.create_cloud_xyz32(
            header=msg.header,
            points=points.astype(np.float32).tolist(),
        )

        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = self.target_frame

        self.cloud_pub.publish(out_msg)

    def pointcloud2_to_xyz_array(self, msg: PointCloud2):
        try:
            arr = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except Exception as e:
            self.get_logger().warn(
                f"PointCloud2 read failed: {e}",
                throttle_duration_sec=1.0,
            )
            return None

        if arr is None or arr.size == 0:
            return None

        points = np.asarray(arr, dtype=np.float32)

        if points.ndim == 1:
            points = points.reshape(-1, 3)

        valid = np.isfinite(points).all(axis=1)
        points = points[valid]

        return points

    def transform_points(self, points: np.ndarray, transform):
        t = transform.transform.translation
        q = transform.transform.rotation

        rotation = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)

        return points @ rotation.T + translation

    def crop_points(self, points: np.ndarray):
        mask = (
            (points[:, 0] >= self.crop_min_x)
            & (points[:, 0] <= self.crop_max_x)
            & (points[:, 1] >= self.crop_min_y)
            & (points[:, 1] <= self.crop_max_y)
            & (points[:, 2] >= self.crop_min_z)
            & (points[:, 2] <= self.crop_max_z)
        )

        return points[mask]

    def voxel_downsample(self, points: np.ndarray):
        if points.shape[0] == 0:
            return points

        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        _, unique_indices = np.unique(
            voxel_indices,
            axis=0,
            return_index=True,
        )

        return points[unique_indices]

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm < 1e-9:
            return np.eye(3, dtype=np.float32)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RSPCDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
