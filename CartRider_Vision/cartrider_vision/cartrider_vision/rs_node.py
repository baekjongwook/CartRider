#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped, Pose2D
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException


class RSNode(Node):
    def __init__(self):
        super().__init__("rs_node")

        self.bridge = CvBridge()

        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cart_pose_topic", "/rs/cart_pose")
        self.declare_parameter("marker_topic", "/rs/cart_marker")

        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "RS ArUco Cart Pose")
        self.declare_parameter("marker_length_m", 0.20)

        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("depth_margin_px", 4)

        self.declare_parameter("id1_yaw_deg", 0.0)
        self.declare_parameter("id3_yaw_deg", 90.0)
        self.declare_parameter("id0_yaw_deg", 180.0)
        self.declare_parameter("id2_yaw_deg", 270.0)

        self.declare_parameter("id1_offset_x", 0.0)
        self.declare_parameter("id1_offset_y", 0.0)
        self.declare_parameter("id3_offset_x", 0.0)
        self.declare_parameter("id3_offset_y", 0.0)
        self.declare_parameter("id0_offset_x", 0.0)
        self.declare_parameter("id0_offset_y", 0.0)
        self.declare_parameter("id2_offset_x", 0.0)
        self.declare_parameter("id2_offset_y", 0.0)

        self.declare_parameter("yaw_snap_enable", True)
        self.declare_parameter("yaw_snap_zero_min_deg", -10.0)
        self.declare_parameter("yaw_snap_zero_max_deg", 10.0)
        self.declare_parameter("yaw_snap_180_min_deg", 175.0)
        self.declare_parameter("yaw_snap_180_max_deg", 180.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cart_pose_topic = self.get_parameter("cart_pose_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value

        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = self.get_parameter("window_name").value
        self.marker_length_m = float(self.get_parameter("marker_length_m").value)

        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.depth_margin_px = int(self.get_parameter("depth_margin_px").value)

        self.yaw_snap_enable = bool(self.get_parameter("yaw_snap_enable").value)
        self.yaw_snap_zero_min_deg = float(
            self.get_parameter("yaw_snap_zero_min_deg").value
        )
        self.yaw_snap_zero_max_deg = float(
            self.get_parameter("yaw_snap_zero_max_deg").value
        )
        self.yaw_snap_180_min_deg = float(
            self.get_parameter("yaw_snap_180_min_deg").value
        )
        self.yaw_snap_180_max_deg = float(
            self.get_parameter("yaw_snap_180_max_deg").value
        )

        self.aruco_id_to_cart_yaw_base_deg = {
            1: float(self.get_parameter("id1_yaw_deg").value),
            3: float(self.get_parameter("id3_yaw_deg").value),
            0: float(self.get_parameter("id0_yaw_deg").value),
            2: float(self.get_parameter("id2_yaw_deg").value),
        }

        self.marker_to_cart_center_offset_m = {
            1: (
                float(self.get_parameter("id1_offset_x").value),
                float(self.get_parameter("id1_offset_y").value),
            ),
            3: (
                float(self.get_parameter("id3_offset_x").value),
                float(self.get_parameter("id3_offset_y").value),
            ),
            0: (
                float(self.get_parameter("id0_offset_x").value),
                float(self.get_parameter("id0_offset_y").value),
            ),
            2: (
                float(self.get_parameter("id2_offset_x").value),
                float(self.get_parameter("id2_offset_y").value),
            ),
        }

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_ok = False
        self.last_tf_warn_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except Exception:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params,
            )
        else:
            self.aruco_detector = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=3,
            slop=0.05,
        )
        self.sync.registerCallback(self.image_callback)

        self.cart_pose_pub = self.create_publisher(
            Pose2D,
            self.cart_pose_topic,
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            self.marker_topic,
            10,
        )

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("rs_node started.")
        self.get_logger().info(f"RGB topic         : {self.rgb_topic}")
        self.get_logger().info(f"Depth topic       : {self.depth_topic}")
        self.get_logger().info(f"Camera info topic : {self.camera_info_topic}")
        self.get_logger().info(f"Base frame        : {self.base_frame}")
        self.get_logger().info(f"Output topic      : {self.cart_pose_topic}")
        self.get_logger().info(f"Marker topic      : {self.marker_topic}")
        self.get_logger().info("Detection         : largest ArUco only, no YOLO")
        self.get_logger().info("Position          : depth first, PnP fallback")
        self.get_logger().info("Output            : Pose2D x[m], y[m], theta[rad]")
        self.get_logger().info(f"Yaw snap enable   : {self.yaw_snap_enable}")
        self.get_logger().info(
            f"Yaw snap 0 deg    : "
            f"{self.yaw_snap_zero_min_deg:.1f} ~ "
            f"{self.yaw_snap_zero_max_deg:.1f} deg"
        )
        self.get_logger().info(
            f"Yaw snap 180 deg  : abs(yaw) "
            f"{self.yaw_snap_180_min_deg:.1f} ~ "
            f"{self.yaw_snap_180_max_deg:.1f} deg"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        if not self.camera_info_ok:
            self.camera_info_ok = True
            self.get_logger().info(
                "Camera intrinsics received: "
                f"fx={self.camera_matrix[0, 0]:.3f}, "
                f"fy={self.camera_matrix[1, 1]:.3f}, "
                f"cx={self.camera_matrix[0, 2]:.3f}, "
                f"cy={self.camera_matrix[1, 2]:.3f}"
            )

    def image_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.camera_info_ok:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        depth_m = self.depth_to_meters(depth)
        annotated = frame.copy()

        marker = self.detect_largest_aruco(frame)

        if marker is None:
            self.draw_status(annotated, "ArUco: not detected", bad=True)
            self.show(annotated)
            return

        marker_id, corners = marker

        marker_yaw_deg, pnp_xyz = self.estimate_marker_pose_pnp(corners)

        if marker_yaw_deg is None or pnp_xyz is None:
            self.draw_marker(annotated, corners, marker_id)
            self.draw_status(annotated, "PnP failed", bad=True)
            self.show(annotated)
            return

        raw_cart_yaw_deg = self.compute_cart_yaw_180(marker_id, marker_yaw_deg)

        if raw_cart_yaw_deg is None:
            self.draw_marker(annotated, corners, marker_id)
            self.draw_status(annotated, f"Unknown marker id: {marker_id}", bad=True)
            self.show(annotated)
            return

        cart_yaw_deg = self.apply_yaw_snap(raw_cart_yaw_deg)

        depth_xyz = self.compute_depth_xyz(depth_m, corners)
        used_source = "PNP"

        if depth_xyz is not None:
            final_xyz = depth_xyz
            used_source = "DEPTH"
        else:
            final_xyz = pnp_xyz
            used_source = "PNP"

        x_cam, y_cam, z_cam = final_xyz

        marker_base = self.transform_camera_point_to_base(
            rgb_msg,
            x_cam,
            y_cam,
            z_cam,
        )

        if marker_base is None:
            self.draw_marker(annotated, corners, marker_id)
            self.draw_status(annotated, "TF failed", bad=True)
            self.show(annotated)
            return

        marker_x = float(marker_base.point.x)
        marker_y = float(marker_base.point.y)

        cart_x, cart_y = self.marker_to_cart_center(
            marker_id,
            marker_x,
            marker_y,
            cart_yaw_deg,
        )

        cart_pose = Pose2D()
        cart_pose.x = float(cart_x)
        cart_pose.y = float(cart_y)
        cart_pose.theta = math.radians(float(cart_yaw_deg))

        self.cart_pose_pub.publish(cart_pose)
        self.publish_cart_marker(cart_pose)

        self.draw_marker(annotated, corners, marker_id)
        self.draw_pose_text(
            annotated,
            marker_id,
            cart_pose,
            raw_cart_yaw_deg,
            cart_yaw_deg,
            pnp_xyz,
            depth_xyz,
            used_source,
        )
        self.show(annotated)

    def detect_largest_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )

        if ids is None or len(corners) == 0:
            return None

        best_idx = -1
        best_area = -1.0

        for i, c in enumerate(corners):
            pts = c.reshape(-1, 2).astype(np.float32)
            area = cv2.contourArea(pts)

            if area > best_area:
                best_area = area
                best_idx = i

        if best_idx < 0:
            return None

        marker_id = int(ids[best_idx][0])
        corners_full = corners[best_idx].reshape(4, 2).astype(np.float32)

        return marker_id, corners_full

    def estimate_marker_pose_pnp(self, corners):
        half = self.marker_length_m / 2.0

        obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            corners.astype(np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners.astype(np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        if not ok:
            return None, None

        rotation, _ = cv2.Rodrigues(rvec)
        z_axis = rotation[:, 2]
        yaw_rad = math.atan2(float(z_axis[0]), float(z_axis[2]))
        marker_yaw_deg = math.degrees(yaw_rad)

        t = tvec.reshape(-1)
        x_cam = float(t[0])
        y_cam = float(t[1])
        z_cam = float(t[2])

        return marker_yaw_deg, (x_cam, y_cam, z_cam)

    def compute_depth_xyz(self, depth_m, corners):
        depth_z = self.compute_marker_depth(depth_m, corners)

        if depth_z is None:
            return None

        center_u, center_v = self.marker_center_pixel(corners)
        return self.pixel_to_camera(center_u, center_v, depth_z)

    def compute_marker_depth(self, depth_m, corners):
        mask = np.zeros(depth_m.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, corners.astype(np.int32), 255)

        if self.depth_margin_px > 0:
            k = 2 * self.depth_margin_px + 1
            kernel = np.ones((k, k), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)

        values = depth_m[mask > 0]

        if values.size == 0:
            return None

        valid = values[np.isfinite(values)]
        valid = valid[(valid > self.depth_min_m) & (valid < self.depth_max_m)]

        if valid.size < 10:
            return None

        median = float(np.median(valid))
        refined = valid[(valid > median - 0.3) & (valid < median + 0.3)]

        if refined.size >= 10:
            median = float(np.median(refined))

        return median

    def marker_center_pixel(self, corners):
        center = np.mean(corners, axis=0)
        return int(center[0]), int(center[1])

    def pixel_to_camera(self, u, v, z):
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])

        x = (float(u) - cx) * float(z) / fx
        y = (float(v) - cy) * float(z) / fy

        return x, y, float(z)

    def compute_cart_yaw_180(self, marker_id, marker_yaw_deg):
        if marker_id not in self.aruco_id_to_cart_yaw_base_deg:
            return None

        base_yaw = self.aruco_id_to_cart_yaw_base_deg[marker_id]
        return self.normalize_angle_180(base_yaw + marker_yaw_deg)

    def normalize_angle_180(self, angle_deg):
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    def apply_yaw_snap(self, yaw_deg):
        yaw = self.normalize_angle_180(yaw_deg)

        if not self.yaw_snap_enable:
            return yaw

        zero_min = min(self.yaw_snap_zero_min_deg, self.yaw_snap_zero_max_deg)
        zero_max = max(self.yaw_snap_zero_min_deg, self.yaw_snap_zero_max_deg)

        yaw_180_min = min(self.yaw_snap_180_min_deg, self.yaw_snap_180_max_deg)
        yaw_180_max = max(self.yaw_snap_180_min_deg, self.yaw_snap_180_max_deg)

        abs_yaw = abs(yaw)

        if zero_min <= yaw <= zero_max:
            return 0.0

        if yaw_180_min <= abs_yaw <= yaw_180_max:
            return 180.0

        return yaw

    def transform_camera_point_to_base(self, rgb_msg, x, y, z):
        camera_msg = PointStamped()
        camera_msg.header = rgb_msg.header
        camera_msg.point.x = float(x)
        camera_msg.point.y = float(y)
        camera_msg.point.z = float(z)

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                camera_msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.005),
            ):
                self.warn_tf_limited(
                    f"TF not available: {camera_msg.header.frame_id} -> {self.base_frame}"
                )
                return None

            return self.tf_buffer.transform(
                camera_msg,
                self.base_frame,
                timeout=Duration(seconds=0.01),
            )

        except TransformException as e:
            self.warn_tf_limited(
                f"TF failed: {camera_msg.header.frame_id} -> {self.base_frame}: {e}"
            )
            return None

    def marker_to_cart_center(self, marker_id, marker_x, marker_y, cart_yaw_deg):
        dx, dy = self.marker_to_cart_center_offset_m.get(marker_id, (0.0, 0.0))

        yaw = math.radians(cart_yaw_deg)

        cart_x = marker_x + math.cos(yaw) * dx - math.sin(yaw) * dy
        cart_y = marker_y + math.sin(yaw) * dx + math.cos(yaw) * dy

        return cart_x, cart_y

    def depth_to_meters(self, depth):
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) / 1000.0

        return depth.astype(np.float32)

    def publish_cart_marker(self, pose: Pose2D):
        now_msg = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.stamp = now_msg
        sphere.header.frame_id = self.base_frame
        sphere.ns = "rs_cart"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(pose.x)
        sphere.pose.position.y = float(pose.y)
        sphere.pose.position.z = 0.08
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.18
        sphere.scale.y = 0.18
        sphere.scale.z = 0.18
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        sphere.lifetime.sec = 0
        sphere.lifetime.nanosec = 300000000

        arrow = Marker()
        arrow.header.stamp = now_msg
        arrow.header.frame_id = self.base_frame
        arrow.ns = "rs_cart"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.05
        arrow.scale.y = 0.10
        arrow.scale.z = 0.10
        arrow.color.r = 1.0
        arrow.color.g = 0.5
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        arrow.lifetime.sec = 0
        arrow.lifetime.nanosec = 300000000

        arrow_len = 0.45

        p0 = Point()
        p0.x = float(pose.x)
        p0.y = float(pose.y)
        p0.z = 0.12

        p1 = Point()
        p1.x = float(pose.x) + arrow_len * math.cos(float(pose.theta))
        p1.y = float(pose.y) + arrow_len * math.sin(float(pose.theta))
        p1.z = 0.12

        arrow.points.append(p0)
        arrow.points.append(p1)

        text = Marker()
        text.header.stamp = now_msg
        text.header.frame_id = self.base_frame
        text.ns = "rs_cart"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(pose.x)
        text.pose.position.y = float(pose.y)
        text.pose.position.z = 0.45
        text.pose.orientation.w = 1.0
        text.scale.z = 0.22
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"x={pose.x:.2f} m, y={pose.y:.2f} m, "
            f"yaw={math.degrees(pose.theta):.1f} deg"
        )
        text.lifetime.sec = 0
        text.lifetime.nanosec = 300000000

        line = Marker()
        line.header.stamp = now_msg
        line.header.frame_id = self.base_frame
        line.ns = "rs_cart"
        line.id = 3
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.025
        line.color.r = 0.0
        line.color.g = 0.7
        line.color.b = 1.0
        line.color.a = 1.0
        line.lifetime.sec = 0
        line.lifetime.nanosec = 300000000

        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.03

        target = Point()
        target.x = float(pose.x)
        target.y = float(pose.y)
        target.z = 0.03

        line.points.append(origin)
        line.points.append(target)

        self.marker_pub.publish(sphere)
        self.marker_pub.publish(arrow)
        self.marker_pub.publish(text)
        self.marker_pub.publish(line)

    def draw_marker(self, image, corners, marker_id):
        cv2.polylines(
            image,
            [corners.astype(np.int32)],
            True,
            (0, 165, 255),
            2,
        )

        center = np.mean(corners, axis=0)
        u = int(center[0])
        v = int(center[1])

        cv2.circle(image, (u, v), 4, (0, 0, 255), -1)

        cv2.putText(
            image,
            f"id={marker_id}",
            (u + 8, v - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

    def draw_status(self, image, text, bad=False):
        color = (0, 0, 255) if bad else (255, 255, 255)

        cv2.putText(
            image,
            text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )

    def draw_pose_text(
        self,
        image,
        marker_id,
        pose,
        raw_yaw_deg,
        snapped_yaw_deg,
        pnp_xyz,
        depth_xyz,
        used_source,
    ):
        pnp_z = pnp_xyz[2] if pnp_xyz is not None else None
        depth_z = depth_xyz[2] if depth_xyz is not None else None

        depth_text = f"{depth_z:.3f} m" if depth_z is not None else "None"
        pnp_text = f"{pnp_z:.3f} m" if pnp_z is not None else "None"

        lines = [
            f"ArUco ID: {marker_id}",
            f"x={pose.x:.3f} m, y={pose.y:.3f} m",
            f"raw yaw={raw_yaw_deg:.1f} deg",
            f"yaw={snapped_yaw_deg:.1f} deg",
            f"theta={pose.theta:.3f} rad",
            f"used={used_source}",
            f"depth z={depth_text}",
            f"PnP z={pnp_text}",
        ]

        y = 30
        for line in lines:
            cv2.putText(
                image,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
            y += 28

    def show(self, image):
        if not self.show_window:
            return

        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rclpy.shutdown()

    def warn_tf_limited(self, text):
        now = self.get_clock().now()

        if self.last_tf_warn_time is None:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)
            return

        elapsed = (now - self.last_tf_warn_time).nanoseconds * 1e-9

        if elapsed >= 1.0:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RSNode()
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
