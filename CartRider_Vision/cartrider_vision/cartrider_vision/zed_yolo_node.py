#!/usr/bin/env python3

import os
import math
import threading

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # Registers PointStamped transforms with tf2.

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException


class ZedYoloNode(Node):
    def __init__(self):
        super().__init__("zed_yolo_node")

        self.bridge = CvBridge()

        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("rgb_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter(
            "camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info"
        )

        self.declare_parameter("cart_target_topic", "/zed_yolo/global_cart_target")
        self.declare_parameter("marker_topic", "/zed_yolo/marker")

        self.declare_parameter("model_package", "cartrider_vision")
        self.declare_parameter("model_file", "cart_1400.pt")

        self.declare_parameter("process_hz", 5.0)
        self.declare_parameter("yolo_imgsz", 416)
        self.declare_parameter("target_class_id", 0)
        self.declare_parameter("conf_thres", 0.4)

        self.declare_parameter("max_rgb_depth_dt_sec", 0.20)

        self.declare_parameter("depth_min_m", 0.2)
        self.declare_parameter("depth_max_m", 20.0)

        self.declare_parameter("roi_ratio_w", 0.25)
        self.declare_parameter("roi_ratio_h", 0.25)
        self.declare_parameter("roi_center_y_ratio", 0.45)

        self.declare_parameter("cart_target_publish_period_sec", 1.0)

        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_name", "ZED YOLO Detect")

        self.base_frame = self.get_parameter("base_frame").value

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.cart_target_topic = self.get_parameter("cart_target_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value

        self.model_package = self.get_parameter("model_package").value
        self.model_file = self.get_parameter("model_file").value

        self.process_hz = float(self.get_parameter("process_hz").value)
        self.process_period_sec = 1.0 / max(self.process_hz, 0.1)

        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.conf_thres = float(self.get_parameter("conf_thres").value)

        self.max_rgb_depth_dt_sec = float(
            self.get_parameter("max_rgb_depth_dt_sec").value
        )

        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)

        self.roi_ratio_w = float(self.get_parameter("roi_ratio_w").value)
        self.roi_ratio_h = float(self.get_parameter("roi_ratio_h").value)
        self.roi_center_y_ratio = float(self.get_parameter("roi_center_y_ratio").value)

        self.cart_target_publish_period_sec = float(
            self.get_parameter("cart_target_publish_period_sec").value
        )

        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.window_name = self.get_parameter("window_name").value

        self.latest_lock = threading.Lock()
        self.latest_rgb_msg = None
        self.latest_depth_msg = None

        self.intrinsics_received = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.processing = False
        self.last_cart_target_publish_time = None
        self.last_tf_warn_time = None
        self.last_state_warn_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        package_share_dir = get_package_share_directory(self.model_package)
        self.model_path = os.path.join(package_share_dir, "models", self.model_file)

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
        self.model = YOLO(self.model_path)

        try:
            self.model.to("cuda")
            self.get_logger().info("YOLO model moved to CUDA.")
        except Exception:
            self.get_logger().warn("CUDA not available for YOLO. Using default device.")

        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            qos_profile_sensor_data,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.cart_target_pub = self.create_publisher(
            PointStamped, self.cart_target_topic, 10
        )
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.process_timer = self.create_timer(
            self.process_period_sec,
            self.process_latest_frame,
        )

        self.gui_timer = self.create_timer(
            1.0 / 30.0,
            self.gui_spin_once,
        )

        if self.show_debug_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.log_startup_info()

    def log_startup_info(self):
        self.get_logger().info("zed_yolo_node started.")
        self.get_logger().info(f"Base frame          : {self.base_frame}")
        self.get_logger().info(f"RGB topic           : {self.rgb_topic}")
        self.get_logger().info(f"Depth topic         : {self.depth_topic}")
        self.get_logger().info(f"Camera info topic   : {self.camera_info_topic}")
        self.get_logger().info(f"Cart target topic   : {self.cart_target_topic}")
        self.get_logger().info(f"Marker topic        : {self.marker_topic}")
        self.get_logger().info(f"Model path          : {self.model_path}")
        self.get_logger().info(f"Processing rate     : {self.process_hz:.1f} Hz")
        self.get_logger().info(f"YOLO imgsz          : {self.yolo_imgsz}")
        self.get_logger().info(f"Target class id     : {self.target_class_id}")
        self.get_logger().info(f"Confidence threshold: {self.conf_thres:.2f}")
        self.get_logger().info(
            "Output: cart target PointStamped every 1 sec and visualization marker"
        )

    def rgb_callback(self, msg: Image):
        with self.latest_lock:
            self.latest_rgb_msg = msg

    def depth_callback(self, msg: Image):
        with self.latest_lock:
            self.latest_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        if not self.intrinsics_received:
            self.intrinsics_received = True
            self.get_logger().info(
                f"Camera intrinsics received: "
                f"fx={self.fx:.3f}, fy={self.fy:.3f}, "
                f"cx={self.cx:.3f}, cy={self.cy:.3f}"
            )

    def process_latest_frame(self):
        if self.processing:
            return

        self.processing = True

        try:
            self._process_latest_frame_impl()
        finally:
            self.processing = False

    def _process_latest_frame_impl(self):
        rgb_msg, depth_msg = self.get_latest_messages()

        if rgb_msg is None or depth_msg is None:
            self.warn_state_limited("Waiting for RGB/depth image.")
            return

        if not self.intrinsics_received:
            self.warn_state_limited("Waiting for camera_info.")
            return

        if not self.is_stamp_close(rgb_msg, depth_msg):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        depth_m = self.depth_to_meters(depth)
        if depth_m is None:
            self.get_logger().error("Depth image conversion to meters failed.")
            return

        annotated = frame.copy()

        try:
            results = self.model(
                frame,
                imgsz=self.yolo_imgsz,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        best_candidate = self.select_best_candidate(results)

        if best_candidate is None:
            self.handle_no_detection()
            self.show_debug_image(annotated)
            return

        x1, y1, x2, y2, _conf = best_candidate

        roi_box = self.compute_depth_roi(
            x1,
            y1,
            x2,
            y2,
            depth_m.shape[1],
            depth_m.shape[0],
        )

        if roi_box is None:
            self.handle_no_detection()
            self.show_debug_image(annotated)
            return

        rx1, ry1, rx2, ry2 = roi_box
        roi_cx = int((rx1 + rx2) * 0.5)
        roi_cy = int((ry1 + ry2) * 0.5)

        raw_distance_m = self.compute_roi_depth(depth_m, rx1, ry1, rx2, ry2)

        if raw_distance_m is None:
            self.handle_no_detection()
            self.draw_detection(annotated, x1, y1, x2, y2, rx1, ry1, rx2, ry2, None)
            self.show_debug_image(annotated)
            return

        x_cam, y_cam, z_cam = self.pixel_to_camera_3d(
            roi_cx,
            roi_cy,
            raw_distance_m,
        )

        base_msg = self.transform_target_point(
            rgb_msg,
            x_cam,
            y_cam,
            z_cam,
        )

        self.draw_detection(
            annotated,
            x1,
            y1,
            x2,
            y2,
            rx1,
            ry1,
            rx2,
            ry2,
            raw_distance_m,
        )

        if base_msg is None:
            self.show_debug_image(annotated)
            return

        if self.should_publish_cart_target():
            self.publish_cart_target_ground(base_msg.point.x, base_msg.point.y)

        self.show_debug_image(annotated)

    def get_latest_messages(self):
        with self.latest_lock:
            return self.latest_rgb_msg, self.latest_depth_msg

    def select_best_candidate(self, results):
        best_candidate = None
        best_score = -1.0

        if len(results) <= 0:
            return None

        result = results[0]

        if result.boxes is None:
            return None

        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            if cls_id != self.target_class_id:
                continue

            if conf < self.conf_thres:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            w = max(0, x2 - x1)
            h = max(0, y2 - y1)
            area = w * h
            score = area * conf

            if score > best_score:
                best_score = score
                best_candidate = (x1, y1, x2, y2, conf)

        return best_candidate

    def handle_no_detection(self):
        self.last_cart_target_publish_time = None

    def transform_target_point(
        self, rgb_msg: Image, x_cam: float, y_cam: float, z_cam: float
    ):
        camera_msg = PointStamped()
        camera_msg.header = rgb_msg.header
        camera_msg.point.x = float(x_cam)
        camera_msg.point.y = float(y_cam)
        camera_msg.point.z = float(z_cam)

        source_frame = camera_msg.header.frame_id

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            ):
                self.warn_tf_limited(
                    f"TF not available: {source_frame} -> {self.base_frame}"
                )
                return None

            base_msg = self.tf_buffer.transform(
                camera_msg,
                self.base_frame,
                timeout=Duration(seconds=0.02),
            )

            base_msg.header.stamp = camera_msg.header.stamp

            ground_msg = PointStamped()
            ground_msg.header = base_msg.header
            ground_msg.point.x = float(base_msg.point.x)
            ground_msg.point.y = float(base_msg.point.y)
            ground_msg.point.z = 0.0

            self.publish_target_marker(base_msg, ground_msg)

            return base_msg

        except TransformException as e:
            self.warn_tf_limited(
                f"Failed to transform target point from {source_frame} to {self.base_frame}: {e}"
            )
            return None

    def should_publish_cart_target(self):
        if self.cart_target_publish_period_sec <= 0.0:
            return True

        now = self.get_clock().now()

        if self.last_cart_target_publish_time is None:
            return True

        elapsed_sec = (
            now - self.last_cart_target_publish_time
        ).nanoseconds * 1e-9

        return elapsed_sec >= self.cart_target_publish_period_sec

    def publish_cart_target_ground(self, cart_x: float, cart_y: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.point.x = float(cart_x)
        msg.point.y = float(cart_y)
        msg.point.z = 0.0

        self.cart_target_pub.publish(msg)
        self.last_cart_target_publish_time = self.get_clock().now()

        self.get_logger().info(
            f"Cart target published: frame={self.base_frame}, "
            f"x={msg.point.x:.3f}, y={msg.point.y:.3f}"
        )

    def publish_target_marker(self, base_msg: PointStamped, ground_msg: PointStamped):
        sphere = Marker()
        sphere.header = base_msg.header
        sphere.ns = "zed_yolo_target"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(base_msg.point.x)
        sphere.pose.position.y = float(base_msg.point.y)
        sphere.pose.position.z = float(base_msg.point.z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.15
        sphere.scale.y = 0.15
        sphere.scale.z = 0.15
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        sphere.lifetime.sec = 0
        sphere.lifetime.nanosec = 300000000
        self.marker_pub.publish(sphere)

        ground_sphere = Marker()
        ground_sphere.header = ground_msg.header
        ground_sphere.ns = "zed_yolo_target"
        ground_sphere.id = 1
        ground_sphere.type = Marker.SPHERE
        ground_sphere.action = Marker.ADD
        ground_sphere.pose.position.x = float(ground_msg.point.x)
        ground_sphere.pose.position.y = float(ground_msg.point.y)
        ground_sphere.pose.position.z = 0.05
        ground_sphere.pose.orientation.w = 1.0
        ground_sphere.scale.x = 0.18
        ground_sphere.scale.y = 0.18
        ground_sphere.scale.z = 0.05
        ground_sphere.color.r = 1.0
        ground_sphere.color.g = 0.5
        ground_sphere.color.b = 0.0
        ground_sphere.color.a = 1.0
        ground_sphere.lifetime.sec = 0
        ground_sphere.lifetime.nanosec = 300000000
        self.marker_pub.publish(ground_sphere)

        line = Marker()
        line.header = ground_msg.header
        line.ns = "zed_yolo_target"
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r = 0.0
        line.color.g = 0.7
        line.color.b = 1.0
        line.color.a = 1.0
        line.lifetime.sec = 0
        line.lifetime.nanosec = 300000000

        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.03

        p1 = Point()
        p1.x = float(ground_msg.point.x)
        p1.y = float(ground_msg.point.y)
        p1.z = 0.03

        line.points.append(p0)
        line.points.append(p1)
        self.marker_pub.publish(line)

    def pixel_to_camera_3d(self, u: int, v: int, z: float):
        x_cam = (float(u) - self.cx) * float(z) / self.fx
        y_cam = (float(v) - self.cy) * float(z) / self.fy
        z_cam = float(z)
        return x_cam, y_cam, z_cam

    def depth_to_meters(self, depth):
        if depth is None:
            return None

        if depth.dtype == np.float32 or depth.dtype == np.float64:
            return depth.astype(np.float32)

        if depth.dtype == np.uint16:
            return depth.astype(np.float32) / 1000.0

        try:
            return depth.astype(np.float32)
        except Exception:
            return None

    def compute_depth_roi(self, x1, y1, x2, y2, img_w, img_h):
        bw = x2 - x1
        bh = y2 - y1

        if bw <= 0 or bh <= 0:
            return None

        cx = int((x1 + x2) * 0.5)
        cy = int(y1 + bh * self.roi_center_y_ratio)

        roi_w = max(6, int(bw * self.roi_ratio_w))
        roi_h = max(6, int(bh * self.roi_ratio_h))

        rx1 = max(0, cx - roi_w // 2)
        ry1 = max(0, cy - roi_h // 2)
        rx2 = min(img_w - 1, cx + roi_w // 2)
        ry2 = min(img_h - 1, cy + roi_h // 2)

        if rx2 <= rx1 or ry2 <= ry1:
            return None

        return rx1, ry1, rx2, ry2

    def compute_roi_depth(self, depth_m, x1, y1, x2, y2):
        roi = depth_m[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        values = roi.reshape(-1)
        valid = values[np.isfinite(values)]
        valid = valid[(valid > self.depth_min_m) & (valid < self.depth_max_m)]

        if valid.size < 10:
            return None

        median = float(np.median(valid))

        lower = median - 0.5
        upper = median + 0.5
        refined = valid[(valid >= lower) & (valid <= upper)]

        if refined.size >= 10:
            median = float(np.median(refined))

        if math.isnan(median) or math.isinf(median):
            return None

        return median

    def is_stamp_close(self, rgb_msg: Image, depth_msg: Image):
        rgb_t = self.stamp_to_sec(rgb_msg.header.stamp)
        depth_t = self.stamp_to_sec(depth_msg.header.stamp)

        if rgb_t <= 0.0 or depth_t <= 0.0:
            return True

        dt = abs(rgb_t - depth_t)

        if dt > self.max_rgb_depth_dt_sec:
            self.warn_state_limited(f"RGB/depth timestamp gap too large: {dt:.3f} sec")
            return False

        return True

    def stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def draw_detection(
        self,
        image,
        x1,
        y1,
        x2,
        y2,
        rx1,
        ry1,
        rx2,
        ry2,
        distance_m,
    ):
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if distance_m is not None:
            label = f"{distance_m:.2f}m"
            text_x = max(2, x1)
            text_y = max(16, y1 - 5)

            cv2.putText(
                image,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    def show_debug_image(self, image):
        if self.show_debug_window:
            cv2.imshow(self.window_name, image)

    def gui_spin_once(self):
        if not self.show_debug_window:
            return

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.get_logger().info("Pressed q. Shutting down...")
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def warn_tf_limited(self, text: str):
        now = self.get_clock().now()

        if self.last_tf_warn_time is None:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)
            return

        elapsed = (now - self.last_tf_warn_time).nanoseconds * 1e-9

        if elapsed >= 1.0:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)

    def warn_state_limited(self, text: str):
        now = self.get_clock().now()

        if self.last_state_warn_time is None:
            self.last_state_warn_time = now
            self.get_logger().warn(text)
            return

        elapsed = (now - self.last_state_warn_time).nanoseconds * 1e-9

        if elapsed >= 2.0:
            self.last_state_warn_time = now
            self.get_logger().warn(text)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ZedYoloNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
