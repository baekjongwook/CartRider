#!/usr/bin/env python3

import os
import math
import threading
from enum import Enum

import numpy as np
import cv2
import rclpy
import tf2_geometry_msgs

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose2D
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException


class PipelineMode(Enum):
    IDLE = "IDLE"
    ZED_SEARCH = "ZED_SEARCH"
    WAIT_NAV_DONE = "WAIT_NAV_DONE"
    D435I_SEARCH = "D435I_SEARCH"
    WAIT_RL_DONE = "WAIT_RL_DONE"
    DONE = "DONE"


class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__("vision_pipeline_node")

        self.bridge = CvBridge()

        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("zed_rgb_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("zed_depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("zed_camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info")

        self.declare_parameter("d435i_rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("d435i_depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("d435i_camera_info_topic", "/camera/color/camera_info")

        self.declare_parameter("vision_on_topic", "/vision_on")
        self.declare_parameter("nav_done_topic", "/nav_goal_reached")
        self.declare_parameter("rl_done_topic", "/rl_done")

        self.declare_parameter("nav_goal_topic", "/vision/nav_goal_pose")
        self.declare_parameter("rl_target_topic", "/vision/rl_target_pose")

        self.declare_parameter("target_camera_topic", "/vision/cart_target_camera")
        self.declare_parameter("target_base_topic", "/vision/cart_target_base")
        self.declare_parameter("target_ground_topic", "/vision/cart_target_ground")
        self.declare_parameter("marker_topic", "/vision/cart_target_marker")
        self.declare_parameter("cart_yaw_topic", "/vision/cart_yaw_deg")

        self.declare_parameter("process_hz", 5.0)
        self.declare_parameter("yolo_imgsz", 416)
        self.declare_parameter("conf_thres", 0.4)
        self.declare_parameter("confirm_window_sec", 1.0)
        self.declare_parameter("confirm_min_count", 4)
        self.declare_parameter("confirm_avg_conf_thres", 0.65)
        self.declare_parameter("confirm_xy_std_max_m", 0.25)

        self.declare_parameter("zed_goal_standoff_m", 1.0)
        self.declare_parameter("max_rgb_depth_dt_sec", 0.20)

        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_name", "Cart Vision Pipeline")

        self.base_frame = self.get_parameter("base_frame").value

        self.zed_rgb_topic = self.get_parameter("zed_rgb_topic").value
        self.zed_depth_topic = self.get_parameter("zed_depth_topic").value
        self.zed_camera_info_topic = self.get_parameter("zed_camera_info_topic").value

        self.d435i_rgb_topic = self.get_parameter("d435i_rgb_topic").value
        self.d435i_depth_topic = self.get_parameter("d435i_depth_topic").value
        self.d435i_camera_info_topic = self.get_parameter("d435i_camera_info_topic").value

        self.vision_on_topic = self.get_parameter("vision_on_topic").value
        self.nav_done_topic = self.get_parameter("nav_done_topic").value
        self.rl_done_topic = self.get_parameter("rl_done_topic").value

        self.nav_goal_topic = self.get_parameter("nav_goal_topic").value
        self.rl_target_topic = self.get_parameter("rl_target_topic").value

        self.target_camera_topic = self.get_parameter("target_camera_topic").value
        self.target_base_topic = self.get_parameter("target_base_topic").value
        self.target_ground_topic = self.get_parameter("target_ground_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.cart_yaw_topic = self.get_parameter("cart_yaw_topic").value

        self.process_hz = float(self.get_parameter("process_hz").value)
        self.process_period_sec = 1.0 / max(self.process_hz, 0.1)
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.conf_thres = float(self.get_parameter("conf_thres").value)

        self.confirm_window_sec = float(self.get_parameter("confirm_window_sec").value)
        self.confirm_min_count = int(self.get_parameter("confirm_min_count").value)
        self.confirm_avg_conf_thres = float(self.get_parameter("confirm_avg_conf_thres").value)
        self.confirm_xy_std_max_m = float(self.get_parameter("confirm_xy_std_max_m").value)

        self.zed_goal_standoff_m = float(self.get_parameter("zed_goal_standoff_m").value)
        self.max_rgb_depth_dt_sec = float(self.get_parameter("max_rgb_depth_dt_sec").value)

        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.window_name = self.get_parameter("window_name").value

        self.mode = PipelineMode.IDLE
        self.nav_goal_sent = False
        self.rl_target_sent = False

        self.latest_lock = threading.Lock()
        self.latest_zed_rgb_msg = None
        self.latest_zed_depth_msg = None
        self.latest_d435i_rgb_msg = None
        self.latest_d435i_depth_msg = None

        self.processing = False
        self.last_tf_warn_time = None
        self.last_state_log_time = None

        self.zed_confirm_buffer = []
        self.d435i_confirm_buffer = []

        self.depth_min_m = 0.2
        self.depth_max_m = 20.0
        self.accept_min_m = 0.2
        self.accept_max_m = 10.0
        self.max_distance_jump_m = 2.5
        self.ema_alpha = 0.4
        self.filtered_distance_m = {
            "zed": None,
            "d435i": None,
        }

        self.roi_ratio_w = 0.25
        self.roi_ratio_h = 0.25
        self.roi_center_y_ratio = 0.45

        self.zed_intrinsics = self.make_empty_intrinsics()
        self.d435i_intrinsics = self.make_empty_intrinsics()

        self.enable_aruco = True
        self.aruco_marker_length_m = 0.20
        self.aruco_id_to_cart_base_yaw_deg = {
            1: 0.0,
            3: 90.0,
            0: 180.0,
            2: 270.0,
        }

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except Exception:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.use_aruco_detector = hasattr(cv2.aruco, "ArucoDetector")
        if self.use_aruco_detector:
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params
            )
        else:
            self.aruco_detector = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        package_share_dir = get_package_share_directory("vision")
        self.model_path = os.path.join(package_share_dir, "models", "cart_1400.pt")

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
        self.model = YOLO(self.model_path)

        try:
            self.model.to("cuda")
            self.get_logger().info("YOLO model moved to CUDA.")
        except Exception:
            self.get_logger().warn("CUDA not available for YOLO. Using default device.")

        self.vision_on_sub = self.create_subscription(
            Bool,
            self.vision_on_topic,
            self.vision_on_callback,
            10,
        )

        self.nav_done_sub = self.create_subscription(
            Bool,
            self.nav_done_topic,
            self.nav_done_callback,
            10,
        )

        self.rl_done_sub = self.create_subscription(
            Bool,
            self.rl_done_topic,
            self.rl_done_callback,
            10,
        )

        self.zed_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.zed_camera_info_topic,
            lambda msg: self.camera_info_callback(msg, "zed"),
            qos_profile_sensor_data,
        )

        self.d435i_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.d435i_camera_info_topic,
            lambda msg: self.camera_info_callback(msg, "d435i"),
            qos_profile_sensor_data,
        )

        self.zed_rgb_sub = self.create_subscription(
            Image,
            self.zed_rgb_topic,
            lambda msg: self.rgb_callback(msg, "zed"),
            qos_profile_sensor_data,
        )

        self.zed_depth_sub = self.create_subscription(
            Image,
            self.zed_depth_topic,
            lambda msg: self.depth_callback(msg, "zed"),
            qos_profile_sensor_data,
        )

        self.d435i_rgb_sub = self.create_subscription(
            Image,
            self.d435i_rgb_topic,
            lambda msg: self.rgb_callback(msg, "d435i"),
            qos_profile_sensor_data,
        )

        self.d435i_depth_sub = self.create_subscription(
            Image,
            self.d435i_depth_topic,
            lambda msg: self.depth_callback(msg, "d435i"),
            qos_profile_sensor_data,
        )

        self.nav_goal_pub = self.create_publisher(PoseStamped, self.nav_goal_topic, 10)
        self.rl_target_pub = self.create_publisher(Pose2D, self.rl_target_topic, 10)

        self.target_camera_pub = self.create_publisher(PointStamped, self.target_camera_topic, 10)
        self.target_base_pub = self.create_publisher(PointStamped, self.target_base_topic, 10)
        self.target_ground_pub = self.create_publisher(PointStamped, self.target_ground_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.cart_yaw_pub = self.create_publisher(Float32, self.cart_yaw_topic, 10)

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

    def make_empty_intrinsics(self):
        return {
            "fx": None,
            "fy": None,
            "cx": None,
            "cy": None,
            "camera_matrix": None,
            "dist_coeffs": None,
            "received": False,
        }

    def log_startup_info(self):
        self.get_logger().info("vision_pipeline_node started.")
        self.get_logger().info(f"Mode              : {self.mode.value}")
        self.get_logger().info(f"Base frame        : {self.base_frame}")
        self.get_logger().info(f"Vision on topic   : {self.vision_on_topic}")
        self.get_logger().info(f"Nav done topic    : {self.nav_done_topic}")
        self.get_logger().info(f"RL done topic     : {self.rl_done_topic}")
        self.get_logger().info(f"Nav goal topic    : {self.nav_goal_topic}")
        self.get_logger().info(f"RL target topic   : {self.rl_target_topic}")
        self.get_logger().info(f"ZED RGB topic     : {self.zed_rgb_topic}")
        self.get_logger().info(f"ZED Depth topic   : {self.zed_depth_topic}")
        self.get_logger().info(f"D435i RGB topic   : {self.d435i_rgb_topic}")
        self.get_logger().info(f"D435i Depth topic : {self.d435i_depth_topic}")
        self.get_logger().info(f"Processing rate   : {self.process_hz:.1f} Hz")
        self.get_logger().info(f"YOLO imgsz        : {self.yolo_imgsz}")
        self.get_logger().info(f"Confirm window    : {self.confirm_window_sec:.2f} sec")
        self.get_logger().info(f"Confirm count     : {self.confirm_min_count}")
        self.get_logger().info(f"Confirm avg conf  : {self.confirm_avg_conf_thres:.2f}")
        self.get_logger().info(f"Confirm xy std    : {self.confirm_xy_std_max_m:.2f} m")
        self.get_logger().info("ZED nav goal      : publish cart center directly")
        self.get_logger().info("Pipeline: vision_on -> ZED_SEARCH -> WAIT_NAV_DONE -> D435I_SEARCH -> WAIT_RL_DONE -> DONE")
        self.get_logger().info("ZED stage publishes cart center to Nav2 once. D435i stage publishes RL target once.")
        self.get_logger().info("Yaw is computed only in D435I_SEARCH.")

    def vision_on_callback(self, msg: Bool):
        if msg.data:
            if self.mode == PipelineMode.IDLE or self.mode == PipelineMode.DONE:
                self.reset_pipeline()
                self.set_mode(PipelineMode.ZED_SEARCH, "vision_on true")
            else:
                self.get_logger().info(
                    f"vision_on true ignored. Pipeline already running: {self.mode.value}"
                )
        else:
            self.reset_pipeline()
            self.set_mode(PipelineMode.IDLE, "vision_on false")

    def nav_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.mode == PipelineMode.WAIT_NAV_DONE:
            self.set_mode(PipelineMode.D435I_SEARCH, "nav goal reached")
        else:
            self.get_logger().info(
                f"nav_done true received but current mode is {self.mode.value}. Ignored."
            )

    def rl_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.mode == PipelineMode.WAIT_RL_DONE:
            self.set_mode(PipelineMode.DONE, "rl done")
        else:
            self.get_logger().info(
                f"rl_done true received but current mode is {self.mode.value}. Ignored."
            )

    def reset_pipeline(self):
        self.nav_goal_sent = False
        self.rl_target_sent = False
        self.zed_confirm_buffer.clear()
        self.d435i_confirm_buffer.clear()
        self.filtered_distance_m["zed"] = None
        self.filtered_distance_m["d435i"] = None
        self.last_tf_warn_time = None
        self.last_state_log_time = None

    def set_mode(self, new_mode: PipelineMode, reason: str = ""):
        if self.mode == new_mode:
            return

        old_mode = self.mode
        self.mode = new_mode
        if reason:
            self.get_logger().info(f"Mode change: {old_mode.value} -> {new_mode.value} ({reason})")
        else:
            self.get_logger().info(f"Mode change: {old_mode.value} -> {new_mode.value}")

    def rgb_callback(self, msg: Image, camera_name: str):
        with self.latest_lock:
            if camera_name == "zed":
                self.latest_zed_rgb_msg = msg
            elif camera_name == "d435i":
                self.latest_d435i_rgb_msg = msg

    def depth_callback(self, msg: Image, camera_name: str):
        with self.latest_lock:
            if camera_name == "zed":
                self.latest_zed_depth_msg = msg
            elif camera_name == "d435i":
                self.latest_d435i_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo, camera_name: str):
        intr = self.zed_intrinsics if camera_name == "zed" else self.d435i_intrinsics

        intr["fx"] = float(msg.k[0])
        intr["fy"] = float(msg.k[4])
        intr["cx"] = float(msg.k[2])
        intr["cy"] = float(msg.k[5])
        intr["camera_matrix"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            intr["dist_coeffs"] = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            intr["dist_coeffs"] = np.zeros((5, 1), dtype=np.float64)

        if not intr["received"]:
            intr["received"] = True
            self.get_logger().info(
                f"{camera_name} intrinsics received: "
                f"fx={intr['fx']:.3f}, fy={intr['fy']:.3f}, "
                f"cx={intr['cx']:.3f}, cy={intr['cy']:.3f}"
            )

    def process_latest_frame(self):
        if self.processing:
            return

        if self.mode in (
            PipelineMode.IDLE,
            PipelineMode.WAIT_NAV_DONE,
            PipelineMode.WAIT_RL_DONE,
            PipelineMode.DONE,
        ):
            return

        self.processing = True

        try:
            if self.mode == PipelineMode.ZED_SEARCH:
                self.process_camera_stage("zed", use_aruco=False)
            elif self.mode == PipelineMode.D435I_SEARCH:
                self.process_camera_stage("d435i", use_aruco=True)
        finally:
            self.processing = False

    def process_camera_stage(self, camera_name: str, use_aruco: bool):
        rgb_msg, depth_msg = self.get_latest_messages(camera_name)

        if rgb_msg is None or depth_msg is None:
            return

        if not self.is_stamp_close(rgb_msg, depth_msg):
            return

        intr = self.zed_intrinsics if camera_name == "zed" else self.d435i_intrinsics
        if not intr["received"]:
            self.warn_state_limited(f"{camera_name} camera info not received yet.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[{camera_name}] Failed to convert RGB image: {e}")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"[{camera_name}] Failed to convert depth image: {e}")
            return

        depth_m = self.depth_to_meters(depth)
        if depth_m is None:
            self.get_logger().error(f"[{camera_name}] Depth image conversion to meters failed.")
            return

        try:
            results = self.model(
                frame,
                imgsz=self.yolo_imgsz,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"[{camera_name}] YOLO inference failed: {e}")
            return

        annotated = frame.copy()
        self.draw_mode_label(annotated, camera_name)

        best_candidate = self.select_best_cart_candidate(results)

        if best_candidate is None:
            self.show_debug_image(annotated)
            return

        x1, y1, x2, y2, conf = best_candidate

        roi_box = self.compute_depth_roi(
            x1, y1, x2, y2, depth_m.shape[1], depth_m.shape[0]
        )

        if roi_box is None:
            self.show_debug_image(annotated)
            return

        rx1, ry1, rx2, ry2 = roi_box
        roi_cx = int((rx1 + rx2) * 0.5)
        roi_cy = int((ry1 + ry2) * 0.5)

        raw_distance_m = self.compute_roi_depth(depth_m, rx1, ry1, rx2, ry2)
        accepted_distance_m = None

        if raw_distance_m is not None:
            if self.is_distance_reasonable(camera_name, raw_distance_m):
                accepted_distance_m = raw_distance_m

        if accepted_distance_m is not None:
            self.filtered_distance_m[camera_name] = self.apply_ema(
                camera_name,
                accepted_distance_m,
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
            conf,
            self.filtered_distance_m[camera_name],
        )

        if self.filtered_distance_m[camera_name] is None:
            self.show_debug_image(annotated)
            return

        x_cam, y_cam, z_cam = self.pixel_to_camera_3d(
            roi_cx,
            roi_cy,
            self.filtered_distance_m[camera_name],
            intr,
        )

        base_msg = self.publish_target_points(
            rgb_msg,
            x_cam,
            y_cam,
            z_cam,
            publish_markers=True,
        )

        if base_msg is None:
            self.show_debug_image(annotated)
            return

        if camera_name == "zed":
            self.handle_zed_candidate(base_msg, conf, annotated)
        elif camera_name == "d435i":
            self.handle_d435i_candidate(
                frame,
                annotated,
                x1,
                y1,
                x2,
                y2,
                base_msg,
                conf,
                intr,
            )

        self.show_debug_image(annotated)

    def get_latest_messages(self, camera_name: str):
        with self.latest_lock:
            if camera_name == "zed":
                return self.latest_zed_rgb_msg, self.latest_zed_depth_msg
            if camera_name == "d435i":
                return self.latest_d435i_rgb_msg, self.latest_d435i_depth_msg

        return None, None

    def select_best_cart_candidate(self, results):
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

            if cls_id != 0:
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

    def handle_zed_candidate(self, base_msg: PointStamped, conf: float, annotated):
        x = float(base_msg.point.x)
        y = float(base_msg.point.y)

        self.add_confirm_sample(
            self.zed_confirm_buffer,
            {
                "time": self.get_clock().now(),
                "x": x,
                "y": y,
                "conf": conf,
            },
        )

        confirmed, mean_x, mean_y, avg_conf = self.check_xy_confirmed(
            self.zed_confirm_buffer
        )

        status = f"ZED confirm {len(self.zed_confirm_buffer)}/{self.confirm_min_count} avg_conf={avg_conf:.2f}"
        cv2.putText(
            annotated,
            status,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if confirmed and not self.nav_goal_sent:
            self.publish_nav_goal_once(mean_x, mean_y)
            self.nav_goal_sent = True
            self.set_mode(PipelineMode.WAIT_NAV_DONE, "ZED target confirmed; nav goal sent once")

    def handle_d435i_candidate(
        self,
        frame,
        annotated,
        x1,
        y1,
        x2,
        y2,
        base_msg: PointStamped,
        conf: float,
        intr,
    ):
        if not self.enable_aruco:
            return

        aruco_result = self.detect_aruco_in_bbox(frame, x1, y1, x2, y2, intr)

        if aruco_result is None:
            cv2.putText(
                annotated,
                "D435i: cart detected, aruco not found",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
            return

        marker_id, marker_corners_full, marker_yaw_deg, cart_yaw_deg = aruco_result
        cart_yaw_deg = self.normalize_angle_360(cart_yaw_deg)

        yaw_msg = Float32()
        yaw_msg.data = float(cart_yaw_deg)
        self.cart_yaw_pub.publish(yaw_msg)

        cv2.polylines(
            annotated,
            [marker_corners_full.astype(np.int32)],
            isClosed=True,
            color=(0, 165, 255),
            thickness=2,
        )

        aruco_label = f"aruco id={marker_id} cart_yaw={cart_yaw_deg:.1f} deg"
        cv2.putText(
            annotated,
            aruco_label,
            (x1, min(y2 + 25, annotated.shape[0] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

        x = float(base_msg.point.x)
        y = float(base_msg.point.y)

        self.add_confirm_sample(
            self.d435i_confirm_buffer,
            {
                "time": self.get_clock().now(),
                "x": x,
                "y": y,
                "yaw_deg": cart_yaw_deg,
                "conf": conf,
            },
        )

        confirmed, mean_x, mean_y, mean_yaw_deg, avg_conf = self.check_pose_confirmed(
            self.d435i_confirm_buffer
        )

        status = f"D435i confirm {len(self.d435i_confirm_buffer)}/{self.confirm_min_count} avg_conf={avg_conf:.2f}"
        cv2.putText(
            annotated,
            status,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if confirmed and not self.rl_target_sent:
            self.publish_rl_target_once(mean_x, mean_y, mean_yaw_deg)
            self.rl_target_sent = True
            self.set_mode(PipelineMode.WAIT_RL_DONE, "D435i pose confirmed; RL target sent once")

    def add_confirm_sample(self, buffer, sample):
        now = self.get_clock().now()
        buffer.append(sample)

        while len(buffer) > 0:
            elapsed = (now - buffer[0]["time"]).nanoseconds * 1e-9
            if elapsed <= self.confirm_window_sec:
                break
            buffer.pop(0)

    def check_xy_confirmed(self, buffer):
        if len(buffer) < self.confirm_min_count:
            avg_conf = float(np.mean([s["conf"] for s in buffer])) if buffer else 0.0
            return False, 0.0, 0.0, avg_conf

        xs = np.array([s["x"] for s in buffer], dtype=np.float32)
        ys = np.array([s["y"] for s in buffer], dtype=np.float32)
        confs = np.array([s["conf"] for s in buffer], dtype=np.float32)

        avg_conf = float(np.mean(confs))
        std_x = float(np.std(xs))
        std_y = float(np.std(ys))

        if avg_conf < self.confirm_avg_conf_thres:
            return False, float(np.mean(xs)), float(np.mean(ys)), avg_conf

        if std_x > self.confirm_xy_std_max_m or std_y > self.confirm_xy_std_max_m:
            return False, float(np.mean(xs)), float(np.mean(ys)), avg_conf

        return True, float(np.mean(xs)), float(np.mean(ys)), avg_conf

    def check_pose_confirmed(self, buffer):
        confirmed_xy, mean_x, mean_y, avg_conf = self.check_xy_confirmed(buffer)

        if not confirmed_xy:
            return False, mean_x, mean_y, 0.0, avg_conf

        yaw_values = [s["yaw_deg"] for s in buffer]
        mean_yaw_deg = self.circular_mean_deg(yaw_values)

        return True, mean_x, mean_y, mean_yaw_deg, avg_conf

    def circular_mean_deg(self, yaw_values):
        if len(yaw_values) == 0:
            return 0.0

        radians = np.deg2rad(np.array(yaw_values, dtype=np.float32))
        mean_sin = float(np.mean(np.sin(radians)))
        mean_cos = float(np.mean(np.cos(radians)))

        angle = math.degrees(math.atan2(mean_sin, mean_cos))
        return self.normalize_angle_360(angle)

    def publish_nav_goal_once(self, cart_x: float, cart_y: float):
        goal_x = float(cart_x)
        goal_y = float(cart_y)

        yaw_to_cart = math.atan2(goal_y, goal_x)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion_z_w(yaw_to_cart)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.nav_goal_pub.publish(msg)

        self.get_logger().info(
            f"Nav goal published once as cart center: frame={self.base_frame}, "
            f"x={goal_x:.3f}, y={goal_y:.3f}, yaw_to_cart={math.degrees(yaw_to_cart):.1f} deg"
        )

    def publish_rl_target_once(self, x: float, y: float, yaw_deg: float):
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(math.radians(self.normalize_angle_360(yaw_deg)))

        self.rl_target_pub.publish(msg)

        self.get_logger().info(
            f"RL target published once: x={msg.x:.3f}, y={msg.y:.3f}, "
            f"theta={msg.theta:.3f} rad ({self.normalize_angle_360(yaw_deg):.1f} deg)"
        )

    def yaw_to_quaternion_z_w(self, yaw_rad: float):
        return math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)

    def publish_target_points(
        self,
        rgb_msg: Image,
        x_cam: float,
        y_cam: float,
        z_cam: float,
        publish_markers: bool = True,
    ):
        camera_msg = PointStamped()
        camera_msg.header = rgb_msg.header
        camera_msg.point.x = float(x_cam)
        camera_msg.point.y = float(y_cam)
        camera_msg.point.z = float(z_cam)

        self.target_camera_pub.publish(camera_msg)

        source_frame = camera_msg.header.frame_id

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.002),
            ):
                self.warn_tf_limited(
                    f"TF not available: {source_frame} -> {self.base_frame}"
                )
                return None

            base_msg = self.tf_buffer.transform(
                camera_msg,
                self.base_frame,
                timeout=Duration(seconds=0.005),
            )

            base_msg.header.stamp = camera_msg.header.stamp
            self.target_base_pub.publish(base_msg)

            ground_msg = PointStamped()
            ground_msg.header = base_msg.header
            ground_msg.point.x = float(base_msg.point.x)
            ground_msg.point.y = float(base_msg.point.y)
            ground_msg.point.z = 0.0

            self.target_ground_pub.publish(ground_msg)

            if publish_markers:
                self.publish_target_marker(base_msg, ground_msg)

            return base_msg

        except TransformException as e:
            self.warn_tf_limited(
                f"Failed to transform target point from "
                f"{camera_msg.header.frame_id} to {self.base_frame}: {e}"
            )
            return None

    def publish_target_marker(self, base_msg: PointStamped, ground_msg: PointStamped):
        sphere = Marker()
        sphere.header = base_msg.header
        sphere.ns = "cart_target"
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

        ground_sphere = Marker()
        ground_sphere.header = ground_msg.header
        ground_sphere.ns = "cart_target"
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

        line = Marker()
        line.header = ground_msg.header
        line.ns = "cart_target"
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

        text = Marker()
        text.header = ground_msg.header
        text.ns = "cart_target"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(ground_msg.point.x)
        text.pose.position.y = float(ground_msg.point.y)
        text.pose.position.z = 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"x={ground_msg.point.x:.2f} m, y={ground_msg.point.y:.2f} m"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 300000000

        self.marker_pub.publish(sphere)
        self.marker_pub.publish(ground_sphere)
        self.marker_pub.publish(line)
        self.marker_pub.publish(text)

    def detect_aruco_in_bbox(self, frame, x1, y1, x2, y2, intr):
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return None

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        if self.use_aruco_detector:
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
            pts = c.reshape(-1, 2)
            area = cv2.contourArea(pts.astype(np.float32))
            if area > best_area:
                best_area = area
                best_idx = i

        if best_idx < 0:
            return None

        marker_id = int(ids[best_idx][0])

        corners_crop = corners[best_idx].reshape(4, 2).astype(np.float32)
        corners_full = corners_crop.copy()
        corners_full[:, 0] += x1
        corners_full[:, 1] += y1

        half = self.aruco_marker_length_m / 2.0
        obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        img_points = corners_full.astype(np.float32)

        ok, rvec, _ = cv2.solvePnP(
            obj_points,
            img_points,
            intr["camera_matrix"],
            intr["dist_coeffs"],
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            ok, rvec, _ = cv2.solvePnP(
                obj_points,
                img_points,
                intr["camera_matrix"],
                intr["dist_coeffs"],
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        if not ok:
            return None

        marker_yaw_deg = self.rvec_to_yaw_deg(rvec)
        cart_yaw_deg = self.compute_cart_yaw_360(marker_id, marker_yaw_deg)

        if cart_yaw_deg is None:
            return None

        return marker_id, corners_full, marker_yaw_deg, cart_yaw_deg

    def rvec_to_yaw_deg(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        z_axis = R[:, 2]
        yaw_rad = math.atan2(z_axis[0], z_axis[2])
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg

    def compute_cart_yaw_360(self, marker_id: int, marker_yaw_deg: float):
        if marker_id not in self.aruco_id_to_cart_base_yaw_deg:
            return None

        base_yaw_deg = self.aruco_id_to_cart_base_yaw_deg[marker_id]
        cart_yaw_deg = self.normalize_angle_360(base_yaw_deg + marker_yaw_deg)
        return cart_yaw_deg

    def normalize_angle_360(self, angle_deg: float):
        angle = float(angle_deg) % 360.0
        if angle < 0.0:
            angle += 360.0
        return angle

    def pixel_to_camera_3d(self, u: int, v: int, z: float, intr):
        x_cam = (float(u) - intr["cx"]) * float(z) / intr["fx"]
        y_cam = (float(v) - intr["cy"]) * float(z) / intr["fy"]
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

    def is_distance_reasonable(self, camera_name: str, distance_m: float):
        if distance_m < self.accept_min_m or distance_m > self.accept_max_m:
            return False

        prev = self.filtered_distance_m[camera_name]

        if prev is not None:
            if abs(distance_m - prev) > self.max_distance_jump_m:
                return False

        return True

    def apply_ema(self, camera_name: str, current_distance_m: float):
        prev = self.filtered_distance_m[camera_name]

        if prev is None:
            return current_distance_m

        return self.ema_alpha * current_distance_m + (1.0 - self.ema_alpha) * prev

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

        if self.last_state_log_time is None:
            self.last_state_log_time = now
            self.get_logger().warn(text)
            return

        elapsed = (now - self.last_state_log_time).nanoseconds * 1e-9

        if elapsed >= 2.0:
            self.last_state_log_time = now
            self.get_logger().warn(text)

    def draw_mode_label(self, image, camera_name: str):
        text = f"mode={self.mode.value} camera={camera_name}"
        cv2.putText(
            image,
            text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

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
        conf,
        distance_m,
    ):
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.rectangle(image, (rx1, ry1), (rx2, ry2), (255, 0, 0), 2)

        label = f"cart {conf:.2f}"
        cv2.putText(
            image,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if distance_m is not None:
            dist_label = f"depth={distance_m:.2f} m"
            cv2.putText(
                image,
                dist_label,
                (x1, max(y1 - 35, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
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

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = VisionPipelineNode()
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