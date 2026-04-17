#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node

from cartrider_rmd_sdk.msg import MotorStateArray as RmdStateArray
from cartrider_vesc_sdk.msg import MotorStateArray as VescStateArray


RADPS_TO_RPM = 60.0 / (2.0 * math.pi)
RAD_TO_DEG = 180.0 / math.pi


class FrontbotStatePlotter(Node):
    def __init__(self):
        super().__init__("frontbot_state_plotter")

        self.declare_parameter("rmd_topic", "/rmd_state")
        self.declare_parameter("vesc_topic", "/vesc_state")
        self.declare_parameter("rmd_id", 1)
        self.declare_parameter("vesc_id", 3)
        self.declare_parameter("window_sec", 10.0)
        self.declare_parameter("plot_rate_hz", 10.0)

        self.rmd_topic = self.get_parameter("rmd_topic").value
        self.vesc_topic = self.get_parameter("vesc_topic").value
        self.rmd_id = int(self.get_parameter("rmd_id").value)
        self.vesc_id = int(self.get_parameter("vesc_id").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.plot_rate_hz = float(self.get_parameter("plot_rate_hz").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.rmd_t = deque()
        self.rmd_current = deque()
        self.rmd_speed_rpm = deque()

        self.vesc_t = deque()
        self.vesc_position_deg = deque()

        self.create_subscription(
            RmdStateArray,
            self.rmd_topic,
            self.rmd_callback,
            10,
        )

        self.create_subscription(
            VescStateArray,
            self.vesc_topic,
            self.vesc_callback,
            10,
        )

        self.get_logger().info(f"Subscribed to {self.rmd_topic} and {self.vesc_topic}")
        self.get_logger().info(f"Tracking RMD id={self.rmd_id}, VESC id={self.vesc_id}")

        self.fig, self.axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        self.fig.canvas.manager.set_window_title("Frontbot State Plotter")

        self.ax_rmd_current = self.axes[0]
        self.ax_rmd_speed = self.axes[1]
        self.ax_vesc_position = self.axes[2]

        (self.line_rmd_current,) = self.ax_rmd_current.plot(
            [], [], label="RMD current [A]"
        )
        (self.line_rmd_speed,) = self.ax_rmd_speed.plot([], [], label="RMD speed [RPM]")
        (self.line_vesc_position,) = self.ax_vesc_position.plot(
            [], [], label="VESC position [DEG]"
        )

        self.ax_rmd_current.set_ylabel("Current [A]")
        self.ax_rmd_speed.set_ylabel("Speed [RPM]")
        self.ax_vesc_position.set_ylabel("Position [DEG]")
        self.ax_vesc_position.set_xlabel("Time [s]")

        for ax in self.axes:
            ax.grid(True)
            ax.legend(loc="upper right")

        self.ax_rmd_speed.set_ylim(-30.0, 30.0)
        self.ax_vesc_position.set_ylim(-90.0, 90.0)

        interval_ms = int(1000.0 / max(self.plot_rate_hz, 1.0))
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=interval_ms)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def get_states(self, msg):
        if hasattr(msg, "states"):
            return msg.states
        return []

    def find_state_by_id(self, msg, target_id):
        for st in self.get_states(msg):
            if st.id == target_id:
                return st
        return None

    def trim_old(self, t_buf, *data_bufs):
        while t_buf and (t_buf[-1] - t_buf[0] > self.window_sec):
            t_buf.popleft()
            for buf in data_bufs:
                buf.popleft()

    def rmd_callback(self, msg):
        st = self.find_state_by_id(msg, self.rmd_id)
        if st is None:
            return

        t = self.now_sec()
        speed_rpm = float(st.speed) * RADPS_TO_RPM

        with self.lock:
            self.rmd_t.append(t)
            self.rmd_current.append(float(st.current))
            self.rmd_speed_rpm.append(speed_rpm)
            self.trim_old(self.rmd_t, self.rmd_current, self.rmd_speed_rpm)

    def vesc_callback(self, msg):
        st = self.find_state_by_id(msg, self.vesc_id)
        if st is None:
            return

        t = self.now_sec()
        position_deg = float(st.position) * RAD_TO_DEG

        with self.lock:
            self.vesc_t.append(t)
            self.vesc_position_deg.append(position_deg)
            self.trim_old(self.vesc_t, self.vesc_position_deg)

    def autoscale_current_axis(self, ax, xs, ys):
        if len(xs) < 2 or len(ys) < 2:
            return

        valid = [(x, y) for x, y in zip(xs, ys) if not math.isnan(y)]
        if not valid:
            return

        vx = [p[0] for p in valid]
        vy = [p[1] for p in valid]

        xmin, xmax = min(vx), max(vx)
        ymin, ymax = min(vy), max(vy)

        if abs(xmax - xmin) < 1e-9:
            xmax = xmin + 1.0

        if abs(ymax - ymin) < 1e-9:
            ymin -= 1.0
            ymax += 1.0
        else:
            margin = 0.1 * (ymax - ymin)
            ymin -= margin
            ymax += margin

        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)

    def autoscale_time_only(self, ax, xs):
        if len(xs) < 2:
            return
        xmin, xmax = min(xs), max(xs)
        if abs(xmax - xmin) < 1e-9:
            xmax = xmin + 1.0
        ax.set_xlim(xmin, xmax)

    def update_plot(self, _frame):
        with self.lock:
            rmd_t = list(self.rmd_t)
            rmd_current = list(self.rmd_current)
            rmd_speed_rpm = list(self.rmd_speed_rpm)

            vesc_t = list(self.vesc_t)
            vesc_position_deg = list(self.vesc_position_deg)

        self.line_rmd_current.set_data(rmd_t, rmd_current)
        self.line_rmd_speed.set_data(rmd_t, rmd_speed_rpm)
        self.line_vesc_position.set_data(vesc_t, vesc_position_deg)

        self.autoscale_current_axis(self.ax_rmd_current, rmd_t, rmd_current)

        self.autoscale_time_only(self.ax_rmd_speed, rmd_t)
        self.autoscale_time_only(self.ax_vesc_position, vesc_t)

        return (
            self.line_rmd_current,
            self.line_rmd_speed,
            self.line_vesc_position,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontbotStatePlotter()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
