#!/usr/bin/env python3

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

lock = threading.Lock()
state = {"depth": None, "jpeg": None}


class ZedViewer(Node):
    def __init__(self):
        super().__init__("zed_live_view")
        self.last_frame = 0.0
        self.rgb_group = MutuallyExclusiveCallbackGroup()
        self.depth_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",
            self.rgb_callback,
            qos_profile_sensor_data,
            callback_group=self.rgb_group
        )

        self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.depth_callback,
            qos_profile_sensor_data,
            callback_group=self.depth_group
        )

        self.get_logger().info("Waiting for RGB and depth images...")

    def depth_callback(self, msg):
        if msg.encoding != "32FC1":
            return

        dtype = ">f4" if msg.is_bigendian else "<f4"
        data = np.frombuffer(msg.data, dtype=dtype)
        image = data.reshape(msg.height, msg.step // 4)

        cx = msg.width // 2
        cy = msg.height // 2
        patch = image[cy-5:cy+6, cx-5:cx+6]

        valid = patch[
            np.isfinite(patch) &
            (patch >= 0.3) &
            (patch <= 10.0)
        ]

        distance = float(np.median(valid)) if valid.size else None

        with lock:
            state["depth"] = distance

    def rgb_callback(self, msg):
        now = time.monotonic()

        if now - self.last_frame < 0.15:
            return

        self.last_frame = now

        if msg.encoding != "bgra8":
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        raw = raw.reshape(msg.height, msg.step)
        bgra = raw[:, :msg.width * 4].reshape(
            msg.height, msg.width, 4
        )

        frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

        with lock:
            distance = state["depth"]

        cx = msg.width // 2
        cy = msg.height // 2

        cv2.drawMarker(
            frame, (cx, cy), (0, 255, 0),
            cv2.MARKER_CROSS, 25, 2
        )

        cv2.rectangle(frame, (0, 0), (470, 80), (0, 0, 0), -1)

        cv2.putText(
            frame, "SCOUT2 - ZED X VISION",
            (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2
        )

        label = (
            f"CENTER DEPTH: {distance:.2f} m"
            if distance is not None
            else "CENTER DEPTH: NO VALID DATA"
        )

        cv2.putText(
            frame, label,
            (15, 65), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2
        )

        success, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50]
        )

        if success:
            with lock:
                state["jpeg"] = encoded.tobytes()


class ViewerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            html = (
                "<html><body style='background:#111;color:white;"
                "text-align:center;font-family:sans-serif'>"
                "<h2>SCOUT2 - ZED X Vision</h2>"
                "<img src='/stream.mjpg' style='width:95%;"
                "max-width:960px'>"
                "</body></html>"
            ).encode()

            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        try:
            while rclpy.ok():
                with lock:
                    frame = state["jpeg"]

                if frame is not None:
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        + frame + b"\r\n"
                    )

                time.sleep(0.05)
        except (BrokenPipeError, ConnectionError, OSError):
            pass

    def log_message(self, *args):
        pass


def main():
    rclpy.init()
    node = ZedViewer()

    server = ThreadingHTTPServer(
        ("127.0.0.1", 8081), ViewerHandler
    )

    thread = threading.Thread(
        target=server.serve_forever, daemon=True
    )
    thread.start()

    print("SCOUT2 vision viewer running on localhost:8081")

    executor = MultiThreadedExecutor(num_threads=2)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.shutdown()
        server.server_close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
