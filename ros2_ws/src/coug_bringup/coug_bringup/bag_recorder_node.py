# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import shutil
import signal
import subprocess
from datetime import datetime, timezone

import diagnostic_updater
import rclpy
import rclpy.logging
from coug_interfaces.srv import BagRecord
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node

PROCESS_WAIT_TIMEOUT_SEC = 10


class BagRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("bag_recorder_node")

        self.declare_parameter("publish_diagnostics", False)
        self.declare_parameter("agent_ns", "auv0")
        self.declare_parameter("bag_record_service", "bag_record")
        self.declare_parameter("log_dir", "")

        publish_diagnostics = self.get_parameter("publish_diagnostics").value
        self._agent_ns = self.get_parameter("agent_ns").value
        self._bag_dir = os.environ.get("BAGS_DIR", "")
        self._log_dir = self.get_parameter("log_dir").value
        bag_record_service = self.get_parameter("bag_record_service").value
        self._bag_path: str | None = None
        self._bag_process: subprocess.Popen | None = None

        if not self._bag_dir:
            self.get_logger().error("BAGS_DIR is not set.")

        self.create_service(BagRecord, bag_record_service, self._bag_record_callback)

        if publish_diagnostics:
            ns = self.get_namespace()
            clean_ns = "" if ns == "/" else ns
            self._diagnostic_updater = diagnostic_updater.Updater(self)
            self._diagnostic_updater.setHardwareID(f"{clean_ns}/bag_recorder_node")
            prefix = f"[{clean_ns}] " if clean_ns else ""
            self._diagnostic_updater.add(
                f"{prefix}Recording Status", self._check_recording_status
            )

        self.get_logger().info("Initialization complete.")

    def _bag_record_callback(
        self, request: BagRecord.Request, response: BagRecord.Response
    ) -> BagRecord.Response:
        if self._bag_process is not None and self._bag_process.poll() is not None:
            self._bag_process = None

        if request.start:
            if self._bag_process is not None:
                response.success = False
                response.message = "Already recording."
                return response

            if not self._bag_dir:
                response.success = False
                response.message = "BAGS_DIR is not set."
                return response

            base = request.prefix if request.prefix else "rosbag"
            timestamp = datetime.now(tz=timezone.utc).strftime("%Y-%m-%d-%H-%M-%S")
            path = os.path.join(self._bag_dir, f"{base}_{self._agent_ns}_{timestamp}")
            self._bag_path = path
            self._bag_process = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "-a",
                    "-o",
                    path,
                    "--storage",
                    "mcap",
                    "--exclude-topics",
                    "/clock",
                ]
            )
            response.success = True
            response.message = f"Recording started: {os.path.basename(path)}"
            self.get_logger().info(f"Bag recording started: {path}")
        else:
            if self._bag_process is None:
                response.success = False
                response.message = "Not recording."
                return response

            self._stop_bag_process()
            self._save_config()
            self._save_logs()
            self._bag_path = None
            response.success = True
            response.message = "Recording stopped."
            self.get_logger().info("Bag recording stopped.")

        return response

    def _check_recording_status(
        self, stat: diagnostic_updater.DiagnosticStatusWrapper
    ) -> diagnostic_updater.DiagnosticStatusWrapper:
        if self._bag_process is not None and self._bag_process.poll() is None:
            stat.summary(DiagnosticStatus.OK, "Recording in progress.")
            stat.add(
                "Bag Path", os.path.basename(self._bag_path) if self._bag_path else ""
            )
        else:
            stat.summary(DiagnosticStatus.OK, "Idle.")
        return stat

    def _stop_bag_process(self) -> None:
        self._bag_process.send_signal(signal.SIGINT)
        try:
            self._bag_process.wait(timeout=PROCESS_WAIT_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            self.get_logger().error("Bag recorder did not stop cleanly. Killing it.")
            self._bag_process.kill()
            self._bag_process.wait()
        self._bag_process = None

    def _save_config(self) -> None:
        if self._bag_path is None or not os.path.isdir(self._bag_path):
            return

        config_dir = os.environ.get("CONFIG_DIR", "")
        if config_dir and os.path.isdir(config_dir):
            dest = os.path.join(self._bag_path, "config")
            shutil.copytree(config_dir, dest, dirs_exist_ok=True)
            self.get_logger().info(f"Config saved: {dest}")

    def _save_logs(self) -> None:
        if self._bag_path is None or not os.path.isdir(self._bag_path):
            return

        log_dir = self._log_dir or rclpy.logging.get_logging_directory()
        if log_dir and os.path.isdir(log_dir):
            dest = os.path.join(self._bag_path, "log")
            shutil.copytree(log_dir, dest, dirs_exist_ok=True)
            self.get_logger().info(f"Logs saved: {dest}")

    def destroy_node(self) -> None:
        if self._bag_process is not None:
            self._stop_bag_process()
            self._save_config()
            self._save_logs()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    bag_recorder_node = BagRecorderNode()
    try:
        rclpy.spin(bag_recorder_node)
    except KeyboardInterrupt:
        pass
    finally:
        bag_recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
