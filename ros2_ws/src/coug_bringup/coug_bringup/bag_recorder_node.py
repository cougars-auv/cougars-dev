# Copyright (c) 2026 BYU FROST Lab
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
    """
    ROS 2 node that starts and stops a rosbag recording on a service call.

    :author: Nelson Durrant
    :date: June 2026
    """

    def __init__(self) -> None:
        super().__init__("bag_recorder_node")

        self.declare_parameter("publish_diagnostics", False)
        self.declare_parameter("auv_ns", "auv")
        self.declare_parameter("bag_record_service", "bag_record")
        self.declare_parameter("log_dir", "")

        publish_diagnostics = (
            self.get_parameter("publish_diagnostics").get_parameter_value().bool_value
        )
        self.auv_ns = self.get_parameter("auv_ns").get_parameter_value().string_value
        self.bag_dir = os.environ.get(
            "BAGS_DIR", os.path.expanduser("~/cougars-dev/bags")
        )
        self.log_dir = self.get_parameter("log_dir").get_parameter_value().string_value
        bag_record_service = (
            self.get_parameter("bag_record_service").get_parameter_value().string_value
        )
        self.bag_path: str | None = None
        self.bag_process: subprocess.Popen | None = None

        self.create_service(BagRecord, bag_record_service, self._bag_record_callback)

        if publish_diagnostics:
            ns = self.get_namespace()
            clean_ns = "" if ns == "/" else ns
            self.updater = diagnostic_updater.Updater(self)
            self.updater.setHardwareID(f"{clean_ns}/bag_recorder_node")
            prefix = f"[{clean_ns}] " if clean_ns else ""
            self.updater.add(f"{prefix}Recording Status", self._check_recording_status)

        self.get_logger().info("Initialization complete.")

    def _bag_record_callback(
        self, request: BagRecord.Request, response: BagRecord.Response
    ) -> BagRecord.Response:
        """
        Handle a bag recording start or stop request.

        :param request: BagRecord request with start flag and bag prefix.
        :param response: BagRecord response with success flag and message.
        :return: The populated BagRecord response.
        """
        if self.bag_process is not None and self.bag_process.poll() is not None:
            self.bag_process = None

        if request.start:
            if self.bag_process is not None:
                response.success = False
                response.message = "Already recording."
                return response

            base = request.prefix if request.prefix else "rosbag"
            timestamp = datetime.now(tz=timezone.utc).strftime("%Y-%m-%d-%H-%M-%S")
            path = os.path.join(self.bag_dir, f"{base}_{self.auv_ns}_{timestamp}")
            self.bag_path = path
            self.bag_process = subprocess.Popen(
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
            if self.bag_process is None:
                response.success = False
                response.message = "Not recording."
                return response

            self._stop_bag_process()
            self._save_config()
            self._save_logs()
            self.bag_path = None
            response.success = True
            response.message = "Recording stopped."
            self.get_logger().info("Bag recording stopped.")

        return response

    def _check_recording_status(
        self, stat: diagnostic_updater.DiagnosticStatusWrapper
    ) -> diagnostic_updater.DiagnosticStatusWrapper:
        """
        Check the status of the bag recording for diagnostics.

        :param stat: Diagnostic status wrapper to update.
        :return: Updated diagnostic status wrapper.
        """
        if self.bag_process is not None and self.bag_process.poll() is None:
            stat.summary(DiagnosticStatus.OK, "Recording in progress.")
            stat.add(
                "Bag Path", os.path.basename(self.bag_path) if self.bag_path else ""
            )
        else:
            stat.summary(DiagnosticStatus.OK, "Idle.")
        return stat

    def _stop_bag_process(self) -> None:
        """Stop the active rosbag recording process."""
        self.bag_process.send_signal(signal.SIGINT)
        try:
            self.bag_process.wait(timeout=PROCESS_WAIT_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            self.get_logger().error("Bag recorder did not stop cleanly. Killing it.")
            self.bag_process.kill()
            self.bag_process.wait()
        self.bag_process = None

    def _save_config(self) -> None:
        """Save the current configuration files to the recorded bag directory."""
        if self.bag_path is None or not os.path.isdir(self.bag_path):
            return

        config_dir = os.environ.get("CONFIG_DIR", "")
        if config_dir and os.path.isdir(config_dir):
            dest = os.path.join(self.bag_path, "config")
            shutil.copytree(config_dir, dest, dirs_exist_ok=True)
            self.get_logger().info(f"Config saved: {dest}")

    def _save_logs(self) -> None:
        """Save the current ROS log files to the recorded bag directory."""
        if self.bag_path is None or not os.path.isdir(self.bag_path):
            return

        log_dir = self.log_dir or rclpy.logging.get_logging_directory()
        if log_dir and os.path.isdir(log_dir):
            dest = os.path.join(self.bag_path, "log")
            shutil.copytree(log_dir, dest, dirs_exist_ok=True)
            self.get_logger().info(f"Logs saved: {dest}")

    def destroy_node(self) -> None:
        """Clean up resources and stop recording when the node is destroyed."""
        if self.bag_process is not None:
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
