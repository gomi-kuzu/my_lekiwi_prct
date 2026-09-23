#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""
ROS2 Bag Recorder Node for LeKiwi (MCAP, per-episode).

This node manages `ros2 bag record` as a subprocess and creates one MCAP bag
per episode. Time synchronization, feature definition, and dataset formatting
are performed later, offline, by `rosbag_to_lerobot.py`.

Design goals:
  - リアルタイム側は取りこぼしゼロを最優先（動画エンコードなどは行わない）。
  - 圧縮画像 (CompressedImage) をそのままbagに書き込むためストレージも軽い。
  - エピソード境界と単一のメタデータ (single_task, fps, robot_type, ...) を
    エピソードディレクトリ内の meta.yaml に保存する。
  - 後段の変換スクリプトが同期基準・許容ズレ・action/observation ラグを
    パラメータとして受け付けられるよう、raw を保持する。
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
import yaml
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


DEFAULT_TOPICS: List[str] = [
    "/lekiwi/joint_states",
    "/lekiwi/cmd_vel",
    "/lekiwi/arm_joint_commands",
    "/lekiwi/camera/front/image_raw/compressed",
    "/lekiwi/camera/wrist/image_raw/compressed",
]


class LeKiwiBagRecorder(Node):
    """Record teleop data as MCAP bags, one per episode."""

    def __init__(self) -> None:
        super().__init__("lekiwi_bag_recorder")

        # Parameters
        self.declare_parameter("session_name", "")
        self.declare_parameter(
            "output_root", str(Path.home() / "lekiwi_bags"))
        self.declare_parameter("single_task", "Pick and place task")
        self.declare_parameter("robot_type", "lekiwi_client")
        self.declare_parameter("target_fps", 30)
        self.declare_parameter("topics", DEFAULT_TOPICS)
        # MCAP storage plugin id ("mcap"). Requires rosbag2_storage_mcap.
        self.declare_parameter("storage_id", "mcap")
        # Optional extra args passed verbatim to `ros2 bag record`.
        self.declare_parameter("extra_record_args", [""])

        self.output_root = Path(
            self.get_parameter("output_root").value).expanduser().resolve()
        self.single_task: str = self.get_parameter("single_task").value
        self.robot_type: str = self.get_parameter("robot_type").value
        self.target_fps: int = int(self.get_parameter("target_fps").value)
        self.topics: List[str] = list(self.get_parameter("topics").value)
        self.storage_id: str = self.get_parameter("storage_id").value
        self.extra_record_args: List[str] = [
            a for a in self.get_parameter("extra_record_args").value if a]

        session_name = self.get_parameter("session_name").value
        if not session_name:
            session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = self.output_root / session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # Write session-level meta
        session_meta = {
            "session_name": session_name,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "robot_type": self.robot_type,
            "target_fps": self.target_fps,
            "single_task": self.single_task,
            "topics": self.topics,
            "storage_id": self.storage_id,
        }
        with (self.session_dir / "session.yaml").open("w") as f:
            yaml.safe_dump(session_meta, f, sort_keys=False,
                           allow_unicode=True)

        # Services
        self.create_service(Trigger, "~/start_episode",
                            self.start_episode_cb)
        self.create_service(Trigger, "~/stop_episode",
                            self.stop_episode_cb)

        # State
        self._proc: Optional[subprocess.Popen] = None
        self._current_episode_dir: Optional[Path] = None
        self._episode_start_time: Optional[float] = None
        self._episode_index: int = self._detect_next_episode_index()

        self.get_logger().info("LeKiwi Bag Recorder initialized")
        self.get_logger().info(f"Session dir: {self.session_dir}")
        self.get_logger().info(f"Task: {self.single_task}")
        self.get_logger().info(f"Topics: {self.topics}")
        self.get_logger().info(
            f"Next episode index: {self._episode_index:06d}")
        self.get_logger().info(
            "Services: ~/start_episode, ~/stop_episode")

    # ---------------- utility ----------------

    def _detect_next_episode_index(self) -> int:
        idx = 0
        for p in self.session_dir.iterdir():
            if p.is_dir() and p.name.startswith("episode_"):
                try:
                    idx = max(idx, int(p.name.split("_")[1]) + 1)
                except (IndexError, ValueError):
                    pass
        return idx

    def _episode_dir(self, index: int) -> Path:
        return self.session_dir / f"episode_{index:06d}"

    # ---------------- services ----------------

    def start_episode_cb(self, request, response):
        if self._proc is not None:
            response.success = False
            response.message = "Already recording an episode"
            return response

        ep_dir = self._episode_dir(self._episode_index)
        if ep_dir.exists():
            response.success = False
            response.message = f"Episode dir already exists: {ep_dir}"
            return response

        bag_path = ep_dir / "bag"
        ep_dir.mkdir(parents=True, exist_ok=False)

        # Write per-episode meta upfront (start time gets filled at stop)
        ep_meta = {
            "episode_index": self._episode_index,
            "single_task": self.single_task,
            "robot_type": self.robot_type,
            "target_fps": self.target_fps,
            "topics": self.topics,
            "start_time": datetime.now().isoformat(timespec="seconds"),
        }
        with (ep_dir / "episode.yaml").open("w") as f:
            yaml.safe_dump(ep_meta, f, sort_keys=False, allow_unicode=True)

        cmd = [
            "ros2", "bag", "record",
            "-s", self.storage_id,
            "-o", str(bag_path),
            *self.extra_record_args,
            *self.topics,
        ]
        self.get_logger().info(f"Starting bag record: {' '.join(cmd)}")

        try:
            # start_new_session so we can send SIGINT to the whole group cleanly.
            self._proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            )
        except FileNotFoundError as e:
            response.success = False
            response.message = f"Failed to launch `ros2 bag record`: {e}"
            self.get_logger().error(response.message)
            return response

        self._current_episode_dir = ep_dir
        self._episode_start_time = time.time()

        response.success = True
        response.message = (
            f"Started episode {self._episode_index} -> {bag_path}")
        self.get_logger().info(response.message)
        return response

    def stop_episode_cb(self, request, response):
        if self._proc is None:
            response.success = False
            response.message = "Not currently recording"
            return response

        assert self._current_episode_dir is not None
        assert self._episode_start_time is not None
        duration = time.time() - self._episode_start_time

        # Send SIGINT to the process group so rosbag2 finalizes the bag.
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            self._proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "rosbag2 did not exit within 15s; sending SIGTERM.")
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            self._proc.wait(timeout=5.0)

        # Update episode meta with stop info.
        meta_path = self._current_episode_dir / "episode.yaml"
        try:
            with meta_path.open() as f:
                meta = yaml.safe_load(f) or {}
            meta["stop_time"] = datetime.now().isoformat(timespec="seconds")
            meta["duration_sec"] = round(duration, 3)
            with meta_path.open("w") as f:
                yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Failed to update episode meta: {e}")

        response.success = True
        response.message = (
            f"Saved episode {self._episode_index} "
            f"({duration:.2f}s) -> {self._current_episode_dir}")
        self.get_logger().info(response.message)

        self._proc = None
        self._current_episode_dir = None
        self._episode_start_time = None
        self._episode_index += 1
        return response

    # ---------------- cleanup ----------------

    def destroy_node(self):
        if self._proc is not None:
            self.get_logger().warn(
                "Node shutting down while recording; stopping bag.")
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=10.0)
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeKiwiBagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
