#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""
Offline converter: LeKiwi MCAP rosbag(s) -> LeRobot Dataset v3.

Two modes:
  - analyze : Show how many frames would be dropped for a range of
              synchronization tolerances, without writing a dataset.
              Use this to choose a good --sync-tolerance-ms.
  - convert : Actually build the LeRobot v3 dataset.

Time synchronization
--------------------
- 各エピソードの中で、指定した *基準トピック* (`--sync-base-topic`) の
  タイムスタンプごとに、他トピックの最近傍メッセージを取り、
  「基準stamp からの最大絶対ズレ」が `--sync-tolerance-ms` を超えた
  フレームは棄却する。
- `--sync-base-topic` を指定しなかった場合は、bag 内で **平均周波数が
  最も低い** トピックを自動選択する（普通はカメラのどちらか）。
- タイムスタンプは `header.stamp` があればそれを使い、無ければ
  bag の受信時刻を使う。CompressedImage も JointState も header を持つ。

Action / Observation lag
------------------------
- `--action-lag-ms` は action 側のタイムスタンプに加算するオフセット。
  すなわち base_ts (obs) に対して `action_ts = base_ts + lag` に最も近い
  action メッセージを選ぶ。
- 推奨デフォルトは 0 ms。SO100 系のテレオペで観察 → 指令のフィードバック
  遅延を打ち消したい場合、1 制御周期 (30Hz なら +33ms) 程度で試すと
  よい。まずは analyze でズレ分布を見てから決めるのが安全。
"""

from __future__ import annotations

import argparse
import os
import sys
from bisect import bisect_left
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# --- ROS2 message deserialization ---------------------------------------
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# --- lazy LeRobot import (convert mode only) -----------------------------

def _import_lerobot():
    lerobot_path = os.environ.get("LEROBOT_PATH")
    if lerobot_path and lerobot_path not in sys.path:
        sys.path.append(lerobot_path)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: WPS433
    return LeRobotDataset


# ============================================================
# Bag reading
# ============================================================

# Topics we care about and their expected message types.
DEFAULT_JOINT_STATE = "/lekiwi/joint_states"
DEFAULT_CMD_VEL = "/lekiwi/cmd_vel"
DEFAULT_ARM_CMD = "/lekiwi/arm_joint_commands"
DEFAULT_FRONT_CAM = "/lekiwi/camera/front/image_raw/compressed"
DEFAULT_WRIST_CAM = "/lekiwi/camera/wrist/image_raw/compressed"


@dataclass
class TopicMessages:
    """All messages of one topic within one episode."""

    topic: str
    msg_type: str
    stamps_ns: List[int] = field(default_factory=list)  # sorted
    messages: List[object] = field(default_factory=list)  # same length


def _stamp_ns(msg, recv_ns: int) -> int:
    """Prefer header.stamp; fall back to bag receive time."""
    hdr = getattr(msg, "header", None)
    if hdr is not None:
        s = hdr.stamp.sec * 1_000_000_000 + hdr.stamp.nanosec
        if s > 0:
            return s
    return recv_ns


def read_episode_bag(bag_dir: Path,
                     wanted_topics: List[str]) -> Dict[str, TopicMessages]:
    """Read all wanted messages from an MCAP bag directory.

    `bag_dir` should be the directory `ros2 bag record -o <bag_dir>` created,
    which contains `metadata.yaml` and one or more `.mcap` files.
    """
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map: Dict[str, str] = {
        t.name: t.type for t in reader.get_all_topics_and_types()}

    result: Dict[str, TopicMessages] = {
        t: TopicMessages(topic=t, msg_type=type_map.get(t, ""))
        for t in wanted_topics if t in type_map
    }

    # Set filter to only requested topics that exist.
    keep = [t for t in wanted_topics if t in type_map]
    reader.set_filter(rosbag2_py.StorageFilter(topics=keep))

    msg_classes: Dict[str, object] = {
        t: get_message(type_map[t]) for t in keep}

    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        cls = msg_classes.get(topic)
        if cls is None:
            continue
        msg = deserialize_message(data, cls)
        ts = _stamp_ns(msg, recv_ns)
        tm = result[topic]
        tm.stamps_ns.append(ts)
        tm.messages.append(msg)

    # Ensure stamps are sorted (header.stamp may not always be monotonic).
    for tm in result.values():
        if not tm.stamps_ns:
            continue
        order = sorted(range(len(tm.stamps_ns)), key=lambda i: tm.stamps_ns[i])
        tm.stamps_ns = [tm.stamps_ns[i] for i in order]
        tm.messages = [tm.messages[i] for i in order]

    return result


# ============================================================
# Nearest-neighbor pairing
# ============================================================


def _nearest_index(stamps: List[int], target: int) -> int:
    """Return index in `stamps` (sorted asc) closest to `target`."""
    pos = bisect_left(stamps, target)
    if pos == 0:
        return 0
    if pos == len(stamps):
        return len(stamps) - 1
    before = stamps[pos - 1]
    after = stamps[pos]
    return pos if (after - target) < (target - before) else pos - 1


@dataclass
class MatchStats:
    """Statistics of nearest-neighbor gaps for one episode."""

    base_topic: str
    base_count: int
    # For each non-base topic (including action pseudo-topics), abs gap in ms.
    gaps_ms: Dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def max_gap_per_frame_ms(self) -> np.ndarray:
        if not self.gaps_ms:
            return np.zeros(self.base_count, dtype=np.float64)
        return np.max(np.stack(list(self.gaps_ms.values()), axis=0), axis=0)

    def drop_rate(self, tolerance_ms: float) -> float:
        if self.base_count == 0:
            return 0.0
        return float(np.mean(self.max_gap_per_frame_ms > tolerance_ms))


def compute_match_stats(topic_msgs: Dict[str, TopicMessages],
                        base_topic: str,
                        obs_topics: List[str],
                        action_topics: List[str],
                        action_lag_ms: float) -> MatchStats:
    """Compute per-frame absolute gaps between base topic and paired topics."""
    base = topic_msgs[base_topic]
    stats = MatchStats(base_topic=base_topic, base_count=len(base.stamps_ns))
    lag_ns = int(action_lag_ms * 1_000_000)

    for topic in obs_topics + action_topics:
        if topic == base_topic:
            continue
        other = topic_msgs.get(topic)
        if other is None or not other.stamps_ns:
            stats.gaps_ms[topic] = np.full(
                stats.base_count, np.inf, dtype=np.float64)
            continue
        stamps = other.stamps_ns
        is_action = topic in action_topics
        gaps = np.empty(stats.base_count, dtype=np.float64)
        for i, base_ts in enumerate(base.stamps_ns):
            target = base_ts + (lag_ns if is_action else 0)
            j = _nearest_index(stamps, target)
            gaps[i] = abs(stamps[j] - target) / 1e6  # ns -> ms
        stats.gaps_ms[topic] = gaps

    return stats


def pick_base_topic(topic_msgs: Dict[str, TopicMessages],
                    candidates: List[str]) -> str:
    """Pick the candidate with the lowest average frequency (i.e. slowest)."""
    best: Optional[Tuple[str, float]] = None
    for t in candidates:
        tm = topic_msgs.get(t)
        if tm is None or len(tm.stamps_ns) < 2:
            continue
        span = (tm.stamps_ns[-1] - tm.stamps_ns[0]) / 1e9
        if span <= 0:
            continue
        hz = (len(tm.stamps_ns) - 1) / span
        if best is None or hz < best[1]:
            best = (t, hz)
    if best is None:
        raise RuntimeError(
            "Cannot pick base topic — no candidate had >= 2 messages.")
    return best[0]


# ============================================================
# Analyze mode
# ============================================================


DEFAULT_TOLERANCE_SWEEP_MS = [5, 10, 15, 20, 25, 30, 40, 50, 75, 100]


def print_analysis(session_dir: Path,
                   episode_dirs: List[Path],
                   sync_base_topic: Optional[str],
                   action_lag_ms: float,
                   tolerances_ms: List[float]) -> None:
    obs_topics = [DEFAULT_JOINT_STATE, DEFAULT_FRONT_CAM, DEFAULT_WRIST_CAM]
    action_topics = [DEFAULT_CMD_VEL, DEFAULT_ARM_CMD]
    all_topics = list(dict.fromkeys(obs_topics + action_topics))

    print(f"\n=== Sync analysis for session {session_dir} ===")
    print(f"  action_lag_ms = {action_lag_ms}")

    aggregate_gaps: List[np.ndarray] = []
    per_episode_rows: List[Tuple[str, str, int, np.ndarray]] = []

    for ep_dir in episode_dirs:
        bag_dir = ep_dir / "bag"
        if not bag_dir.exists():
            print(f"  [skip] {ep_dir.name}: no bag/ subdir")
            continue
        msgs = read_episode_bag(bag_dir, all_topics)
        if not msgs:
            print(f"  [skip] {ep_dir.name}: empty bag")
            continue

        # topic frequency summary
        freq_info = []
        for t in all_topics:
            tm = msgs.get(t)
            if tm and len(tm.stamps_ns) >= 2:
                span = (tm.stamps_ns[-1] - tm.stamps_ns[0]) / 1e9
                hz = (len(tm.stamps_ns) - 1) / span if span > 0 else 0.0
                freq_info.append(f"{t}={len(tm.stamps_ns)}msg/{hz:.1f}Hz")
            else:
                freq_info.append(f"{t}=MISSING")

        base = sync_base_topic or pick_base_topic(msgs, obs_topics)
        if base not in msgs:
            print(f"  [skip] {ep_dir.name}: base topic {base} missing")
            continue
        stats = compute_match_stats(
            msgs, base, obs_topics, action_topics, action_lag_ms)
        max_gap = stats.max_gap_per_frame_ms
        aggregate_gaps.append(max_gap)
        per_episode_rows.append(
            (ep_dir.name, base, stats.base_count, max_gap))
        print(f"  {ep_dir.name}: base={base} frames={stats.base_count}")
        for fi in freq_info:
            print(f"      {fi}")

    if not per_episode_rows:
        print("No episodes with usable bags.")
        return

    all_gaps = np.concatenate(aggregate_gaps)
    print("\n--- Drop rate vs. --sync-tolerance-ms ---")
    header = f"{'tol_ms':>8} | {'kept':>8} | {'dropped':>8} | {'drop%':>7}"
    print(header)
    print("-" * len(header))
    for tol in tolerances_ms:
        kept = int(np.sum(all_gaps <= tol))
        dropped = int(np.sum(all_gaps > tol))
        total = kept + dropped
        pct = (dropped / total * 100.0) if total else 0.0
        print(f"{tol:>8.1f} | {kept:>8d} | {dropped:>8d} | {pct:>6.2f}%")

    q = np.quantile(all_gaps, [0.5, 0.9, 0.95, 0.99, 1.0])
    print("\n--- Per-frame max-gap distribution (ms) ---")
    print(f"  p50={q[0]:.2f}  p90={q[1]:.2f}  p95={q[2]:.2f}  "
          f"p99={q[3]:.2f}  max={q[4]:.2f}")


# ============================================================
# Convert mode
# ============================================================


def _joint_state_to_state_vec(js) -> np.ndarray:
    positions = list(js.position)
    if len(positions) >= 6:
        arm = positions[:6]
    elif len(positions) == 5:
        arm = positions[:5] + [0.0]
    else:
        arm = positions + [0.0] * (6 - len(positions))
    if len(js.velocity) >= 3:
        base = [js.velocity[0], js.velocity[1], js.velocity[2]]
    else:
        base = [0.0, 0.0, 0.0]
    return np.array(arm + base, dtype=np.float32)


def _action_vec(arm_cmd, cmd_vel) -> np.ndarray:
    positions = list(arm_cmd.position) if arm_cmd is not None else []
    if len(positions) >= 6:
        arm = positions[:6]
    elif len(positions) == 5:
        arm = positions[:5] + [0.0]
    else:
        arm = positions + [0.0] * (6 - len(positions))
    if cmd_vel is not None:
        base = [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z]
    else:
        base = [0.0, 0.0, 0.0]
    return np.array(arm + base, dtype=np.float32)


def _decode_compressed(msg) -> Optional[np.ndarray]:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _default_features(fps: int, use_videos: bool,
                      front_shape: Tuple[int, int, int],
                      wrist_shape: Tuple[int, int, int]) -> dict:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (9,),
            "names": [
                "arm_shoulder_pan.pos", "arm_shoulder_lift.pos",
                "arm_elbow_flex.pos", "arm_wrist_flex.pos",
                "arm_wrist_roll.pos", "arm_gripper.pos",
                "x.vel", "y.vel", "theta.vel",
            ],
        },
        "observation.images.front": {
            "dtype": "video" if use_videos else "image",
            "shape": front_shape,
            "names": ["height", "width", "channels"],
        },
        "observation.images.wrist": {
            "dtype": "video" if use_videos else "image",
            "shape": wrist_shape,
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (9,),
            "names": [
                "arm_shoulder_pan.pos", "arm_shoulder_lift.pos",
                "arm_elbow_flex.pos", "arm_wrist_flex.pos",
                "arm_wrist_roll.pos", "arm_gripper.pos",
                "x.vel", "y.vel", "theta.vel",
            ],
        },
    }


def convert_session(session_dir: Path,
                    episode_dirs: List[Path],
                    dataset_repo_id: str,
                    dataset_root: Path,
                    fps: int,
                    single_task: str,
                    robot_type: str,
                    use_videos: bool,
                    sync_base_topic: Optional[str],
                    sync_tolerance_ms: float,
                    action_lag_ms: float,
                    resume: bool,
                    front_shape: Tuple[int, int, int],
                    wrist_shape: Tuple[int, int, int]) -> None:
    LeRobotDataset = _import_lerobot()

    obs_topics = [DEFAULT_JOINT_STATE, DEFAULT_FRONT_CAM, DEFAULT_WRIST_CAM]
    action_topics = [DEFAULT_CMD_VEL, DEFAULT_ARM_CMD]
    all_topics = list(dict.fromkeys(obs_topics + action_topics))

    dataset_path = dataset_root / dataset_repo_id
    dataset_exists = (dataset_path / "meta" / "info.json").exists()

    if resume:
        if not dataset_exists:
            raise SystemExit(
                f"--resume given but dataset not found: {dataset_path}")
        dataset = LeRobotDataset(
            repo_id=dataset_repo_id,
            root=str(dataset_path),
            revision="v3.0",
        )
        dataset.start_image_writer(num_processes=0, num_threads=4)
    else:
        if dataset_exists:
            raise SystemExit(
                f"Dataset already exists: {dataset_path}. "
                f"Delete it or use --resume.")
        features = _default_features(
            fps, use_videos, front_shape, wrist_shape)
        dataset = LeRobotDataset.create(
            repo_id=dataset_repo_id,
            fps=fps,
            root=str(dataset_root),
            robot_type=robot_type,
            features=features,
            use_videos=use_videos,
            image_writer_processes=0,
            image_writer_threads=4,
            batch_encoding_size=1,
        )

    total_kept = 0
    total_dropped = 0

    for ep_dir in episode_dirs:
        bag_dir = ep_dir / "bag"
        if not bag_dir.exists():
            print(f"[skip] {ep_dir.name}: no bag/")
            continue
        # per-episode task override
        ep_task = single_task
        ep_meta_path = ep_dir / "episode.yaml"
        if ep_meta_path.exists():
            try:
                ep_meta = yaml.safe_load(ep_meta_path.read_text()) or {}
                ep_task = ep_meta.get("single_task", single_task)
            except Exception:  # noqa: BLE001
                pass

        msgs = read_episode_bag(bag_dir, all_topics)
        if not msgs:
            print(f"[skip] {ep_dir.name}: empty bag")
            continue
        base = sync_base_topic or pick_base_topic(msgs, obs_topics)
        if base not in msgs:
            print(f"[skip] {ep_dir.name}: base topic {base} missing")
            continue
        stats = compute_match_stats(
            msgs, base, obs_topics, action_topics, action_lag_ms)
        max_gap = stats.max_gap_per_frame_ms

        base_stamps = msgs[base].stamps_ns
        lag_ns = int(action_lag_ms * 1_000_000)

        ep_kept = 0
        ep_dropped = 0
        for i, base_ts in enumerate(base_stamps):
            if max_gap[i] > sync_tolerance_ms:
                ep_dropped += 1
                continue

            js_tm = msgs.get(DEFAULT_JOINT_STATE)
            front_tm = msgs.get(DEFAULT_FRONT_CAM)
            wrist_tm = msgs.get(DEFAULT_WRIST_CAM)
            arm_cmd_tm = msgs.get(DEFAULT_ARM_CMD)
            cmd_vel_tm = msgs.get(DEFAULT_CMD_VEL)
            if not (js_tm and front_tm and wrist_tm
                    and arm_cmd_tm and cmd_vel_tm):
                ep_dropped += 1
                continue

            js_msg = js_tm.messages[_nearest_index(js_tm.stamps_ns, base_ts)]
            front_msg = front_tm.messages[
                _nearest_index(front_tm.stamps_ns, base_ts)]
            wrist_msg = wrist_tm.messages[
                _nearest_index(wrist_tm.stamps_ns, base_ts)]
            action_target = base_ts + lag_ns
            arm_cmd_msg = arm_cmd_tm.messages[
                _nearest_index(arm_cmd_tm.stamps_ns, action_target)]
            cmd_vel_msg = cmd_vel_tm.messages[
                _nearest_index(cmd_vel_tm.stamps_ns, action_target)]

            front_img = _decode_compressed(front_msg)
            wrist_img = _decode_compressed(wrist_msg)
            if front_img is None or wrist_img is None:
                ep_dropped += 1
                continue
            # Resize if necessary
            fh, fw, _ = front_shape
            if front_img.shape[:2] != (fh, fw):
                front_img = cv2.resize(front_img, (fw, fh))
            wh, ww, _ = wrist_shape
            if wrist_img.shape[:2] != (wh, ww):
                wrist_img = cv2.resize(wrist_img, (ww, wh))

            frame = {
                "observation.state": _joint_state_to_state_vec(js_msg),
                "observation.images.front": front_img,
                "observation.images.wrist": wrist_img,
                "action": _action_vec(arm_cmd_msg, cmd_vel_msg),
                "task": ep_task,
            }
            dataset.add_frame(frame)
            ep_kept += 1

        try:
            dataset.save_episode()
        except Exception as e:  # noqa: BLE001
            print(f"[error] Failed to save episode {ep_dir.name}: {e}")
            continue

        total_kept += ep_kept
        total_dropped += ep_dropped
        print(f"  {ep_dir.name}: kept={ep_kept} dropped={ep_dropped} "
              f"(base={base})")

    print(f"\nDone. total kept={total_kept} dropped={total_dropped} "
          f"-> {dataset_path}")


# ============================================================
# CLI
# ============================================================


def _find_episodes(session_dir: Path) -> List[Path]:
    return sorted(
        p for p in session_dir.iterdir()
        if p.is_dir() and p.name.startswith("episode_"))


def main() -> None:
    p = argparse.ArgumentParser(
        description="Convert LeKiwi MCAP bags to LeRobot Dataset v3.")
    p.add_argument("--session-dir", type=Path, required=True,
                   help="Session directory produced by lekiwi_bag_recorder.")
    p.add_argument("--mode", choices=["analyze", "convert"], default="analyze",
                   help="analyze: only report sync statistics. "
                        "convert: build LeRobot dataset.")
    p.add_argument("--sync-base-topic", default=None,
                   help="Topic to align to. Default: lowest-freq obs topic.")
    p.add_argument("--sync-tolerance-ms", type=float, default=20.0,
                   help="Max allowed |gap| per frame (convert mode). "
                        "Default 20ms (~half of 33ms @ 30Hz).")
    p.add_argument("--action-lag-ms", type=float, default=0.0,
                   help="Positive value shifts action lookup forward in time. "
                        "Default 0. Try +33ms (1/fps) to compensate teleop "
                        "loop delay if analyze shows systematic action lag.")
    p.add_argument("--tolerance-sweep-ms", type=float, nargs="+",
                   default=DEFAULT_TOLERANCE_SWEEP_MS,
                   help="Tolerance values to report in analyze mode.")
    # convert-only options
    p.add_argument("--dataset-repo-id",
                   help="username/dataset_name (convert mode)")
    p.add_argument("--dataset-root", type=Path,
                   default=Path.home() / "lerobot_datasets")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--single-task", default=None,
                   help="Override task string. Default: from session.yaml.")
    p.add_argument("--robot-type", default="lekiwi_client")
    p.add_argument("--use-videos", action="store_true", default=True)
    p.add_argument("--no-videos", dest="use_videos", action="store_false")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--front-shape", type=int, nargs=3, default=[480, 640, 3],
                   help="H W C for observation.images.front.")
    p.add_argument("--wrist-shape", type=int, nargs=3, default=[640, 480, 3],
                   help="H W C for observation.images.wrist.")
    args = p.parse_args()

    session_dir: Path = args.session_dir.expanduser().resolve()
    if not session_dir.is_dir():
        raise SystemExit(f"Session dir not found: {session_dir}")

    episodes = _find_episodes(session_dir)
    if not episodes:
        raise SystemExit(f"No episode_* dirs under {session_dir}")

    # Resolve defaults from session.yaml if present.
    session_yaml = session_dir / "session.yaml"
    session_meta = {}
    if session_yaml.exists():
        session_meta = yaml.safe_load(session_yaml.read_text()) or {}

    if args.mode == "analyze":
        print_analysis(
            session_dir=session_dir,
            episode_dirs=episodes,
            sync_base_topic=args.sync_base_topic,
            action_lag_ms=args.action_lag_ms,
            tolerances_ms=args.tolerance_sweep_ms,
        )
        return

    # convert mode
    if not args.dataset_repo_id:
        raise SystemExit(
            "--dataset-repo-id is required in convert mode "
            "(format: username/dataset_name)")
    if "/" not in args.dataset_repo_id:
        raise SystemExit(
            "--dataset-repo-id must be in 'username/dataset_name' form.")

    single_task = args.single_task or session_meta.get(
        "single_task", "Pick and place task")
    fps = args.fps or int(session_meta.get("target_fps", 30))
    robot_type = args.robot_type or session_meta.get(
        "robot_type", "lekiwi_client")

    convert_session(
        session_dir=session_dir,
        episode_dirs=episodes,
        dataset_repo_id=args.dataset_repo_id,
        dataset_root=args.dataset_root.expanduser().resolve(),
        fps=fps,
        single_task=single_task,
        robot_type=robot_type,
        use_videos=args.use_videos,
        sync_base_topic=args.sync_base_topic,
        sync_tolerance_ms=args.sync_tolerance_ms,
        action_lag_ms=args.action_lag_ms,
        resume=args.resume,
        front_shape=tuple(args.front_shape),
        wrist_shape=tuple(args.wrist_shape),
    )


if __name__ == "__main__":
    main()
