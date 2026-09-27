#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""Shared metadata utilities for LeKiwi teleop recording and conversion.

Concerns:
  1. Signal semantics (units, frames, absolute/relative, gripper representation,
     pre/post-processing) that must survive raw -> dataset round-trips.
  2. Provenance helpers (git info, sha256, environment snapshots).

These utilities are intentionally dependency-light so they can be imported both
inside the ROS2 recorder node and by the offline conversion CLI.
"""

from __future__ import annotations

import hashlib
import os
import platform
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


# ============================================================
# Signal specification (data semantics)
# ============================================================
#
# This block is the single source of truth for how each signal in the LeKiwi
# dataset is defined. Keep it in sync with lekiwi_data_recorder.py and any
# downstream consumer. Bumping SIGNAL_SPEC_VERSION signals a breaking change
# (units, order, or semantics) so old raw data should be interpreted with the
# old spec.

SIGNAL_SPEC_VERSION = "1.0.0"


def build_signal_spec() -> Dict[str, Any]:
    """Return the canonical LeKiwi signal specification.

    All numeric conventions used at record time. Downstream code (training,
    replay, evaluation) MUST read these values instead of hard-coding them.
    """
    arm_joint_order = [
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    ]
    base_dof_order = ["x", "y", "theta"]

    return {
        "spec_version": SIGNAL_SPEC_VERSION,
        # ---- observation.state (9-dim) --------------------------------
        "observation.state": {
            "layout": [
                {"name": f"{n}.pos", "kind": "arm_joint_position",
                 "unit": "rad", "absolute": True, "range": None,
                 "source_topic": "/lekiwi/joint_states",
                 "source_field": f"position[{i}]"}
                for i, n in enumerate(arm_joint_order)
            ] + [
                {"name": f"{d}.vel", "kind": "base_velocity",
                 "unit": "m/s" if d != "theta" else "rad/s",
                 "absolute": True,
                 "frame": "base_link",
                 "source_topic": "/lekiwi/joint_states",
                 "source_field": f"velocity[{i}]"}
                for i, d in enumerate(base_dof_order)
            ],
            "coordinate_frames": {
                "arm": "leader/follower joint space (SO101 raw encoder units "
                       "post-calibration; unit=rad if calibrated to SI)",
                "base": "base_link (x forward, y left, theta ccw about z)",
            },
            "gripper_representation": {
                "name": "arm_gripper.pos",
                "type": "continuous",
                "closed_when": "position -> min",
                "open_when": "position -> max",
                "note": "Raw follower joint position; no thresholding applied.",
            },
            "preprocessing": {
                "normalization": "none",
                "smoothing": "none",
                "clipping": "none",
            },
        },
        # ---- action (9-dim, same layout as state) ---------------------
        "action": {
            "layout": [
                {"name": f"{n}.pos", "kind": "arm_joint_position_command",
                 "unit": "rad", "absolute": True,
                 "source_topic": "/lekiwi/arm_joint_commands",
                 "source_field": f"position[{i}]"}
                for i, n in enumerate(arm_joint_order)
            ] + [
                {"name": "x.vel", "kind": "base_velocity_command",
                 "unit": "m/s", "absolute": True, "frame": "base_link",
                 "source_topic": "/lekiwi/cmd_vel",
                 "source_field": "linear.x"},
                {"name": "y.vel", "kind": "base_velocity_command",
                 "unit": "m/s", "absolute": True, "frame": "base_link",
                 "source_topic": "/lekiwi/cmd_vel",
                 "source_field": "linear.y"},
                {"name": "theta.vel", "kind": "base_velocity_command",
                 "unit": "rad/s", "absolute": True, "frame": "base_link",
                 "source_topic": "/lekiwi/cmd_vel",
                 "source_field": "angular.z"},
            ],
            "value_type": "absolute",  # not delta / not residual
            "gripper_command": {
                "type": "continuous",
                "note": "Same continuous range as state; teleop sends target "
                        "position, not open/close events.",
            },
            "preprocessing": {
                "normalization": "none",
                "delta_to_state": False,
                "clipping": "none",
            },
        },
        # ---- images ---------------------------------------------------
        "observation.images.front": {
            "source_topic": "/lekiwi/camera/front/image_raw/compressed",
            "encoding_on_wire": "jpeg (sensor_msgs/CompressedImage)",
            "color_space_stored": "RGB",
            "shape_hw": [480, 640],
            "channels": 3,
            "notes": "Decoded with cv2.imdecode then BGR->RGB at conversion "
                     "time. Raw JPEG is kept in the bag verbatim.",
        },
        "observation.images.wrist": {
            "source_topic": "/lekiwi/camera/wrist/image_raw/compressed",
            "encoding_on_wire": "jpeg (sensor_msgs/CompressedImage)",
            "color_space_stored": "RGB",
            "shape_hw": [640, 480],
            "channels": 3,
            "notes": "Wrist camera is rotated 90deg vs front; shape reflects "
                     "sensor orientation as received.",
        },
        # ---- timestamps ----------------------------------------------
        "timestamps": {
            "primary_source": "header.stamp of each ROS message",
            "fallback_source": "rosbag2 receive time when header.stamp==0",
            "clock_domain": "system_clock on the recording host",
            "notes": "See provenance/conversion.json for sync_base_topic, "
                     "sync_tolerance_ms and action_lag_ms used to build the "
                     "final dataset.",
        },
    }


# ============================================================
# Git / environment provenance
# ============================================================


def _run(cmd: List[str], cwd: Optional[Path] = None) -> Optional[str]:
    try:
        out = subprocess.check_output(
            cmd, cwd=str(cwd) if cwd else None,
            stderr=subprocess.DEVNULL, timeout=5)
        return out.decode("utf-8", errors="replace").strip()
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
            FileNotFoundError, OSError):
        return None


def git_info(repo_dir: Path) -> Dict[str, Any]:
    """Best-effort git snapshot for `repo_dir`. All fields may be None."""
    repo_dir = Path(repo_dir)
    commit = _run(["git", "rev-parse", "HEAD"], repo_dir)
    branch = _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], repo_dir)
    origin = _run(["git", "config", "--get", "remote.origin.url"], repo_dir)
    status = _run(["git", "status", "--porcelain"], repo_dir)
    dirty = bool(status) if status is not None else None
    diff = _run(["git", "diff", "HEAD"], repo_dir) if dirty else ""
    return {
        "repo_dir": str(repo_dir),
        "commit": commit,
        "branch": branch,
        "origin": origin,
        "dirty": dirty,
        "diff": diff or "",
    }


_THIS_FILE = Path(__file__).resolve()


def package_git_info() -> Dict[str, Any]:
    """Git info of the lekiwi_ros2_teleop package (this file's repo)."""
    # Walk up looking for a .git dir; stop at filesystem root.
    p = _THIS_FILE.parent
    for cand in [p, *p.parents]:
        if (cand / ".git").exists():
            return git_info(cand)
    return {"repo_dir": str(p), "commit": None, "branch": None,
            "origin": None, "dirty": None, "diff": ""}


def workspace_git_info(workspace_dir: Optional[Path] = None) -> Dict[str, Any]:
    """Git info for the surrounding colcon workspace (best-effort)."""
    if workspace_dir is None:
        # jazzy_ws is typically two levels above src/lekiwi_ros2_teleop/...
        workspace_dir = _THIS_FILE.parents[3]
    if (workspace_dir / ".git").exists():
        return git_info(workspace_dir)
    return {"repo_dir": str(workspace_dir), "commit": None, "branch": None,
            "origin": None, "dirty": None, "diff": ""}


def software_snapshot() -> Dict[str, Any]:
    """Versions of key libraries and the runtime environment."""
    def _ver(mod_name: str) -> Optional[str]:
        try:
            mod = __import__(mod_name)
            return getattr(mod, "__version__", None)
        except Exception:  # noqa: BLE001
            return None

    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "hostname": socket.gethostname(),
        "user": os.environ.get("USER") or os.environ.get("USERNAME"),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "libs": {
            "numpy": _ver("numpy"),
            "opencv": _ver("cv2"),
            "rclpy": _ver("rclpy"),
            "rosbag2_py": _ver("rosbag2_py"),
            "yaml": _ver("yaml"),
        },
        "lerobot_path": os.environ.get("LEROBOT_PATH"),
        "argv": list(sys.argv),
    }


def hash_file(path: Path, algo: str = "sha256",
              chunk_size: int = 1 << 20) -> str:
    h = hashlib.new(algo)
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk_size)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def hash_bag_dir(bag_dir: Path) -> Dict[str, str]:
    """Return {relative_path: sha256} for every *.mcap plus metadata.yaml.

    Path keys are relative to `bag_dir` and sorted for reproducibility.
    """
    bag_dir = Path(bag_dir)
    results: Dict[str, str] = {}
    if not bag_dir.is_dir():
        return results
    candidates: List[Path] = []
    candidates.extend(sorted(bag_dir.rglob("*.mcap")))
    metadata = bag_dir / "metadata.yaml"
    if metadata.exists():
        candidates.append(metadata)
    for p in candidates:
        rel = p.relative_to(bag_dir).as_posix()
        results[rel] = hash_file(p)
    return results
