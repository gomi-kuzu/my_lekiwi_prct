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
import json
import os
import shutil
import sys
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# --- ROS2 message deserialization ---------------------------------------
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# --- shared provenance / signal-spec helpers ----------------------------
try:
    from .lekiwi_metadata import (
        build_signal_spec,
        package_git_info,
        software_snapshot,
        workspace_git_info,
        SIGNAL_SPEC_VERSION,
    )
except ImportError:  # allow running as a script
    from lekiwi_metadata import (  # type: ignore
        build_signal_spec,
        package_git_info,
        software_snapshot,
        workspace_git_info,
        SIGNAL_SPEC_VERSION,
    )
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


def compute_overlap_window(topic_msgs: Dict[str, "TopicMessages"],
                           topics: List[str]) -> Optional[Tuple[int, int]]:
    """Return the time window [start_ns, end_ns] in which *all* given topics
    have messages, i.e. ``[max(first stamp), min(last stamp)]``.

    This is the interval where every topic is actually streaming, so pairing
    the base topic only inside it removes the episode-edge "holes" where an
    action topic (leader arm / joy) has not started yet or already stopped.
    Topics with no messages are ignored (they cannot bound the overlap and are
    handled separately as fully-missing). Returns ``None`` if there is no
    overlap or no usable topic.
    """
    firsts: List[int] = []
    lasts: List[int] = []
    for t in topics:
        tm = topic_msgs.get(t)
        if tm and tm.stamps_ns:
            firsts.append(tm.stamps_ns[0])
            lasts.append(tm.stamps_ns[-1])
    if not firsts:
        return None
    start = max(firsts)
    end = min(lasts)
    if start > end:
        return None
    return start, end


def trim_base_to_window(topic_msgs: Dict[str, "TopicMessages"],
                        base_topic: str,
                        window: Optional[Tuple[int, int]]) -> Tuple[int, int]:
    """Restrict the base topic's frames to ``window`` (mutates the entry).

    Only the base topic is trimmed; the paired topics keep all their messages
    so nearest-neighbor lookups near the edges still have valid candidates.
    Returns ``(n_trimmed_head, n_trimmed_tail)``.
    """
    if window is None:
        return 0, 0
    base = topic_msgs.get(base_topic)
    if base is None or not base.stamps_ns:
        return 0, 0
    start, end = window
    stamps = base.stamps_ns
    lo = bisect_left(stamps, start)
    hi = bisect_right(stamps, end)
    n_head = lo
    n_tail = len(stamps) - hi
    if n_head == 0 and n_tail == 0:
        return 0, 0
    base.stamps_ns = stamps[lo:hi]
    base.messages = base.messages[lo:hi]
    return n_head, n_tail



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


def _short_topic(t: str, width: int = 22) -> str:
    # Drop the common /lekiwi/ prefix and the verbose image suffix so that
    # e.g. camera/front and camera/wrist stay distinguishable in legends.
    s = t
    if s.startswith("/lekiwi/"):
        s = s[len("/lekiwi/"):]
    s = s.replace("/image_raw/compressed", "").replace("/compressed", "")
    s = s.lstrip("/")
    if len(s) <= width:
        return s
    return "..." + s[-(width - 3):]


def plot_episode_timing(ep_name: str,
                        raw_stamps_ns: Dict[str, np.ndarray],
                        base_topic: str,
                        window_ns: Optional[Tuple[int, int]],
                        base_stamps_trimmed_ns: np.ndarray,
                        gaps_ms: Dict[str, np.ndarray],
                        sync_tolerance_ms: float,
                        out_path: Path,
                        start_s: float = 0.0,
                        end_s: Optional[float] = 5.0) -> Optional[Path]:
    """Render a 3-panel timing figure for one episode and save it.

    Panels:
      1. Inter-message period Δt[ms] per topic (raw, pre-sync).
      2. Raw message-time raster per topic with the overlap window shaded
         (shows cross-topic phase / the episode-edge holes).
      3. Nearest-neighbor pairing gap[ms] per topic at each (trimmed) base
         frame, i.e. the state *after* synchronization, with the tolerance
         line drawn for reference.

    Returns the written path, or ``None`` if matplotlib is unavailable.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print(f"  [plot] skipped ({e}); install matplotlib to enable plots")
        return None

    if not raw_stamps_ns:
        return None
    t0 = min(int(arr[0]) for arr in raw_stamps_ns.values() if arr.size)
    topics = list(raw_stamps_ns.keys())

    # Time window (relative seconds from the first message). Clamp to the data.
    t_last = max((int(arr[-1]) for arr in raw_stamps_ns.values()
                  if arr.size), default=t0)
    data_end_s = (t_last - t0) / 1e9
    win_lo = max(0.0, start_s)
    win_hi = end_s if end_s is not None else data_end_s
    if win_hi <= win_lo:
        win_hi = data_end_s

    def _mask(x: np.ndarray) -> np.ndarray:
        return (x >= win_lo) & (x <= win_hi)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f"Timing analysis — {ep_name} (base={base_topic})  "
                 f"window [{win_lo:.2f}, {win_hi:.2f}] s")

    # Panel 1: inter-message period Δt
    ax = axes[0]
    for t in topics:
        s = raw_stamps_ns[t]
        if s.size < 2:
            continue
        x = (s[1:] - t0) / 1e9
        dt_ms = np.diff(s) / 1e6
        m = _mask(x)
        if not np.any(m):
            continue
        ax.plot(x[m], dt_ms[m], marker=".", ms=3, lw=0.8,
                label=_short_topic(t))
    ax.set_xlim(win_lo, win_hi)
    ax.set_title("1) Inter-message period Δt (raw, pre-sync)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Δt [ms]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right", ncol=2)

    # Panel 2: raw message-time raster
    ax = axes[1]
    for i, t in enumerate(topics):
        s = raw_stamps_ns[t]
        if s.size == 0:
            continue
        x = (s - t0) / 1e9
        m = _mask(x)
        if not np.any(m):
            continue
        xm = x[m]
        ax.plot(xm, np.full_like(xm, i, dtype=float), "|", ms=8,
                color=f"C{i % 10}")
    if window_ns is not None:
        ws = (window_ns[0] - t0) / 1e9
        we = (window_ns[1] - t0) / 1e9
        ax.axvspan(ws, we, color="green", alpha=0.08,
                   label="all-topic overlap window")
        ax.axvline(ws, color="green", ls="--", lw=0.8)
        ax.axvline(we, color="green", ls="--", lw=0.8)
        ax.legend(fontsize=8, loc="upper right")
    ax.set_xlim(win_lo, win_hi)
    ax.set_yticks(range(len(topics)))
    ax.set_yticklabels([_short_topic(t) for t in topics], fontsize=8)
    ax.set_title("2) Raw message times (pre-sync); shaded = overlap window")
    ax.set_xlabel("time [s]")
    ax.grid(True, axis="x", alpha=0.3)

    # Panel 3: pairing gap after synchronization
    ax = axes[2]
    if base_stamps_trimmed_ns.size:
        xb = (base_stamps_trimmed_ns - t0) / 1e9
        for t, g in gaps_ms.items():
            finite = np.isfinite(g) & _mask(xb)
            if not np.any(finite):
                continue
            ax.plot(xb[finite], g[finite], marker=".", ms=3, lw=0.8,
                    label=_short_topic(t))
    ax.axhline(sync_tolerance_ms, color="red", ls="--", lw=1.0,
               label=f"tolerance {sync_tolerance_ms:.0f} ms")
    ax.set_xlim(win_lo, win_hi)
    ax.set_title("3) Nearest-neighbor gap at each base frame (post-sync)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("gap [ms]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right", ncol=2)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def print_analysis(session_dir: Path,
                   episode_dirs: List[Path],
                   sync_base_topic: Optional[str],
                   action_lag_ms: float,
                   tolerances_ms: List[float],
                   trim_to_overlap: bool = True,
                   plot: bool = False,
                   plot_dir: Optional[Path] = None,
                   plot_episode: Optional[str] = None,
                   plot_tolerance_ms: float = 20.0,
                   plot_start_s: float = 0.0,
                   plot_end_s: Optional[float] = 5.0) -> None:
    obs_topics = [DEFAULT_JOINT_STATE, DEFAULT_FRONT_CAM, DEFAULT_WRIST_CAM]
    action_topics = [DEFAULT_CMD_VEL, DEFAULT_ARM_CMD]
    all_topics = list(dict.fromkeys(obs_topics + action_topics))

    print(f"\n=== Sync analysis for session {session_dir} ===")
    print(f"  action_lag_ms = {action_lag_ms}")
    print(f"  trim_to_overlap = {trim_to_overlap}")
    if plot:
        print(f"  plot = True (episode={plot_episode or 'first usable'})")
    plot_done = False

    aggregate_gaps: List[np.ndarray] = []
    per_episode_rows: List[Tuple[str, str, int, np.ndarray]] = []
    # Per-paired-topic gaps aggregated across episodes, so we can tell which
    # topic is responsible for the max-gap (and therefore the drops).
    per_topic_gaps: Dict[str, List[np.ndarray]] = {}
    # How often each topic is the single worst (argmax) contributor per frame.
    per_topic_worst_count: Dict[str, int] = {}
    total_frames_for_worst = 0

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

        # Detect entirely-missing topics so we don't conflate
        # "topic never recorded" with "topic out of sync tolerance".
        present_obs = [t for t in obs_topics
                       if msgs.get(t) and len(msgs[t].stamps_ns) >= 1]
        present_act = [t for t in action_topics
                       if msgs.get(t) and len(msgs[t].stamps_ns) >= 1]
        missing_topics = [t for t in obs_topics + action_topics
                          if t not in present_obs + present_act]

        # Decide whether to plot this episode (first usable, or the named one).
        want_plot = plot and not plot_done and (
            plot_episode is None or ep_dir.name == plot_episode)
        # Snapshot raw stamps + overlap window *before* trimming mutates them.
        raw_stamps_ns: Dict[str, np.ndarray] = {}
        plot_window: Optional[Tuple[int, int]] = None
        if want_plot:
            raw_stamps_ns = {
                t: np.array(msgs[t].stamps_ns, dtype=np.int64)
                for t in present_obs + present_act if msgs.get(t)}
            plot_window = compute_overlap_window(
                msgs, present_obs + present_act)

        # Auto-trim the base topic to the window where *all present* topics
        # are streaming, removing episode-edge frames where an action topic
        # has not started yet / has already stopped.
        n_head = n_tail = 0
        if trim_to_overlap:
            window = compute_overlap_window(
                msgs, present_obs + present_act)
            n_head, n_tail = trim_base_to_window(msgs, base, window)

        stats = compute_match_stats(
            msgs, base, present_obs, present_act, action_lag_ms)
        max_gap = stats.max_gap_per_frame_ms
        aggregate_gaps.append(max_gap)
        per_episode_rows.append(
            (ep_dir.name, base, stats.base_count, max_gap))

        if want_plot:
            out_dir = plot_dir or (session_dir / "analysis_plots")
            out_path = out_dir / f"timing_{ep_dir.name}.png"
            written = plot_episode_timing(
                ep_name=ep_dir.name,
                raw_stamps_ns=raw_stamps_ns,
                base_topic=base,
                window_ns=plot_window,
                base_stamps_trimmed_ns=np.array(
                    msgs[base].stamps_ns, dtype=np.int64),
                gaps_ms=stats.gaps_ms,
                sync_tolerance_ms=plot_tolerance_ms,
                out_path=out_path,
                start_s=plot_start_s,
                end_s=plot_end_s,
            )
            if written is not None:
                plot_done = True
                print(f"  [plot] wrote {written}")

        # Accumulate per-topic gaps and per-frame worst contributor.
        if stats.gaps_ms:
            paired = list(stats.gaps_ms.keys())
            for t in paired:
                per_topic_gaps.setdefault(t, []).append(stats.gaps_ms[t])
            stacked = np.stack([stats.gaps_ms[t] for t in paired], axis=0)
            worst_idx = np.argmax(stacked, axis=0)
            for k, t in enumerate(paired):
                per_topic_worst_count[t] = (
                    per_topic_worst_count.get(t, 0)
                    + int(np.sum(worst_idx == k)))
            total_frames_for_worst += stats.base_count

        print(f"  {ep_dir.name}: base={base} frames={stats.base_count}")
        for fi in freq_info:
            print(f"      {fi}")
        if trim_to_overlap and (n_head or n_tail):
            print(f"      [trim] dropped {n_head} head + {n_tail} tail base "
                  f"frames outside the all-topic overlap window")
        # Recommend an --fps that matches the base topic's real rate, since
        # LeRobot stores timestamp = frame_index / fps.
        base_tm = msgs.get(base)
        if base_tm and len(base_tm.stamps_ns) >= 2:
            span = (base_tm.stamps_ns[-1] - base_tm.stamps_ns[0]) / 1e9
            base_hz = (len(base_tm.stamps_ns) - 1) / span if span > 0 else 0.0
            if base_hz > 0:
                print(f"      -> set --fps {round(base_hz)} at convert "
                      f"(base '{base}' runs at ~{base_hz:.1f}Hz)")
        if missing_topics:
            print(f"      [WARN] entirely-missing topics excluded from "
                  f"gap stats: {missing_topics}")
            print(f"             -> at convert time every frame would be "
                  f"DROPPED regardless of --sync-tolerance-ms.")

    if not per_episode_rows:
        print("No episodes with usable bags.")
        return

    all_gaps = np.concatenate(aggregate_gaps)
    finite_gaps = all_gaps[np.isfinite(all_gaps)]
    n_inf = int(all_gaps.size - finite_gaps.size)

    print("\n--- Drop rate vs. --sync-tolerance-ms ---")
    print("  (based on topics actually present; frames with a fully-missing "
          "topic are always dropped and shown separately)")
    header = f"{'tol_ms':>8} | {'kept':>8} | {'dropped':>8} | {'drop%':>7}"
    print(header)
    print("-" * len(header))
    for tol in tolerances_ms:
        kept = int(np.sum(finite_gaps <= tol))
        dropped_sync = int(np.sum(finite_gaps > tol))
        dropped_total = dropped_sync + n_inf
        total = kept + dropped_total
        pct = (dropped_total / total * 100.0) if total else 0.0
        print(f"{tol:>8.1f} | {kept:>8d} | {dropped_total:>8d} | "
              f"{pct:>6.2f}%")
    if n_inf > 0:
        print(f"  note: {n_inf} frame(s) are unconditionally dropped due to "
              f"a fully-missing paired topic.")

    print("\n--- Per-frame max-gap distribution (ms) ---")
    if finite_gaps.size == 0:
        print("  (no finite gaps — every frame has at least one missing "
              "paired topic; fix data collection before choosing a "
              "tolerance)")
    else:
        q = np.quantile(finite_gaps, [0.5, 0.9, 0.95, 0.99, 1.0])
        print(f"  p50={q[0]:.2f}  p90={q[1]:.2f}  p95={q[2]:.2f}  "
              f"p99={q[3]:.2f}  max={q[4]:.2f}  "
              f"(over {finite_gaps.size} frames with all paired topics "
              f"present)")

    # Per-topic gap breakdown: which paired topic drives the max-gap (= drops).
    if per_topic_gaps:
        print("\n--- Per-paired-topic gap breakdown (ms) ---")
        print("  gap = |base stamp - nearest message of this topic|. "
              "The frame's max-gap is the worst of these.")
        header = (f"  {'topic':<48} | {'p50':>7} | {'p95':>7} | "
                  f"{'max':>7} | {'worst%':>7}")
        print(header)
        print("  " + "-" * (len(header) - 2))
        # Order by p95 descending so the worst offender is on top.
        rows = []
        for t, chunks in per_topic_gaps.items():
            g = np.concatenate(chunks)
            g = g[np.isfinite(g)]
            if g.size == 0:
                rows.append((t, float("inf"), float("inf"),
                             float("inf"), per_topic_worst_count.get(t, 0)))
                continue
            p50, p95, gmax = np.quantile(g, [0.5, 0.95, 1.0])
            rows.append((t, p50, p95, gmax,
                         per_topic_worst_count.get(t, 0)))
        rows.sort(key=lambda r: (r[2] if np.isfinite(r[2]) else 1e18),
                  reverse=True)
        for t, p50, p95, gmax, worst in rows:
            worst_pct = (worst / total_frames_for_worst * 100.0
                         if total_frames_for_worst else 0.0)
            short = t if len(t) <= 48 else "..." + t[-45:]
            print(f"  {short:<48} | {p50:>7.2f} | {p95:>7.2f} | "
                  f"{gmax:>7.2f} | {worst_pct:>6.1f}%")
        print("  worst% = share of frames where this topic is the single "
              "largest gap (i.e. the one causing the drop).")


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
                    wrist_shape: Tuple[int, int, int],
                    trim_to_overlap: bool = True) -> None:

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
    episode_records: List[Dict[str, object]] = []

    for ep_dir in episode_dirs:
        bag_dir = ep_dir / "bag"
        if not bag_dir.exists():
            print(f"[skip] {ep_dir.name}: no bag/")
            continue
        # per-episode task override + source provenance
        ep_task = single_task
        ep_meta_dict: Dict[str, object] = {}
        ep_meta_path = ep_dir / "episode.yaml"
        if ep_meta_path.exists():
            try:
                ep_meta_dict = yaml.safe_load(ep_meta_path.read_text()) or {}
                ep_task = ep_meta_dict.get("single_task", single_task)
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

        # Auto-trim the base topic to the window where all present topics are
        # streaming, so episode-edge frames (action topic not started yet /
        # already stopped) are not force-dropped by the sync tolerance.
        if trim_to_overlap:
            present = [t for t in all_topics
                       if msgs.get(t) and msgs[t].stamps_ns]
            window = compute_overlap_window(msgs, present)
            n_head, n_tail = trim_base_to_window(msgs, base, window)
            if n_head or n_tail:
                print(f"  {ep_dir.name}: [trim] dropped {n_head} head + "
                      f"{n_tail} tail base frames outside overlap window")

        stats = compute_match_stats(
            msgs, base, obs_topics, action_topics, action_lag_ms)
        max_gap = stats.max_gap_per_frame_ms

        base_stamps = msgs[base].stamps_ns

        # Sanity check: LeRobot assigns timestamp = frame_index / fps, so the
        # base topic's actual rate must match --fps or the stored timestamps
        # become physically wrong (e.g. 15Hz data labelled as 30Hz).
        if len(base_stamps) >= 2:
            span = (base_stamps[-1] - base_stamps[0]) / 1e9
            base_hz = (len(base_stamps) - 1) / span if span > 0 else 0.0
            if base_hz > 0 and abs(base_hz - fps) / fps > 0.15:
                print(f"  [WARN] {ep_dir.name}: base topic '{base}' measured "
                      f"~{base_hz:.1f}Hz but --fps={fps}. Stored timestamps "
                      f"(index/fps) will not match real time. Re-run with "
                      f"--fps {round(base_hz)} or pick a base topic that runs "
                      f"at {fps}Hz.")

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

        # Per-episode provenance record
        q = np.quantile(max_gap, [0.5, 0.9, 0.95, 0.99, 1.0]) \
            if len(max_gap) else np.zeros(5)
        episode_records.append({
            "episode_dir": ep_dir.name,
            "source_session_uuid": ep_meta_dict.get("session_uuid"),
            "source_episode_uuid": ep_meta_dict.get("episode_uuid"),
            "source_episode_index": ep_meta_dict.get("episode_index"),
            "bag_sha256": ep_meta_dict.get("bag_sha256", {}),
            "start_time": ep_meta_dict.get("start_time"),
            "stop_time": ep_meta_dict.get("stop_time"),
            "duration_sec": ep_meta_dict.get("duration_sec"),
            "single_task": ep_task,
            "sync_base_topic_used": base,
            "frames_kept": ep_kept,
            "frames_dropped": ep_dropped,
            "max_gap_ms_p50": float(q[0]),
            "max_gap_ms_p90": float(q[1]),
            "max_gap_ms_p95": float(q[2]),
            "max_gap_ms_p99": float(q[3]),
            "max_gap_ms_max": float(q[4]),
        })

    print(f"\nDone. total kept={total_kept} dropped={total_dropped} "
          f"-> {dataset_path}")

    # -------- Provenance emission --------
    _write_conversion_provenance(
        dataset_path=dataset_path,
        session_dir=session_dir,
        episode_records=episode_records,
        params={
            "sync_base_topic": sync_base_topic,
            "sync_tolerance_ms": sync_tolerance_ms,
            "action_lag_ms": action_lag_ms,
            "fps": fps,
            "use_videos": use_videos,
            "front_shape": list(front_shape),
            "wrist_shape": list(wrist_shape),
            "robot_type": robot_type,
            "single_task_default": single_task,
            "resume": resume,
        },
        totals={"kept": total_kept, "dropped": total_dropped},
    )
    _inject_x_lekiwi_into_info_json(
        dataset_path=dataset_path,
        session_dir=session_dir,
        episode_records=episode_records,
        params={
            "sync_base_topic": sync_base_topic,
            "sync_tolerance_ms": sync_tolerance_ms,
            "action_lag_ms": action_lag_ms,
        },
        totals={"kept": total_kept, "dropped": total_dropped},
    )


# ============================================================
# Provenance writers
# ============================================================


def _load_session_meta(session_dir: Path) -> Dict[str, object]:
    session_yaml = session_dir / "session.yaml"
    if session_yaml.exists():
        try:
            return yaml.safe_load(session_yaml.read_text()) or {}
        except Exception:  # noqa: BLE001
            return {}
    return {}


def _load_session_software(session_dir: Path) -> Dict[str, object]:
    p = session_dir / "software.json"
    if p.exists():
        try:
            return json.loads(p.read_text())
        except Exception:  # noqa: BLE001
            return {}
    return {}


def _write_conversion_provenance(dataset_path: Path,
                                 session_dir: Path,
                                 episode_records: List[Dict[str, object]],
                                 params: Dict[str, object],
                                 totals: Dict[str, int]) -> None:
    prov = dataset_path / "provenance"
    prov.mkdir(parents=True, exist_ok=True)

    session_meta = _load_session_meta(session_dir)
    session_software = _load_session_software(session_dir)

    conversion = {
        "generated_at": datetime.now(timezone.utc).isoformat(
            timespec="seconds"),
        "params": params,
        "totals": totals,
        "signal_spec_version": SIGNAL_SPEC_VERSION,
        "signal_spec": build_signal_spec(),
        "conversion_software": software_snapshot(),
        "conversion_git_package": package_git_info(),
        "conversion_git_workspace": workspace_git_info(),
        "source": {
            "session_dir": str(session_dir),
            "session_uuid": session_meta.get("session_uuid"),
            "session_name": session_meta.get("session_name"),
            "session_created_at": session_meta.get("created_at"),
            "operator": session_meta.get("operator"),
            "location": session_meta.get("location"),
            "note": session_meta.get("note"),
            "hardware": session_meta.get("hardware"),
            "record_software": session_software.get("software"),
            "record_git_package": session_software.get("git_package"),
            "record_git_workspace": session_software.get("git_workspace"),
        },
        "episodes": episode_records,
    }
    (prov / "conversion.json").write_text(
        json.dumps(conversion, indent=2, ensure_ascii=False, default=str))

    # Snapshot the actual conversion script for reproducibility.
    try:
        shutil.copy2(Path(__file__), prov / "rosbag_to_lerobot.py")
    except Exception:  # noqa: BLE001
        pass
    # Copy the session-level provenance files verbatim.
    for name in ("session.yaml", "software.json",
                 "git_package.diff", "git_workspace.diff"):
        src = session_dir / name
        if src.exists():
            try:
                shutil.copy2(src, prov / f"source_{name}")
            except Exception:  # noqa: BLE001
                pass


def _inject_x_lekiwi_into_info_json(dataset_path: Path,
                                    session_dir: Path,
                                    episode_records: List[Dict[str, object]],
                                    params: Dict[str, object],
                                    totals: Dict[str, int]) -> None:
    """Add a namespaced summary block to meta/info.json.

    All custom fields live under the single top-level key ``x_lekiwi`` to
    minimize the chance of colliding with future LeRobot schema keys.
    """
    info_path = dataset_path / "meta" / "info.json"
    if not info_path.exists():
        return
    try:
        info = json.loads(info_path.read_text())
    except Exception as e:  # noqa: BLE001
        print(f"[warn] Could not parse info.json for x_lekiwi injection: {e}")
        return

    session_meta = _load_session_meta(session_dir)
    session_software = _load_session_software(session_dir)
    pkg_git = session_software.get("git_package") or {}
    convert_git = package_git_info()

    x = {
        "schema_version": "1.0.0",
        "signal_spec_version": SIGNAL_SPEC_VERSION,
        "signal_spec": build_signal_spec(),
        "source_session": {
            "session_uuid": session_meta.get("session_uuid"),
            "session_name": session_meta.get("session_name"),
            "session_dir": str(session_dir),
            "operator": session_meta.get("operator"),
            "location": session_meta.get("location"),
            "note": session_meta.get("note"),
        },
        "record_git_commit": pkg_git.get("commit"),
        "record_git_dirty": pkg_git.get("dirty"),
        "conversion_git_commit": convert_git.get("commit"),
        "conversion_git_dirty": convert_git.get("dirty"),
        "conversion_params": params,
        "conversion_totals": totals,
        "episode_summary": [
            {k: r.get(k) for k in (
                "source_episode_uuid", "source_episode_index",
                "sync_base_topic_used",
                "frames_kept", "frames_dropped",
                "max_gap_ms_p95", "max_gap_ms_max")}
            for r in episode_records
        ],
        "provenance_dir": "provenance/",
        "provenance_note": (
            "Full raw hashes, git diffs, and per-frame stats live in "
            "provenance/conversion.json. This info.json block is a summary."
        ),
    }

    info["x_lekiwi"] = x
    info_path.write_text(
        json.dumps(info, indent=2, ensure_ascii=False, default=str))


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
    p.add_argument("--trim-to-overlap", action="store_true", default=True,
                   help="Trim base-topic frames to the window where all "
                        "topics are streaming (removes episode-edge holes). "
                        "On by default.")
    p.add_argument("--no-trim-to-overlap", dest="trim_to_overlap",
                   action="store_false",
                   help="Disable the all-topic overlap trim.")
    p.add_argument("--plot", action="store_true", default=False,
                   help="Analyze mode: render a timing figure (message "
                        "period + raw raster + post-sync gap) for one episode.")
    p.add_argument("--plot-dir", type=Path, default=None,
                   help="Where to save plots. Default: "
                        "<session-dir>/analysis_plots/.")
    p.add_argument("--plot-episode", default=None,
                   help="Episode dir name to plot (e.g. episode_000000). "
                        "Default: first usable episode.")
    p.add_argument("--plot-start-s", type=float, default=0.0,
                   help="Plot window start [s] from episode start. "
                        "Default 0.")
    p.add_argument("--plot-end-s", type=float, default=5.0,
                   help="Plot window end [s] from episode start. "
                        "Default 5. Use a negative value to plot to the end.")
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
            trim_to_overlap=args.trim_to_overlap,
            plot=args.plot,
            plot_dir=(args.plot_dir.expanduser().resolve()
                      if args.plot_dir else None),
            plot_episode=args.plot_episode,
            plot_tolerance_ms=args.sync_tolerance_ms,
            plot_start_s=args.plot_start_s,
            plot_end_s=(None if args.plot_end_s < 0 else args.plot_end_s),
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
        trim_to_overlap=args.trim_to_overlap,
    )


if __name__ == "__main__":
    main()
