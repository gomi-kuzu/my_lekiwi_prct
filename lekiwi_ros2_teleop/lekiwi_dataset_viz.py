#!/usr/bin/env python3

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
LeKiwi Dataset Visualization Tool (Fixed version)

This is a fixed version of lerobot_dataset_viz.py that correctly handles episode filtering.
The original version had a bug where it used cumulative frame indices after filtering episodes,
causing IndexError when visualizing episodes other than the first one.

Fixed: EpisodeSampler now uses the actual dataset length after filtering instead of
cumulative indices from the original unfiltered dataset metadata.

Examples:

- Visualize a specific episode:
```
python lekiwi_dataset_viz.py \
    --repo-id gomi-kuzu/lekiwi_pick_place \
    --root ~/lerobot_datasets \
    --episode-index 5
```

- Save visualization to file:
```
python lekiwi_dataset_viz.py \
    --repo-id gomi-kuzu/lekiwi_pick_place \
    --root ~/lerobot_datasets \
    --episode-index 5 \
    --save 1 \
    --output-dir ./viz_output
```

- Run as ROS2 node (distant mode):
```
python lekiwi_dataset_viz.py \
    --repo-id gomi-kuzu/lekiwi_pick_place \
    --root ~/lerobot_datasets \
    --episode-index 5 \
    --mode distant \
    --ws-port 9087

# Then on local machine:
# rerun ws://localhost:9087
```
"""

import argparse
import gc
import logging
import time
from collections.abc import Iterator
from pathlib import Path

import numpy as np
import rerun as rr
import torch
import torch.utils.data
import tqdm

# Monkey-patch get_safe_version to avoid HuggingFace Hub API calls for local datasets
# This must be done BEFORE importing LeRobotDataset
from lerobot.datasets import utils as lerobot_utils

def _get_safe_version_offline(repo_id: str, version: str):
    """Return the version without checking HuggingFace Hub.
    
    This allows loading local datasets without network access.
    """
    return version if isinstance(version, str) else str(version)

# Apply the patch to utils module
lerobot_utils.get_safe_version = _get_safe_version_offline

# Also patch snapshot_download to avoid HuggingFace Hub access
from huggingface_hub import snapshot_download as _original_snapshot_download

def _snapshot_download_local(repo_id, *, repo_type='dataset', revision=None, local_dir=None, **kwargs):
    """Return local directory without attempting to download from Hub."""
    if local_dir:
        local_path = Path(local_dir) / repo_id
        if local_path.exists():
            return str(local_path)
        return local_dir
    return str(Path.home() / '.cache' / 'huggingface' / 'lerobot' / repo_id)

import huggingface_hub
huggingface_hub.snapshot_download = _snapshot_download_local

# Now import LeRobotDataset (after patching)
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Also patch the lerobot_dataset module's global namespace
import lerobot.datasets.lerobot_dataset as lerobot_dataset_module
lerobot_dataset_module.get_safe_version = _get_safe_version_offline

from lerobot.utils.constants import ACTION, DONE, OBS_STATE, REWARD


class EpisodeSampler(torch.utils.data.Sampler):
    """Fixed sampler that works with filtered datasets.
    
    The original version used cumulative indices from dataset.meta.episodes which are
    based on the unfiltered dataset. This caused IndexError when the dataset was filtered
    to a specific episode using select_episodes() or episodes=[...] parameter.
    
    This fixed version simply iterates over the actual length of the filtered dataset.
    """
    def __init__(self, dataset: LeRobotDataset, episode_index: int):
        # After filtering with episodes=[episode_index], the dataset only contains
        # frames from that episode. So we just need to iterate from 0 to len(dataset).
        self.frame_ids = range(len(dataset))
        self.episode_index = episode_index

    def __iter__(self) -> Iterator:
        return iter(self.frame_ids)

    def __len__(self) -> int:
        return len(self.frame_ids)


def to_hwc_uint8_numpy(chw_float32_torch: torch.Tensor) -> np.ndarray:
    """Convert CHW float32 torch tensor to HWC uint8 numpy array."""
    assert chw_float32_torch.dtype == torch.float32
    assert chw_float32_torch.ndim == 3
    c, h, w = chw_float32_torch.shape
    assert c < h and c < w, f"expect channel first images, but instead {chw_float32_torch.shape}"
    hwc_uint8_numpy = (chw_float32_torch * 255).type(torch.uint8).permute(1, 2, 0).numpy()
    return hwc_uint8_numpy


def visualize_dataset(
    dataset: LeRobotDataset,
    episode_index: int,
    batch_size: int = 32,
    num_workers: int = 0,
    mode: str = "local",
    web_port: int = 9090,
    ws_port: int = 9087,
    save: bool = False,
    output_dir: Path | None = None,
) -> Path | None:
    """Visualize a dataset episode using rerun.
    
    Args:
        dataset: LeRobotDataset instance (already filtered to specific episode)
        episode_index: Episode index to display (used for naming)
        batch_size: Batch size for dataloader
        num_workers: Number of worker processes for dataloader
        mode: 'local' or 'distant' viewing mode
        web_port: Web port for distant mode
        ws_port: WebSocket port for distant mode
        save: Whether to save to .rrd file
        output_dir: Directory to save .rrd file
        
    Returns:
        Path to saved .rrd file if save=True, None otherwise
    """
    if save:
        assert output_dir is not None, (
            "Set an output directory where to write .rrd files with `--output-dir path/to/directory`."
        )

    repo_id = dataset.repo_id

    logging.info("Loading dataloader")
    episode_sampler = EpisodeSampler(dataset, episode_index)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=num_workers,
        batch_size=batch_size,
        sampler=episode_sampler,
    )

    logging.info("Starting Rerun")

    if mode not in ["local", "distant"]:
        raise ValueError(f"Invalid mode: {mode}. Must be 'local' or 'distant'")

    spawn_local_viewer = mode == "local" and not save
    rr.init(f"{repo_id}/episode_{episode_index}", spawn=spawn_local_viewer)

    # Manually call python garbage collector after `rr.init` to avoid hanging in a blocking flush
    # when iterating on a dataloader with `num_workers` > 0
    # TODO(rcadene): remove `gc.collect` when rerun version 0.16 is out, which includes a fix
    gc.collect()

    if mode == "distant":
        rr.serve_web_viewer(open_browser=False, web_port=web_port, ws_port=ws_port)

    logging.info(f"Logging episode {episode_index} to Rerun ({len(dataset)} frames)")

    for batch in tqdm.tqdm(dataloader, total=len(dataloader)):
        # iterate over the batch
        for i in range(len(batch["index"])):
            rr.set_time("frame_index", sequence=batch["frame_index"][i].item())
            rr.set_time("timestamp", timestamp=batch["timestamp"][i].item())

            # display each camera image
            for key in dataset.meta.camera_keys:
                # TODO(rcadene): add `.compress()`? is it lossless?
                rr.log(key, rr.Image(to_hwc_uint8_numpy(batch[key][i])))

            # display each dimension of action space (e.g. actuators command)
            if ACTION in batch:
                for dim_idx, val in enumerate(batch[ACTION][i]):
                    rr.log(f"{ACTION}/{dim_idx}", rr.Scalars(val.item()))

            # display each dimension of observed state space (e.g. agent position in joint space)
            if OBS_STATE in batch:
                for dim_idx, val in enumerate(batch[OBS_STATE][i]):
                    rr.log(f"state/{dim_idx}", rr.Scalars(val.item()))

            if DONE in batch:
                rr.log(DONE, rr.Scalars(batch[DONE][i].item()))

            if REWARD in batch:
                rr.log(REWARD, rr.Scalars(batch[REWARD][i].item()))

            if "next.success" in batch:
                rr.log("next.success", rr.Scalars(batch["next.success"][i].item()))

    if mode == "local" and save:
        # save .rrd locally
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        repo_id_str = repo_id.replace("/", "_")
        rrd_path = output_dir / f"{repo_id_str}_episode_{episode_index}.rrd"
        rr.save(rrd_path)
        logging.info(f"Saved visualization to {rrd_path}")
        return rrd_path

    elif mode == "distant":
        # stop the process from exiting since it is serving the websocket connection
        logging.info(f"Serving on ws://localhost:{ws_port} and http://localhost:{web_port}")
        logging.info("Connect with: rerun ws://localhost:{ws_port}")
        logging.info("Press Ctrl+C to exit")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nCtrl-C received. Exiting.")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Visualize LeRobot dataset episodes (fixed version that works with any episode)"
    )

    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Dataset repository ID (e.g. 'gomi-kuzu/lekiwi_pick_place' or 'lerobot/pusht').",
    )
    parser.add_argument(
        "--episode-index",
        type=int,
        required=True,
        help="Episode index to visualize (0-based).",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help=(
            "Root directory for locally stored dataset (e.g. '~/lerobot_datasets'). "
            "If not provided, will try to load from HuggingFace cache or download from hub."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory path to write .rrd file when `--save 1` is set.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=32,
        help="Batch size for DataLoader (default: 32).",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=4,
        help="Number of DataLoader worker processes (default: 4).",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="local",
        choices=["local", "distant"],
        help=(
            "'local': Spawn a local viewer. "
            "'distant': Create a server for remote viewing (connect with 'rerun ws://localhost:PORT')."
        ),
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=9090,
        help="Web port for rerun.io when '--mode distant' is set (default: 9090).",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=9087,
        help="WebSocket port for rerun.io when '--mode distant' is set (default: 9087).",
    )
    parser.add_argument(
        "--save",
        type=int,
        default=0,
        help=(
            "Save .rrd file to '--output-dir' (1=yes, 0=no). "
            "When saving, local viewer is not spawned. "
            "View later with: rerun path/to/file.rrd"
        ),
    )
    parser.add_argument(
        "--tolerance-s",
        type=float,
        default=1e-4,
        help=(
            "Tolerance in seconds for timestamp validation against dataset FPS. "
            "Passed to LeRobotDataset constructor (default: 1e-4)."
        ),
    )

    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] %(message)s'
    )

    # Load dataset with episode filtering
    # This is the KEY FIX: passing episodes=[episode_index] filters the dataset
    # so it only contains frames from that specific episode
    logging.info(f"Loading dataset '{args.repo_id}' episode {args.episode_index}")
    if args.root:
        logging.info(f"Loading from local path: {args.root}")
    
    # For local datasets, use full path to avoid Hub access
    dataset_root = args.root
    if dataset_root:
        dataset_root = Path(dataset_root).expanduser() / args.repo_id
    
    dataset = LeRobotDataset(
        args.repo_id,
        episodes=[args.episode_index],  # Filter to specific episode
        root=str(dataset_root) if dataset_root else None,
        revision='v3.0',  # Use fixed version to avoid Hub version check
        tolerance_s=args.tolerance_s
    )
    
    logging.info(f"Loaded episode {args.episode_index}: {len(dataset)} frames")

    # Visualize
    visualize_dataset(
        dataset=dataset,
        episode_index=args.episode_index,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        mode=args.mode,
        web_port=args.web_port,
        ws_port=args.ws_port,
        save=bool(args.save),
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
