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
Upload LeRobot dataset to Hugging Face Hub

This script uploads a locally recorded dataset to the Hugging Face Hub.

Example usage:
    python upload_dataset.py \
        --dataset_repo_id username/dataset_name \
        --dataset_root ~/lerobot_datasets \
        --private \
        --tags robot lekiwi teleoperation
"""

import os
import sys
import argparse
from pathlib import Path

# Import LeRobot components
lerobot_path = os.environ.get('LEROBOT_PATH')
if lerobot_path is None:
    raise EnvironmentError(
        "LEROBOT_PATH environment variable is not set. "
        "Please set it to the path of your LeRobot installation's src directory. "
        "Example: export LEROBOT_PATH='/path/to/lerobot/src'"
    )
if lerobot_path not in sys.path:
    sys.path.append(lerobot_path)

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def main():
    parser = argparse.ArgumentParser(
        description='Upload LeRobot dataset to Hugging Face Hub'
    )
    
    parser.add_argument(
        '--dataset_repo_id',
        type=str,
        required=True,
        help='Dataset repository ID (e.g., username/dataset_name)'
    )
    
    parser.add_argument(
        '--dataset_root',
        type=str,
        default=str(Path.home() / 'lerobot_datasets'),
        help='Root directory where dataset is stored (default: ~/lerobot_datasets)'
    )
    
    parser.add_argument(
        '--private',
        action='store_true',
        help='Upload to a private repository'
    )
    
    parser.add_argument(
        '--tags',
        nargs='*',
        default=None,
        help='Tags to add to the dataset on the Hub (e.g., robot lekiwi teleoperation)'
    )
    
    parser.add_argument(
        '--revision',
        type=str,
        default='main',
        help='Git revision to push to (default: main)'
    )
    
    args = parser.parse_args()
    
    # Load the dataset
    print(f"Loading dataset from: {args.dataset_root}/{args.dataset_repo_id}")
    try:
        dataset = LeRobotDataset(
            repo_id=args.dataset_repo_id,
            root=args.dataset_root,
        )
    except Exception as e:
        print(f"Error loading dataset: {e}")
        print("\nMake sure:")
        print(f"  1. The dataset exists at {args.dataset_root}/{args.dataset_repo_id}")
        print(f"  2. The dataset_repo_id matches the one used during recording")
        return 1
    
    # Display dataset info
    print(f"\nDataset Information:")
    print(f"  Repository ID: {dataset.repo_id}")
    print(f"  Number of episodes: {dataset.num_episodes}")
    print(f"  Total frames: {dataset.num_frames}")
    print(f"  FPS: {dataset.fps}")
    print(f"  Robot type: {dataset.meta.robot_type}")
    
    # Confirm upload
    print(f"\nUploading to Hugging Face Hub:")
    print(f"  Repository: {args.dataset_repo_id}")
    print(f"  Private: {args.private}")
    print(f"  Tags: {args.tags}")
    print(f"  Revision: {args.revision}")
    
    response = input("\nProceed with upload? [y/N]: ")
    if response.lower() not in ['y', 'yes']:
        print("Upload cancelled.")
        return 0
    
    # Upload to Hub
    print("\nUploading dataset to Hugging Face Hub...")
    print("This may take a while depending on dataset size...")
    
    try:
        dataset.push_to_hub(
            tags=args.tags,
            private=args.private,
            revision=args.revision,
        )
        print(f"\n✓ Dataset successfully uploaded to: https://huggingface.co/datasets/{args.dataset_repo_id}")
        return 0
    except Exception as e:
        print(f"\n✗ Error uploading dataset: {e}")
        print("\nMake sure:")
        print("  1. You are logged in to Hugging Face CLI (run 'huggingface-cli login')")
        print("  2. You have write permissions for the repository")
        print("  3. Your internet connection is stable")
        return 1


if __name__ == '__main__':
    sys.exit(main())
