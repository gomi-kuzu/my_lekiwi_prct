# LeKiwi ROS2 Teleoperation

ROS2 を使用した LeKiwi ロボットのテレオペレーション・推論・データ収集を行うパッケージ。

## パッケージ概要

**LeRobot ライブラリ**（ハードウェア抽象化・ML 推論）と **ROS2**（通信・システム統合）を
組み合わせ、両者をつなぐブリッジノード群で構成する。実機制御・自律推論・模倣学習用データ収集を
ノード単位で差し替えられるのが特徴。

| 実行名 | 種別 | 役割 |
|---|---|---|
| `lekiwi_teleop_node` | ノード | LeKiwi 実機を制御（コマンド受信・状態/カメラ配信） |
| `lekiwi_policy_node` | ノード | 学習済みポリシーで自律制御（GPU PC 上で実行） |
| `lekiwi_ros2_teleop_client` | ノード | SO-101 Leader Arm / キーボード入力を送信 |
| `lekiwi_bag_recorder` | ノード | テレオペを MCAP bag で記録（データパイプライン Stage 1） |
| `lekiwi_rosbag_to_lerobot` | CLI | bag を LeRobot Dataset v3 に変換（Stage 2） |
| `lekiwi_data_recorder` | ノード | LeRobot v3 へ直接記録する旧経路（非推奨） |
| `lekiwi_upload_dataset` | CLI | データセットを Hugging Face Hub にアップロード |

各ノードのトピック・パラメータの詳細は [docs/nodes.md](docs/nodes.md) を参照すること。

## アーキテクチャ

**LeRobot × ROS2 の役割分担:**

| 層 | 担当 | 役割 |
|---|---|---|
| **LeRobot** | ハードウェア抽象化・ML 推論 | モーター制御、データセット、ポリシー推論 |
| **ROS2** | 通信・システム統合 | ノード間通信、分散実行、モニタリング |
| **ブリッジノード** | 変換 | LeRobot の dict 形式 ↔ ROS2 メッセージ型 |

LeRobot は ROS2 非依存のまま保たれ、ブリッジ層が変換を担うため、`ros2 topic echo` /
`ros2 bag` などの ROS2 ツールでそのまま監視・記録・再生できる。

**パターン1: ダイレクトテレオペレーション**

```
┌────────────────────┐
│ SO-101 Leader Arm  │
│ Joy Controller     │
└─────────┬──────────┘
          │ input
          ↓
┌─────────────────────────┐
│ lekiwi_ros2_teleop_     │
│       client            │
└──────────┬──────────────┘
           │ publishes (ROS2)
           ↓
   ┌───────────────────┐
   │ /lekiwi/cmd_vel   │
   │ /lekiwi/arm_joint_│
   │  commands         │
   └────────┬──────────┘
            │ subscribes
            ↓
┌─────────────────────┐
│  lekiwi_teleop_node │ → LeKiwi Robot
└─────────────────────┘
```

**パターン2: ポリシー推論（分散実行）**

```
┌──────────────────────┐   observations (ROS2)   ┌─────────────────────┐
│  Desktop PC (GPU)    │ ◄─────────────────────  │  LeKiwi (RasPi)     │
│  lekiwi_policy_node  │   joint_states / images │  lekiwi_teleop_node │
│   - policy inference │ ──────────────────────► │   - robot control   │
└──────────────────────┘   commands (ROS2)       └─────────────────────┘
```

詳細は [docs/policy_inference.md](docs/policy_inference.md) を参照すること。

## 検証済み環境・依存関係
- OS Ubuntu24
- ROS2 Jazzy
- Python 3.12
- rclpy / geometry_msgs / sensor_msgs / cv_bridge
- opencv-python (cv2) / numpy / torch
- [LeRobot](https://github.com/huggingface/lerobot) **v0.6.2 以上**（LeKiwi ハードウェア抽象化・ポリシー推論）


## クイックスタート

### セットアップ
- [docs/setup.md](docs/setup.md) を参照

### ビルド

```bash
cd [ROS2 のワークスペース]
colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash
```

> 以降、各ターミナルで作業する前に `source install/setup.bash` を実行すること。

### テレオペレーション

**ターミナル1（LeKiwi 実機上）: 制御ノードを起動**

```bash
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args -p robot_port:=/dev/ttyACM0
```

**ターミナル2（PC 上）: テレオペクライアントを起動**

```bash
export LEROBOT_PATH="$HOME/lerobot/src"
export LEKIWI_REMOTE_IP="<LEKIWI_IP>"   # 例: LEKIWI_REMOTE_IP="172.18.11.100"

ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
  --ros-args \
  -p leader_arm_port:=/dev/ttyACM0 \
  -p use_keyboard:=false
```

**ターミナル3（PC 上）: ベースをキーボードで操作**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/lekiwi/cmd_vel
```

SO-101 Leader Arm でアームを、`teleop_twist_keyboard` でベースを操作できる。
ジョイスティックで操作する場合は `teleop_twist_keyboard` の代わりに次を起動する:

```bash
ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py
```

### 動作確認

```bash
# ベース速度コマンドの監視 / 配信頻度の確認
ros2 topic echo /lekiwi/cmd_vel
ros2 topic hz /lekiwi/cmd_vel

# 関節状態のモニタリング
ros2 topic echo /lekiwi/joint_states

# カメラ画像の表示
ros2 run image_view image_view --ros-args \
  -r image:=/lekiwi/camera/front/image_raw -p image_transport:=compressed
```

## ドキュメント

| ドキュメント | 内容 |
|---|---|
| [docs/setup.md](docs/setup.md) | インストール・カメラ設定・環境変数 |
| [docs/nodes.md](docs/nodes.md) | ノード・トピック・パラメータのリファレンス |
| [docs/data_pipeline.md](docs/data_pipeline.md) | データ収集 → 変換パイプライン（**推奨**） |
| [docs/policy_inference.md](docs/policy_inference.md) | ポリシー推論ノードの使い方 |
| [docs/custom_joy_setup.md](docs/custom_joy_setup.md) | ジョイスティック設定 |
| [docs/direct_recording.md](docs/direct_recording.md) | LeRobot へ直接記録する旧経路（非推奨） |
| [docs/train_guide.md](docs/train_guide.md)| 転移学習方法の参考文書 |

## ライセンス

Apache-2.0
