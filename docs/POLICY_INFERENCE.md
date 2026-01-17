# LeKiwi Policy Inference Node

このドキュメントでは、学習済みモデルを使用してLeKiwiロボットを自律制御するための推論ノードの使用方法を説明します。

## 概要

`lekiwi_policy_node`は、LeRobotで学習された方策モデル（policy）を使用してロボットを自律制御するためのROS2ノードです。

**重要**: このノードはGPUを搭載したデスクトップPCで実行し、ROS2トピック経由でラズベリーパイ上の`lekiwi_teleop_node`と通信します。直接ロボットに接続する必要はありません。

## システム構成

```
┌─────────────────────┐        ROS2 Topics         ┌──────────────────────┐
│  Desktop PC (GPU)   │ ◄────────────────────────► │  Raspberry Pi        │
│                     │                             │  (on LeKiwi robot)   │
│ lekiwi_policy_node  │   Observations (Subscribe)  │                      │
│  - Policy inference │ ◄─────────────────────────  │ lekiwi_teleop_node   │
│  - Action prediction│   /lekiwi/joint_states      │  - Robot control     │
│                     │   /lekiwi/camera/front      │  - Sensor reading    │
│                     │   /lekiwi/camera/wrist      │                      │
│                     │                             │                      │
│                     │   Commands (Publish)        │                      │
│                     │  ─────────────────────────► │                      │
│                     │   /lekiwi/cmd_vel           │                      │
│                     │   /lekiwi/arm_joint_cmds    │                      │
└─────────────────────┘                             └──────────────────────┘
```

## 主な機能

- 学習済み方策モデルの読み込み
- ロボットの観測データ取得
- 方策による行動予測
- ロボットへの行動指令送信
- サービスによる開始/停止制御

## 前提条件

### デスクトップPC側
1. LeRobotがインストールされている
2. LEROBOT_PATH環境変数が設定されている
3. 学習済みモデルが準備されている
4. データセットがローカルに存在している
5. ROS2がインストールされている
6. (オプション) CUDAが利用可能
ステップ1: ラズベリーパイでロボット制御ノードを起動

まず、ラズベリーパイ上で`lekiwi_teleop_node`を起動します：

```bash
# ラズベリーパイ上で実行
ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py
```

または直接ノードを起動：

```bash
# ラズベリーパイ上で実行
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node
```

### ステップ2: デスクトップPCで推論ノードを起動

次に、デスクトップPC上で推論ノードを起動します：

```bash
# デスクトップPC上で実行
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=/path/to/trained/model \
    dataset_repo_id:=username/dataset_name \
    single_task:="Pick and place the cube"
```

### ステップ3: 推論を開始

サービスコールで推論を開始：

```bash
ros2 service call /lekiwi/policy/start std_srvs/srv/Trigger
```

## 使用方法

### パラメータ詳細

#### 必須パラメータ

- `policy_path`: 学習済みモデルのディレクトリパス
  - 例: `~/outputs/train/2025-01-15/14-30-00/checkpoints/last`
- `dataset_repo_id`: 学習に使用したデータセットのリポジトリID
  - 例: `gomi-kuzu/lekiwi_pick_place`

#### オプションパラメータ

- `control_frequency`: 制御ループの周波数（デフォルト: `50.0` Hz）
- `device`: 推論デバイス（デフォルト: `cuda`、他に `cpu`, `mps`）
- `use_amp`: 自動混合精度を使用（デフォルト: `false`）
- `use_degrees`: 角度を度数法で扱う（デフォルト: `false`）
- `rotate_front_camera`: フロントカメラを180度回転（デフォルト: `true`）

### サービスでの制御

ノード起動後、以下のサービスで推論の開始/停止を制御できます：

```bash
# 推論開始
ros2 service call /lekiwi/policy/start std_srvs/srv/Trigger

# 推論停止
ros2 service call /lekiwi/policy/stop std_srvs/srv/Trigger
```

### 実行例

#### 例1: 基本的な推論（分散システム）

```bash
# ラズベリーパイ上
ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py

# デスクトップPC上
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=~/outputs/train/2025-01-15/14-30-00/checkpoints/last \
    dataset_repo_id:=gomi-kuzu/lekiwi_pick_place \
    single_task:="Pick the blue cube and place it in the box"

# 推論開始
ros2 service call /lekiwi/policy/start std_srvs/srv/Trigger
```

#### 例2: CPUで推論

```bash
# デスクトップPC上
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=~/outputs/train/2025-01-15/14-30-00/checkpoints/last \
    dataset_repo_id:=gomi-kuzu/lekiwi_pick_place \
    device:=cpu
```

#### 例3: 異なるネットワーク上のマシン間通信

```bash
# ラズベリーパイのIPを確認
# 例: 192.168.1.100

# デスクトップPC上で環境変数を設定
export ROS_DOMAIN_ID=42  # 同じドメインIDを使用
export ROS_LOCALHOST_ONLY=0

# 通常通り起動
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=~/outputs/train/2025-01-15/14-30-00/checkpoints/last \
    dataset_repo_id:=gomi-kuzu/lekiwi_pick_place
```

## パブリッシュされるトピック（デスクトップPC → ラズベリーパイ）

- `/lekiwi/cmd_vel` (geometry_msgs/Twist): ベース速度コマンド
- `/lekiwi/arm_joint_commands` (sensor_msgs/JointState): アーム関節位置コマンド

## サブスクライブするトピック（ラズベリーパイ → デスクトップPC）

- `/lekiwi/joint_states` (sensor_msgs/JointState): 現在のロボット状態
- `/lekiwi/camera/front/image_raw/compressed` (sensor_msgs/CompressedImage): フロントカメラ画像
- `/lekiwi/camera/wrist/image_raw/compressed` (sensor_msgs/CompressedImage): リストカメラ画像

## サービス

- `/lekiwi/policy/start` (std_srvs/Trigger): 推論開始
- `/lekiwi/policy/stop` (std_srvs/Trigger): 推論停止

## トラブルシューティング

### 観測データが受信できない

- ラズベリーパイ上で`lekiwi_teleop_node`が起動しているか確認
- トピックが正しくパブリッシュされているか確認:
  ```bash
  ros2 topic list
  ros2 topic hz /lekiwi/joint_states
  ros2 topic hz /lekiwi/camera/front/image_raw/compressed
  ```
- ROS_DOMAIN_IDが両方のマシンで一致しているか確認
- ネットワーク接続を確認（異なるマシン間の場合）

### ロボットに接続できない

**注意**: 推論ノードはロボットに直接接続しません。ラズベリーパイ側の問題です。

- `lekiwi_teleop_node`がラズベリーパイで起動しているか確認
- ロボットの電源が入っているか確認
- ポート設定が正しいか確認（ラズベリーパイ側）

## アーキテクチャの詳細

このノードは分散システム向けに設計されています：

1. **観測取得**: ROS2トピックから観測データを受信（`/lekiwi/joint_states`, カメラ画像）
2. **データ変換**: `build_dataset_frame()`で観測をデータセット形式に変換
3. **行動予測**: `predict_action()`で方策モデルから行動を予測
4. **行動送信**: ROS2トピックで行動指令を送信（直接ロボットには送信しない）

### アーキテクチャの利点

- **GPU活用**: デスクトップPCのGPUで高速推論
- **リアルタイム性**: ロボット制御は軽量なラズベリーパイで実行
- **柔軟性**: 推論ノードとロボットを独立して開発・テスト可能
- **拡張性**: 複数の推論ノードや可視化ノードを簡単に追加可能

## 実装の詳細

このノードは`lerobot-record`コマンドの`--policy.path`オプション機能を参考に実装されています：

1. **観測取得**: ROS2トピックからロボットの状態とカメラ画像を取得
2. **データ変換**: `build_dataset_frame()`で観測をデータセット形式に変換
3. **行動予測**: `predict_action()`で方策モデルから行動を予測
4. **行動送信**: ROS2トピックでロボットに行動を送信

## 参考

- 元のスクリプト: `~/study/lerobot/src/lerobot/scripts/lerobot_record.py`
- データ記録ノード: [lekiwi_data_recorder.py](../lekiwi_ros2_teleop/lekiwi_data_recorder.py)
- テレオペノード: [lekiwi_teleop_node.py](../lekiwi_ros2_teleop/lekiwi_teleop_node.py)

