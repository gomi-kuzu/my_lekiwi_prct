# LeKiwi ROS2 Teleoperation

ROS2を使用したLeKiwiロボットのテレオペレーションとVLA推論ノード

## 概要

このパッケージには3つのノードが含まれています:

1. **lekiwi_teleop_node**: `lekiwi_host.py`を参考に作成されており、ZMQ通信の代わりにROS2トピック通信を使用してLeKiwiロボットを制御します。

2. **lekiwi_vla_node**: LeRobotのVLA (Vision-Language-Action) モデルを使用して、カメラ画像とロボット状態から次のアクションを自動生成します。

3. **lekiwi_ros2_teleop_client**: SO100 Leader ArmとKeyboardからのテレオペレーション入力を受け取り、ROS2トピック経由でlekiwi_teleop_nodeを制御します。

## アーキテクチャ

### パターン1: ダイレクトテレオペレーション（lekiwi_ros2_teleop_client）

```
┌────────────────────┐
│ SO100 Leader Arm   │
│ Keyboard           │
└─────────┬──────────┘
          │ input
          ↓
┌─────────────────────────┐
│ lekiwi_ros2_teleop_     │
│       client            │
└──────────┬──────────────┘
           │ publishes (ROS2)
           ↓
    ┌──────────────┐
    │ /cmd_vel     │
    │ /arm_joint_  │
    │  commands    │
    └──────┬───────┘
           │ subscribes
           ↓
┌─────────────────────┐
│  lekiwi_teleop_node │ → LeKiwi Robot
└─────────────────────┘
```

### パターン2: VLA自律制御

```
┌─────────────────────┐
│  lekiwi_teleop_node │ ← ロボット直接制御
└──────────┬──────────┘
           │ publishes
           ↓
    ┌──────────────┐
    │ /joint_states│
    │ /camera/...  │
    └──────┬───────┘
           │ subscribes
           ↓
  ┌────────────────┐
  │ lekiwi_vla_node│ ← VLA推論
  └────────┬───────┘
           │ publishes
           ↓
    ┌──────────────┐
    │ /cmd_vel     │
    │ /arm_joint_  │
    │  commands    │
    └──────┬───────┘
           │ subscribes
           ↓
┌─────────────────────┐
│  lekiwi_teleop_node │ → ロボットへ送信
└─────────────────────┘
```

## 機能

### lekiwi_teleop_node

- **トピック通信**: ROS2トピックを使用したロボット制御
- **アーム制御**: 6自由度アームの位置制御
- **ベース制御**: オムニホイールベースの速度制御
- **カメラ配信**: フロントとリストカメラの画像配信
- **ウォッチドッグ**: コマンドが一定時間受信されない場合、ベースを自動停止

### lekiwi_vla_node

- **VLA推論**: LeRobotのVLAモデルを使用した自律動作生成
- **マルチモーダル入力**: カメラ画像、関節状態、言語命令を統合
- **時間的アンサンブル**: 滑らかな動作のための時間的フィルタリング
- **リアルタイム制御**: 設定可能な推論周波数で動作

### lekiwi_ros2_teleop_client

- **テレオペレーション**: SO100 Leader ArmとKeyboardからの入力取得
- **ROS2パブリッシュ**: lekiwi_teleop_nodeへコマンド送信
- **環境変数対応**: `LEKIWI_REMOTE_IP`でLeKiwiのIPアドレスを指定
- **可視化**: Rerun統合によるリアルタイムモニタリング（オプション）

## トピック

### Subscribe (受信)

- `/lekiwi/cmd_vel` (geometry_msgs/Twist): ベース速度コマンド
  - `linear.x`: X方向速度 (m/s)
  - `linear.y`: Y方向速度 (m/s)
  - `angular.z`: 回転速度 (rad/s)

- `/lekiwi/arm_joint_commands` (sensor_msgs/JointState): アーム関節位置コマンド
  - 6つの関節位置が必要です

### Publish (配信)

- `/lekiwi/joint_states` (sensor_msgs/JointState): 現在のロボット状態
  - `position`: アーム関節位置
  - `velocity`: ベース速度

- `/lekiwi/camera/front/image_raw/compressed` (sensor_msgs/CompressedImage): フロントカメラ画像

- `/lekiwi/camera/wrist/image_raw/compressed` (sensor_msgs/CompressedImage): リストカメラ画像

## パラメータ

### lekiwi_teleop_node

- `robot_port` (string, default: "/dev/ttyACM0"): ロボットのシリアルポート
- `control_frequency` (float, default: 30.0): 制御ループ周波数 (Hz)
- `watchdog_timeout_ms` (int, default: 500): ウォッチドッグタイムアウト (ms)
- `use_degrees` (bool, default: false): 角度単位を度で使用するか
- `rotate_front_camera` (bool, default: true): フロントカメラを180度回転するか（カメラが逆さまに取り付けられている場合）

### lekiwi_vla_node

- `policy_path` (string, required): VLAモデルのチェックポイントパス
- `inference_frequency` (float, default: 10.0): 推論ループ周波数 (Hz)
- `task_instruction` (string, default: "Pick up the object"): タスクの言語命令
- `use_cuda` (bool, default: true): CUDAを使用するか
- `temporal_ensemble_coeff` (float, default: 0.01): 時間的アンサンブルの係数 (0.0-1.0)

### lekiwi_ros2_teleop_client

- `lekiwi_remote_ip` (string, default: ""): LeKiwiのIPアドレス（空の場合は環境変数`LEKIWI_REMOTE_IP`を使用）
- `lekiwi_id` (string, default: "my_lekiwi"): LeKiwiのID
- `leader_arm_port` (string, default: "/dev/tty.usbmodem585A0077581"): SO100 Leader Armのシリアルポート
- `leader_arm_id` (string, default: "my_leader_arm"): Leader ArmのID
- `keyboard_id` (string, default: "my_keyboard"): KeyboardのID
- `control_frequency` (float, default: 30.0): 制御ループ周波数 (Hz)
- `use_rerun` (bool, default: false): Rerun可視化を使用するか

## セットアップ

### 1. 環境変数の設定（必須）

#### LEROBOT_PATH（必須）

両方のノードで必要です。LeRobotのパスを環境変数として設定します。
お使いのシェルの設定ファイルを編集してください：

**bashの場合 (`~/.bashrc`):**
```bash
# LeRobotの設定（あなたのパスに置き換えてください）
export LEROBOT_PATH="/path/to/your/lerobot/src"
```

**zshの場合 (`~/.zshrc`):**
```bash
# LeRobotの設定（あなたのパスに置き換えてください）
export LEROBOT_PATH="/path/to/your/lerobot/src"
```

**例:**
```bash
# LeRobotを ~/study/lerobot にインストールした場合
export LEROBOT_PATH="$HOME/study/lerobot/src"

# 別の場所にインストールした場合
export LEROBOT_PATH="/opt/lerobot/src"
```

#### LEKIWI_REMOTE_IP（lekiwi_ros2_teleop_clientで必須）

テレオペレーションクライアントを使用する場合、LeKiwiのIPアドレスを設定します：

```bash
# LeKiwiのIPアドレスを設定
export LEKIWI_REMOTE_IP="172.18.134.136"
```

設定を反映：
```bash
source ~/.bashrc  # または source ~/.zshrc
```

### 2. 環境変数の確認

正しく設定されているか確認します：
```bash
echo $LEROBOT_PATH
# 出力例: /home/your_username/study/lerobot/src

echo $LEKIWI_REMOTE_IP
# 出力例: 172.18.134.136
```

**注意**: 環境変数が設定されていない場合、ノードの起動時にエラーが発生します。

**Micromamba環境での注意点**:
- `micromamba run -n leros` コマンドは新しいシェルを起動するため、`.bashrc`が自動的に読み込まれます
- 環境変数 `LEROBOT_PATH` は micromamba 環境内でも有効です
- 既にアクティベートされた環境内で作業する場合は、以下のようにシェルを再起動してください：
  ```bash
  exec bash  # または exec zsh
  ```

## ビルド

### 環境変数の設定（推奨）

VLAノードを使用する前に、LeRobotのパスを環境変数として設定することを推奨します。
`~/.bashrc`または`~/.zshrc`に以下を追加してください：

```bash
# LeRobotのパスを設定（あなたの環境に合わせて変更してください）
export LEROBOT_PATH="/path/to/your/lerobot/src"
```

設定後、以下のコマンドで反映させます：

```bash
source ~/.bashrc  # bashの場合
# または
source ~/.zshrc   # zshの場合
```

**重要**: `LEROBOT_PATH`環境変数が設定されていないと、VLAノードは起動できません。

### ビルド手順

```bash
cd ~/ros2_ws
micromamba run -n leros colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash
```

## 実行

### 1. テレオペレーション（Leader Arm + Keyboard）

#### ターミナル1: LeKiwi上でlekiwi_teleop_nodeを起動

```bash
# LeKiwiロボット上で実行
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args -p robot_port:=/dev/ttyACM0
```

#### ターミナル2: PC上でlekiwi_ros2_teleop_clientを起動

```bash
# 環境変数を設定（.bashrcに書いていない場合）
export LEROBOT_PATH="$HOME/study/lerobot/src"
export LEKIWI_REMOTE_IP="172.18.134.136"

# テレオペレーションクライアントを起動
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client

# パラメータ指定の例
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
  --ros-args \
  -p leader_arm_port:=/dev/ttyUSB0 \
  -p control_frequency:=30.0 \
  -p use_rerun:=true
```

これで、SO100 Leader Armでロボットアームを、Keyboardでベースを操作できます。

### 2. VLA自律制御

#### ターミナル1: Teleopノード（ロボット制御）

#### ターミナル1: Teleopノード（ロボット制御）

```bash
# デフォルトで実行
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_teleop_node

# パラメータを指定して実行
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args \
  -p robot_port:=/dev/ttyACM0 \
  -p control_frequency:=30.0 \
  -p rotate_front_camera:=true

# フロントカメラの回転を無効化する場合
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args -p rotate_front_camera:=false
```

#### ターミナル2: VLAノード（自律制御）

#### ターミナル2: VLAノード（自律制御）

```bash
# VLAノードを実行（policy_pathは必須）
# HuggingFace Hubからモデルを使用する場合
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_vla_node \
  --ros-args \
  -p policy_path:=lerobot/smolvla_base \
  -p task_instruction:="Pick up the red cup" \
  -p inference_frequency:=10.0

# ローカルのモデルを使用する場合
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_vla_node \
  --ros-args \
  -p policy_path:=/path/to/your/local/policy \
  -p task_instruction:="Pick up the red cup" \
  -p inference_frequency:=10.0
```

### 3. キーボードでベース制御（手動）

```bash
# teleop_twist_keyboardパッケージを使用
micromamba run -n leros ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/lekiwi/cmd_vel
```

### 4. カメラ画像の表示

```bash
# rqtで画像表示
micromamba run -n leros rqt_image_view
# トピック: /lekiwi/camera/front/image_raw/compressed
```

### 5. 関節状態のモニタリング

```bash
micromamba run -n leros ros2 topic echo /lekiwi/joint_states
```

## 利用可能なVLAモデル

### VLA (Vision-Language-Action) モデル

#### 1. SmolVLA（推奨）
- **モデルID**: `lerobot/smolvla_base`
- **特徴**: 効率的でコスト効率の良いVision-Language-Actionモデル
- **適用**: リアルタイム制御に最適（軽量で高速）
- **論文**: [SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics](https://arxiv.org/abs/2506.01844)
- **入力**: カメラ画像 + ロボット状態 + 言語命令

```bash
# SmolVLAの使用例
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_vla_node \
  --ros-args \
  -p policy_path:=lerobot/smolvla_base \
  -p task_instruction:="Pick up the object"
```

#### 2. X-VLA
- **モデルID例**:
  - `lerobot/xvla-base` - 基本モデル
  - `lerobot/xvla-folding` - 布折りタスク特化
  - `lerobot/xvla-libero` - LIBERO環境用
  - `lerobot/xvla-google-robot` - Google Robotデータセット用
  - `lerobot/xvla-agibot-world` - Agibot環境用
- **特徴**: クロスエンボディメント対応（約0.9Bパラメータ）
- **適用**: 複数のロボットプラットフォームでの汎用的なタスク実行

```bash
# X-VLAの使用例
micromamba run -n leros ros2 run lekiwi_ros2_teleop lekiwi_vla_node \
  --ros-args \
  -p policy_path:=lerobot/xvla-base \
  -p task_instruction:="Grasp and move the cup"
```

### その他のLeRobotポリシーモデル

以下のモデルもVLAノードで使用可能です（言語命令なし）:

- **ACT** (Action Chunking with Transformers)
- **Diffusion Policy**
- **PI0** & **PI05** (Physical Intelligence models)
- **Groot**
- **VQBeT**
- **TDMPC**

### モデルのダウンロード

モデルは初回実行時に自動的にHugging Face Hubからダウンロードされます。
ローカルにダウンロード済みのモデルを使用する場合は、そのパスを`policy_path`に指定してください。

```bash
# HuggingFace Hubから自動ダウンロード
-p policy_path:=lerobot/smolvla_base

# ローカルモデルを使用
-p policy_path:=/path/to/local/model/checkpoint
```

## 依存関係

- ROS2 Jazzy
- rclpy
- geometry_msgs
- sensor_msgs
- cv_bridge
- opencv-python (cv2)
- numpy
- torch
- lerobot (LeKiwiロボットライブラリとVLAモデル)

### 環境変数

- `LEROBOT_PATH` (**必須**): LeRobotライブラリのsrcディレクトリへのパス
  - 例: `export LEROBOT_PATH="$HOME/study/lerobot/src"`
  - 例: `export LEROBOT_PATH="/opt/lerobot/src"`
  
- `LEKIWI_REMOTE_IP` (lekiwi_ros2_teleop_clientで必須): LeKiwiロボットのIPアドレス
  - 例: `export LEKIWI_REMOTE_IP="172.18.134.136"`
  
**注意**: 環境変数が設定されていないと、各ノードは起動時にエラーを出力します。

## 参考リンク

- **LeRobot GitHub**: https://github.com/huggingface/lerobot
- **LeRobot HuggingFace**: https://huggingface.co/lerobot
- **SmolVLA論文**: https://arxiv.org/abs/2506.01844
- **LeRobot ドキュメント**: https://huggingface.co/docs/lerobot
- **LeRobot Discord**: https://discord.gg/s3KuuzsPFb

## ライセンス

Apache-2.0
