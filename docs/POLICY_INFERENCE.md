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
  - 正規化統計などのメタデータ取得に使用します。

> **重要（データセットのパス解決）**: データセットは `{dataset_root}/{dataset_repo_id}`
> の場所から読み込まれます（HuggingFace標準レイアウト）。
> 例えば `dataset_root:=/home/inoma/jazzy_ws/output` かつ
> `dataset_repo_id:=gomi-kuzu/lekiwi_dance_and_bow` の場合、
> `/home/inoma/jazzy_ws/output/gomi-kuzu/lekiwi_dance_and_bow` が読み込まれます。
>
> - 上記が見つからない場合、`dataset_root` 自体をデータセットとして読み込みます（警告を表示）。
> - どちらにも `meta/info.json` が無い場合は、無関係なデータセットを黙って読み込まず、
>   明確なエラーで停止します。
> - 起動ログに実際に読み込んだ**絶対パスとフレーム数**が出力されるので、
>   意図したデータセットか必ず確認してください。

#### オプションパラメータ

- `dataset_root`: データセットの親ディレクトリ（デフォルト: `~/lerobot_datasets`）
  - 実際のデータセットは `{dataset_root}/{dataset_repo_id}` から読み込まれます。
- `control_frequency`: 制御ループの周波数（デフォルト: `30.0` Hz)
- `device`: 推論デバイス（デフォルト: `cuda`、他に `cpu`, `mps`）
- `use_amp`: 自動混合精度を使用（デフォルト: `false`）
- `n_action_steps`: 1回の推論で消費するアクション数（デフォルト: `0` = 学習時の値を使用）
  - ACTのアクションチャンキングの挙動を推論時に調整するパラメータ。**再学習は不要**。
  - 小さくすると、より頻繁に新しい観測で再推論するため反応性が向上（クローズドループに近づく）。
  - 制約: `1 <= n_action_steps <= chunk_size`（例: 学習時 `chunk_size=100`）。
- `temporal_ensemble_coeff`: ACTのTemporal Ensemblingを有効化（デフォルト: `-1.0` = 学習時の値を使用）
  - `0` 以上を指定すると有効化（原論文の推奨値は `0.01`）。**再学習は不要**。
  - 有効化すると `n_action_steps` は自動的に `1` に強制される（LeRobotの制約）。
- `clamp_zero_std_state`: 学習時に分散がほぼ0だった観測次元を学習平均に固定（デフォルト: `true`）
  - データ収集中ずっと一定だった次元（例: 動かさなかったグリッパー）は、学習統計の
    標準偏差が極小（例: `~4e-7`）になります。この次元に実機のわずかなズレが入ると、
    MEAN_STD正規化で `(値 - 平均) / 極小std` が巨大化し、推論が破綻します。
  - 本機能は該当次元を学習平均に固定することでこれを防ぎます。多様な値をとる次元
    （std が閾値以上）はそのまま通常どおり推論されるため、副作用はありません。
  - 無効化するには `clamp_zero_std_state:=false` を指定します。
- `zero_std_threshold`: 上記で「分散がほぼ0」とみなす標準偏差のしきい値（デフォルト: `1e-4`）
  - 正規化器は一定信号でも厳密な0ではなく極小のstdを保存するため、`== 0` ではなく
    しきい値で判定します。実際に変動する最小次元のstd（例: 台車速度 ~0.02）より
    十分小さい値に設定してください。

> **補足**: 学習済みモデルは `n_action_steps=100`, `chunk_size=100`, `temporal_ensemble_coeff=null`
> の場合、1回の推論で得た100ステップを約3.3秒（30Hz時）かけてオープンループ実行してから
> 次の推論を行います。反応性を高めたい場合は `n_action_steps` を小さくするか、
> `temporal_ensemble_coeff` を有効化してください（いずれも再学習不要）。

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

