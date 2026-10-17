# LeKiwi ROS2 Data Recording

このパッケージには、LeKiwiロボットのテレオペレーションデータをLeRobot Dataset v3形式で記録するためのROS2ノードが含まれている。

## ファイル構成

- `lekiwi_data_recorder.py`: データ収集用ROS2ノード（オンライン直接LeRobot v3書き出し）
- `lekiwi_bag_recorder.py`: **推奨** MCAP rosbag でエピソード単位に記録するノード
- `rosbag_to_lerobot.py`: MCAP → LeRobot v3 のオフライン変換 & 同期分析ツール
- `lekiwi_ros2_teleop_client.py`: テレオペレーション用ROS2クライアント
- `upload_dataset.py`: データセットをHugging Face Hubにアップロードするスクリプト
- `launch/lekiwi_record.launch.py`: データ記録用launchファイル（直接LeRobot方式）
- `launch/lekiwi_bag_record.launch.py`: MCAP方式のlaunchファイル

## セットアップ

1. 環境変数を設定:
```bash
export LEROBOT_PATH="/path/to/lerobot/src"
export LEKIWI_REMOTE_IP="172.18.134.136"  # LeKiwiロボットのIPアドレス
```

2. パッケージをビルド:
```bash
cd $HOME/jazzy_ws
colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash
```

## 使用方法

## 推奨: MCAP rosbag 記録 → オフラインで LeRobot v3 に変換

直接LeRobot形式で書き込む代わりに、まず MCAP rosbag として raw データを保存し、
後段のオフラインスクリプトで同期チェック・フォーマッティングを行うワークフロー。

**メリット**
- リアルタイム側は動画エンコード等を行わないので取りこぼしが起きにくい
- 時刻同期・特徴量定義・画像リサイズを後から何度でも調整できる
- `header.stamp` に基づく厳密な最近傍/補間ペアリングが可能
- 同期ズレの分布を可視化してから許容幅を決められる

### 前提: MCAP ストレージプラグイン

```bash
sudo apt install ros-jazzy-rosbag2-storage-mcap
```

### ステップ1: テレオペレーションノードを起動

上の「推奨ワークフロー」と同じ。

### ステップ2: bag レコーダーを起動

```bash
source install/setup.bash
ros2 launch lekiwi_ros2_teleop lekiwi_bag_record.launch.py \
    launch_teleop:=false \
    output_root:=$HOME/lekiwi_bags \
    single_task:="Pick and place the bottle cap" \
    target_fps:=30
```

- `output_root` 配下に `session_YYYYmmdd_HHMMSS/` が作られる。
- `session_name` を指定すれば任意名。既存セッションを指定するとエピソードが追加される。

### ステップ3: エピソードの記録

```bash
ros2 service call /lekiwi_bag_recorder/start_episode std_srvs/srv/Trigger
# ... テレオペ操作 ...
ros2 service call /lekiwi_bag_recorder/stop_episode  std_srvs/srv/Trigger
```

各エピソードは `session_.../episode_000000/bag/` に MCAP として保存され、
`episode.yaml` に `single_task` などのメタが記録される。

### ステップ4: 同期分析（変換前に必ず実行推奨）

各エピソードで、`header.stamp` を使った最近傍ペアリング時のズレを集計し、
「許容幅ごとの棄却率」を表示する。

```bash
python3 src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/rosbag_to_lerobot.py \
    --session-dir $HOME/lekiwi_bags/session_20260923_120000 \
    --mode analyze \
    --action-lag-ms 0
```

出力例:

```
--- Drop rate vs. --sync-tolerance-ms ---
  tol_ms |     kept |  dropped |   drop%
--------------------------------------------
    5.0 |     1234 |      560 |  31.20%
   10.0 |     1650 |      144 |   8.03%
   20.0 |     1780 |       14 |   0.78%
   30.0 |     1794 |        0 |   0.00%
...
--- Per-frame max-gap distribution (ms) ---
  p50=6.42  p90=14.85  p95=17.20  p99=25.10  max=38.4
```

この結果を見ながら、後段の `--sync-tolerance-ms` を決める。

### ステップ5: LeRobot v3 データセットに変換

```bash
python3 src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/rosbag_to_lerobot.py \
    --session-dir $HOME/lekiwi_bags/session_20260923_120000 \
    --mode convert \
    --dataset-repo-id john/lekiwi_pick_place \
    --dataset-root $HOME/lerobot_datasets \
    --fps 30 \
    --sync-tolerance-ms 20 \
    --action-lag-ms 0
```

または colcon build 後は entry point で:

```bash
lekiwi_rosbag_to_lerobot --session-dir ... --mode convert --dataset-repo-id ...
```

### 同期・ペアリングのパラメータ

| パラメータ | 意味 | デフォルト | 推奨 |
|---|---|---|---|
| `--sync-base-topic` | 時刻同期の基準トピック | 未指定なら observation 系で最も低周波なものを自動選択 | カメラのどちらか（30Hz） |
| `--sync-tolerance-ms` | 基準stampから他トピック最近傍までの最大許容ズレ [ms]。超えたフレームは棄却 | `20.0` | `analyze` の p95 付近を目安。30Hz なら 15〜25ms |
| `--action-lag-ms` | action 側のstampに加算するオフセット [ms]。`observation_ts + lag` に最も近い action を採用 | `0.0` | 0 が無難。テレオペのループ遅延を打ち消したいときは `+1/fps` (30Hz なら `+33`) を試す |
| `--tolerance-sweep-ms` | analyze モードで表示するズレ許容幅の一覧 | `5 10 15 20 25 30 40 50 75 100` | 用途に応じて変更 |

**`--action-lag-ms` の推奨デフォルト** について:
- 一般的な imitation learning では「観測 obs_t と、その時刻にオペレータが出していた
  指令 action_t を対で学ぶ」ため `0` が自然です。
- 実機では leader arm 読み取り → 送信 → 受信 → 記録の間に遅延が入るので、
  `analyze` を最小許容幅で走らせて action 系のズレが片側に偏っているようなら、
  片側偏りが最小になる方向へ `+15〜+33ms` 程度で試すと良いです。

### 既存 LeRobot データセットへの追記

```bash
python3 src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/rosbag_to_lerobot.py \
    --session-dir $HOME/lekiwi_bags/session_20260923_120000 \
    --mode convert \
    --dataset-repo-id john/lekiwi_pick_place \
    --resume
```

---

## 使用方法（旧: 直接 LeRobot 書き込み方式）

#### ステップ1: テレオペレーションノードを起動

```bash
ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
    --ros-args \
    -p leader_arm_port:=/dev/ttyACM0 \
    -p control_frequency:=30.0 \
    -p use_keyboard:=false
```

台車をキーボード操作する場合

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r /cmd_vel:=/lekiwi/cmd_vel
```

台車をjoyコン操作する場合
```bash
ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py
```

#### ステップ2: データレコーダーノードを起動（別のターミナル）

**新しいデータセットを作成する場合:**
```bash
source install/setup.bash
ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py \
    launch_teleop:=false \
    dataset_repo_id:=john/lekiwi_pick_place \
    fps:=30 \
    single_task:="Pick and place the bottle cap"
```

**既存のデータセットにエピソードを追加する場合:**
```bash
source install/setup.bash
ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py \
    launch_teleop:=false \
    dataset_repo_id:=john/lekiwi_pick_place \
    fps:=30 \
    resume:=true \
    single_task:="Pick and place the bottle cap"
```

**重要な注意事項**: 
- `dataset_repo_id`は**必ず**`username/dataset_name`の形式で指定してください
  - ✓ 正しい例: `john/lekiwi_pick_place`, `alice/bottle_cap_demo`, `lab/manipulation_001`
  - ✗ 間違い例: `my_dataset`, `lekiwi_data`, `test` (スラッシュなし)
- この形式で指定することで、データセットが`~/lerobot_datasets/username/dataset_name/`に正しく保存されます
- `username`部分は任意の名前（自分の名前やプロジェクト名など）で構いません
- スラッシュ`/`を含む形式にしないと、LeRobotの可視化ツールやトレーニングスクリプトが正しく動作しません
- **`resume:=false`** (デフォルト): 新しいデータセットを作成。既存のデータセットがある場合はエラー
- **`resume:=true`**: 既存のデータセットにエピソードを追加。データセットがない場合はエラー

**注意**: 
- `launch_teleop:=false`を指定することで、既に起動しているテレオペノードと競合しません。
- 初回起動時または新しいデータセットを作成する場合は、データセットが自動的に作成されます。
- フィーチャー定義を変更した場合は、既存のデータセットディレクトリを削除する必要があります（トラブルシューティング参照）。

#### ステップ3: エピソードの記録（別のターミナル）

データレコーダーノードが起動したら、サービスを使用してエピソードを制御します:

**エピソードの開始**
```bash
ros2 service call /lekiwi_data_recorder/start_episode std_srvs/srv/Trigger
```

**エピソードの停止と保存**
```bash
ros2 service call /lekiwi_data_recorder/stop_episode std_srvs/srv/Trigger
```

**複数エピソードを記録する場合**
エピソードを停止した後、再度`start_episode`を呼び出して次のエピソードを記録できます。

#### ステップ4: 記録の終了

全てのエピソードを記録し終えたら:

1. データセットの状態を確認（オプション）:
```bash
ros2 service call /lekiwi_data_recorder/save_dataset std_srvs/srv/Trigger
```

2. データレコーダーノードを終了: `Ctrl+C`

3. テレオペレーションノードを終了: `Ctrl+C`

**注意**: 各エピソードは`stop_episode`サービスで既に保存されているため、Ctrl+Cで安全に終了できます。

---

### 方法2: Launchファイルで両方を同時に起動

テレオペレーションノードとデータレコーダーノードを同時に起動:

```bash
ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py \
    launch_teleop:=true \
    dataset_repo_id:=username/my_dataset \
    single_task:="Pick and place the cube" \
    leader_arm_port:=/dev/ttyACM0 \
    fps:=30
```

### 方法3: ノードを完全に個別に起動

#### ステップ1: テレオペレーションノードを起動

```bash
ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
    --ros-args \
    -p leader_arm_port:=/dev/ttyACM0 \
    -p use_keyboard:=false
```

#### ステップ2: データレコーダーノードを起動（別のターミナル）

```bash
ros2 run lekiwi_ros2_teleop lekiwi_data_recorder \
    --ros-args \
    -p dataset_repo_id:=username/my_dataset \
    -p single_task:="Pick and place the cube" \
    -p fps:=30
```

### データセットのアップロード（オプション）

記録が完了したら、データセットをHugging Face Hubにアップロードできます:

```bash
lekiwi_upload_dataset \
    --dataset_repo_id username/my_dataset \
    --dataset_root ~/lerobot_datasets \
    --private \
    --tags robot lekiwi teleoperation
```

または、Pythonスクリプトとして直接実行:

```bash
python3 src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/upload_dataset.py \
    --dataset_repo_id username/my_dataset \
    --private
```

## パラメータ

### データレコーダーノードのパラメータ

- `dataset_repo_id` (string): データセットのリポジトリID（例: `username/dataset_name`）。**必ず`/`を含む形式で指定**
- `dataset_root` (string): データセット保存先のルートディレクトリ（デフォルト: `~/lerobot_datasets`）
- `single_task` (string): 記録するタスクの説明（例: "Pick and place the cube"）
- `fps` (int): 記録フレームレート（デフォルト: 30）
- `robot_type` (string): ロボットタイプ（デフォルト: `lekiwi_client`）
- `resume` (bool): 既存データセットへの追記モード（デフォルト: false）
  - `false`: 新規データセット作成（既存があればエラー）
  - `true`: 既存データセットにエピソード追加（なければエラー）
- `use_videos` (bool): 画像を動画としてエンコード（デフォルト: true）
- `num_image_writer_processes` (int): 画像書き込みプロセス数（デフォルト: 0）
- `num_image_writer_threads` (int): カメラあたりのスレッド数（デフォルト: 4）
- `video_encoding_batch_size` (int): 動画エンコードのバッチサイズ（デフォルト: 1）

### テレオペレーションノードのパラメータ

- `lekiwi_remote_ip` (string): LeKiwiロボットのIPアドレス
- `leader_arm_port` (string): SO100リーダーアームのシリアルポート
- `control_frequency` (float): 制御周波数（Hz）（デフォルト: 30.0）
- `use_keyboard` (bool): キーボードテレオペレーションを有効化（デフォルト: true）

## データフロー

```
[テレオペレーター]
    ↓
[lekiwi_ros2_teleop_client]
    ↓ (ROS2トピック)
    ├─ /lekiwi/arm_joint_commands
    ├─ /lekiwi/cmd_vel
    ↓
[LeKiwiロボット]
    ↓ (ROS2トピック)
    ├─ /lekiwi/joint_states
    ├─ /lekiwi/camera/front/image_raw/compressed
    └─ /lekiwi/camera/wrist/image_raw/compressed
    ↓
[lekiwi_data_recorder]
    ↓
[LeRobot Dataset v3]
```

## トラブルシューティング

### データセット作成時のエラー

**症状**: データレコーダー起動時に`FileExistsError`や`KeyError: 'shape'`などのエラーが発生する

**原因**: 
- 既存のデータセットディレクトリが残っている
- フィーチャー定義が変更されたが、古いデータセットメタデータが残っている

**解決方法**:
新しいデータセットを作成する場合、または画像解像度などのフィーチャー定義を変更した場合は、既存のデータセットディレクトリを削除してください：

```bash
rm -rf $HOME/lerobot_datasets/username/my_dataset
# または全てのデータセットを削除する場合
rm -rf $HOME/lerobot_datasets
```

**注意**: この操作により既存のデータが削除されます。必要なデータは事前にバックアップしてください。

### トレーニング時の KeyError: 'names' エラー

**症状**: Colab等でトレーニングを開始すると`KeyError: 'names'`というエラーが発生する

```
File "/content/lerobot/src/lerobot/datasets/utils.py", line 723, in dataset_to_policy_features
    names = ft["names"]
            ~~^^^^^^^^^
KeyError: 'names'
```

**原因**: 
古いバージョンのデータレコーダーで記録されたデータセットのメタデータに、画像フィーチャーの`names`キーが含まれていない

**解決方法**:
1. データレコーダーのコードを最新版に更新してください（画像フィーチャーに`names: ["height", "width", "channels"]`が追加されています）
2. 既存のデータセットを削除し、新しいメタデータで再作成してください：
```bash
rm -rf ~/lerobot_datasets/username/my_dataset
```
3. データを再記録してください

**代替方法（データを保持したい場合）**:
既存のデータセットのメタデータファイル`~/lerobot_datasets/username/my_dataset/meta/info.json`を手動で編集し、画像フィーチャーに`"names": ["height", "width", "channels"]`を追加します。

### データセットに追加記録する場合

既存のデータセットにエピソードを追加したい場合は、データセットディレクトリを削除**しないで**ください。
データレコーダーは自動的に既存のデータセットを読み込み、新しいエピソードを追加します。

### データが記録されない
- すべての必要なトピックがパブリッシュされているか確認: `ros2 topic list`
- データレコーダーが警告を出していないか確認

### 画像が記録されない
- カメラトピックがパブリッシュされているか確認: `ros2 topic echo /lekiwi/camera/front/image_raw/compressed`
- 画像デコードエラーがログに出ていないか確認

### アップロードが失敗する
- Hugging Face CLIにログイン: `huggingface-cli login`
- リポジトリへの書き込み権限があるか確認
- インターネット接続を確認

### データセット可視化時のエラー

**症状**: `lerobot-dataset-viz`でエピソード2以降を指定すると`IndexError: Invalid key: XXXX is out of bounds for size YYYY`が発生する

**原因**: 
これはLeRobotの`lerobot-dataset-viz`ツール側のバグです。特定のエピソードを指定した場合、データをフィルタリングした後も累積インデックスでアクセスしようとするため、範囲外エラーが発生します。

**解決方法（回避策）**:

1. **エピソード0のみ可視化する**:
```bash
lerobot-dataset-viz \
    --repo-id username/my_dataset \
    --root ~/lerobot_datasets \
    --episode-index 0
```

2. **全エピソードを含めて可視化** (エピソード指定なし):
```bash
# 注意: このオプションは現在のバージョンでサポートされていない可能性があります
lerobot-dataset-viz \
    --repo-id username/my_dataset \
    --root ~/lerobot_datasets
```

3. バグフィックス版（一時しのぎ）:
```
python src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/lekiwi_dataset_viz.py   --repo-id  [dataset path]  --episode-index 1
```
<!-- ### データセットのトレーニングでの使用

データセットは正しく保存されているため、トレーニングには問題なく使用できます：

```bash
# Colabやローカル環境でのトレーニング例
lerobot-train \
    policy=act \
    env=real_world \
    dataset.repo_id=username/my_dataset \
    dataset.root=~/lerobot_datasets -->
```

## 参考リンク

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [LeRobot Getting Started](https://huggingface.co/docs/lerobot/getting_started_real_world_robot#record-a-dataset)
