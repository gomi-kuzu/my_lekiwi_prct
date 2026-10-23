# ノード・トピック・パラメータ リファレンス

本パッケージに含まれるノード／CLI ツールの機能・トピック・パラメータの一覧。
概要・アーキテクチャは [README](../README.md) を参照すること。

---

## ノード / ツール一覧

| 実行名 | 種別 | 役割 |
|---|---|---|
| `lekiwi_teleop_node` | ノード | LeKiwi 実機を制御。ROS2 トピックでコマンド受信・状態/カメラ配信 |
| `lekiwi_policy_node` | ノード | 学習済みポリシーで自律制御（GPU PC 上で実行） |
| `lekiwi_ros2_teleop_client` | ノード | SO-101 Leader Arm / キーボード入力を受け取りコマンド送信 |
| `lekiwi_bag_recorder` | ノード | テレオペ中のトピックを MCAP bag で記録（パイプライン Stage 1） |
| `lekiwi_rosbag_to_lerobot` | CLI | bag を LeRobot Dataset v3 に変換（パイプライン Stage 2） |
| `lekiwi_data_recorder` | ノード | LeRobot v3 へ直接記録する旧経路（非推奨。[direct_recording.md](direct_recording.md) 参照） |
| `lekiwi_upload_dataset` | CLI | データセットを Hugging Face Hub にアップロード |

---

## 機能詳細

### lekiwi_teleop_node

- **トピック通信**: ROS2 トピックを使用したロボット制御
- **アーム制御**: 5自由度+グリッパアームの位置制御
- **ベース制御**: オムニホイールベースの速度制御
- **カメラ配信**: フロントとリストカメラの画像配信
- **ウォッチドッグ**: コマンドが一定時間受信されない場合、ベースを自動停止

### lekiwi_policy_node

- **ポリシー推論**: 学習済みポリシーをロードし、カメラ画像・関節状態から次のアクションを推論
- **リモート実行**: GPU 搭載 PC 上で動作し、ROS2 トピック経由で `lekiwi_teleop_node` と通信
- **サービス制御**: `/lekiwi/policy/start` / `/lekiwi/policy/stop` で推論の開始・停止を制御
- **周波数設定**: `control_frequency` で制御ループ周波数を設定（30Hz で動作）

詳細は [policy_inference.md](policy_inference.md) を参照すること。

### lekiwi_ros2_teleop_client

- **テレオペレーション**: SO-101 Leader Arm とキーボードからの入力取得
- **ROS2 パブリッシュ**: `lekiwi_teleop_node` へコマンド送信
- **環境変数対応**: `LEKIWI_REMOTE_IP` で LeKiwi の IP アドレスを指定
- **可視化**: Rerun 統合によるリアルタイムモニタリング（オプション）

---

## トピック

### Subscribe（`lekiwi_teleop_node` が受信）

- `/lekiwi/cmd_vel` (geometry_msgs/TwistStamped): ベース速度コマンド
  - `twist.linear.x`: X 方向速度 (m/s)
  - `twist.linear.y`: Y 方向速度 (m/s)
  - `twist.angular.z`: 回転速度 (rad/s)
  - 素の `geometry_msgs/Twist` を使いたい場合は `cmd_vel_stamped:=false` を指定する
- `/lekiwi/arm_joint_commands` (sensor_msgs/JointState): アーム関節位置コマンド（6 関節）

### Publish（`lekiwi_teleop_node` が配信）

- `/lekiwi/joint_states` (sensor_msgs/JointState): 現在のロボット状態
  - `position`: アーム関節位置
  - `velocity`: ベース速度
- `/lekiwi/camera/front/image_raw/compressed` (sensor_msgs/CompressedImage): フロントカメラ画像
- `/lekiwi/camera/wrist/image_raw/compressed` (sensor_msgs/CompressedImage): リストカメラ画像

---

## パラメータ

### lekiwi_teleop_node

- `robot_port` (string, default: `/dev/ttyACM0`): ロボットのシリアルポート
- `control_frequency` (float, default: `30.0`): 制御ループ周波数 (Hz)
- `watchdog_timeout_ms` (int, default: `500`): ウォッチドッグタイムアウト (ms)
- `use_degrees` (bool, default: `false`): 角度単位を度で使用するか
- `cmd_vel_stamped` (bool, default: `true`): `/lekiwi/cmd_vel` を `TwistStamped` で購読するか（`false` で素の `Twist`）
- `rotate_front_camera` (bool, default: `true`): フロントカメラを 180 度回転するか（逆さ取り付け時）
- `camera_manual_exposure` (bool, default: `true`): 自動露出（`auto_exposure=3`）を無効化しマニュアル露出に固定するか。自動露出のままだと暗所で露出時間が伸び実効フレームレートが ~13.5fps まで低下するため、短い露出時間に固定して ~28-30fps を維持する。`false` でドライバー既定（自動露出）
- `camera_exposure_time` (int, default: `200`): マニュアル露出時の `exposure_time_absolute`（100us 単位、200 = 20ms）。30fps 近くを維持するには 200 前後を推奨。大きくすると明るくなるがフレームレートが低下
- `camera_gain` (int, default: `63`): マニュアル露出時のアナログゲイン（0〜63）。大きいほど明るくなるがノイズも増える
- `camera_power_line_frequency` (int, default: `0`): フリッカー対策の電源周波数（0=Disabled, 1=50Hz, 2=60Hz）

### lekiwi_policy_node

- `policy_path` (string, **必須**): 学習済みポリシーのチェックポイントパス
- `dataset_repo_id` (string, **必須**): データセットのリポジトリ ID（特徴取得用）
- `control_frequency` (float, default: `30.0`): 制御ループ周波数 (Hz)
- `device` (string, default: `cuda`): 推論に使用するデバイス
- `single_task` (string, default: `""`): タスクの単一言語説明
- `use_amp` (bool, default: `false`): 自動混合精度を使用するか

その他の推論調整パラメータは [policy_inference.md](policy_inference.md) を参照すること。

### lekiwi_ros2_teleop_client

- `lekiwi_remote_ip` (string, default: `""`): LeKiwi の IP アドレス（空なら環境変数 `LEKIWI_REMOTE_IP`）
- `lekiwi_id` (string, default: `my_lekiwi`): LeKiwi の ID
- `leader_arm_port` (string, default: `/dev/tty.usbmodem585A0077581`): SO-101 Leader Arm のシリアルポート
- `leader_arm_id` (string, default: `my_leader_arm`): Leader Arm の ID
- `keyboard_id` (string, default: `my_keyboard`): Keyboard の ID
- `control_frequency` (float, default: `30.0`): 制御ループ周波数 (Hz)
- `use_rerun` (bool, default: `false`): Rerun 可視化を使用するか
- `use_keyboard` (bool, default: `true`): 内部キーボードテレオペを使用するか（`false` で外部の `teleop_twist_keyboard` を使用）
