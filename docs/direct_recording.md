# データ記録（旧経路: LeRobot へ直接書き込み）

> **⚠️ 非推奨 / レガシー**: この `lekiwi_data_recorder` による「テレオペ中に LeRobot Dataset v3 を
> 直接書き込む」経路は、**取りこぼしや後からの再同期ができない**という制約がある。
> 現在の推奨は **MCAP bag で生データを記録し、後段でオフライン変換する**データパイプラインで、
> 設計・使い方は [data_pipeline.md](data_pipeline.md) を参照すること。本ドキュメントは既存運用の
> 互換のために最小限の手順のみ残す。

## 概要

`lekiwi_data_recorder` は、テレオペレーション中のカメラ画像・関節状態を
LeRobot Dataset v3 形式でリアルタイムに書き込むノード。

- ノード: `lekiwi_data_recorder`
- Launch: `launch/lekiwi_record.launch.py`
- アップロード: `lekiwi_upload_dataset`（CLI）

環境構築・環境変数は [setup.md](setup.md) を参照すること。

## クイックスタート

### 1. テレオペレーションノードを起動

```bash
ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
    --ros-args \
    -p leader_arm_port:=/dev/ttyACM0 \
    -p use_keyboard:=false
```

台車をキーボードで操作する場合:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/lekiwi/cmd_vel
```

### 2. データレコーダーを起動（別ターミナル）

**新しいデータセットを作成:**

```bash
source install/setup.bash
ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py \
    launch_teleop:=false \
    dataset_repo_id:=john/lekiwi_pick_place \
    fps:=30 \
    single_task:="Pick and place the bottle cap"
```

**既存データセットに追記:** 上記に `resume:=true` を追加する。

> `dataset_repo_id` は **必ず** `username/dataset_name` 形式で指定する
> （`~/lerobot_datasets/username/dataset_name/` に保存され、LeRobot のツールが正しく動く）。

### 3. エピソードの記録（別ターミナル）

```bash
# 開始
ros2 service call /lekiwi_data_recorder/start_episode std_srvs/srv/Trigger
# ... テレオペ操作 ...
# 停止と保存
ros2 service call /lekiwi_data_recorder/stop_episode std_srvs/srv/Trigger
```

各エピソードは `stop_episode` で保存されるため、記録し終えたら `Ctrl+C` で安全に終了できる。

### 4. データセットのアップロード（オプション）

```bash
lekiwi_upload_dataset \
    --dataset_repo_id username/my_dataset \
    --dataset_root ~/lerobot_datasets \
    --private \
    --tags robot lekiwi teleoperation
```

## 主なパラメータ（lekiwi_data_recorder）

- `dataset_repo_id` (string): データセットのリポジトリ ID（`username/dataset_name` 形式必須）
- `dataset_root` (string, default: `~/lerobot_datasets`): 保存先ルート
- `single_task` (string): タスク説明
- `fps` (int, default: `30`): 記録フレームレート
- `robot_type` (string, default: `lekiwi_client`): ロボットタイプ
- `resume` (bool, default: `false`): `true` で既存データセットに追記
- `use_videos` (bool, default: `true`): 画像を動画としてエンコード

## トラブルシューティング

### データセット作成時に `FileExistsError` / `KeyError: 'shape'`

既存ディレクトリやフィーチャー定義の不整合が原因。新規作成時やフィーチャー定義変更時は
既存ディレクトリを削除する:

```bash
rm -rf $HOME/lerobot_datasets/username/my_dataset
```

> **注意**: 既存データが削除される。必要なら事前にバックアップすること。

### データが記録されない / 画像が記録されない

- 必要なトピックが publish されているか確認: `ros2 topic list`
- カメラトピックを確認: `ros2 topic echo /lekiwi/camera/front/image_raw/compressed`

### アップロードが失敗する

- Hugging Face CLI にログイン: `huggingface-cli login`
- リポジトリへの書き込み権限・ネットワーク接続を確認

## 参考リンク

- 推奨パイプライン（bag → 変換）: [data_pipeline.md](data_pipeline.md)
- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
