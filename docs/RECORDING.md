# LeKiwi ROS2 Data Recording

このパッケージには、LeKiwiロボットのテレオペレーションデータをLeRobot Dataset v3形式で記録するためのROS2ノードが含まれています。

## ファイル構成

- `lekiwi_data_recorder.py`: データ収集用ROS2ノード
- `lekiwi_ros2_teleop_client.py`: テレオペレーション用ROS2クライアント
- `upload_dataset.py`: データセットをHugging Face Hubにアップロードするスクリプト
- `launch/lekiwi_record.launch.py`: データ記録用launchファイル

## セットアップ

1. 環境変数を設定:
```bash
export LEROBOT_PATH="/path/to/lerobot/src"
export LEKIWI_REMOTE_IP="172.18.134.136"  # LeKiwiロボットのIPアドレス
```

2. パッケージをビルド:
```bash
cd /home/inoma/jazzy_ws
colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash
```

## 使用方法

### 推奨ワークフロー: テレオペノードとデータレコーダーを別々に起動

#### ステップ1: テレオペレーションノードを起動

```bash
ros2 run lekiwi_ros2_teleop lekiwi_ros2_teleop_client \
    --ros-args \
    -p leader_arm_port:=/dev/ttyACM0 \
    -p control_frequency:=50.0 \
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
rm -rf /home/inoma/lerobot_datasets/username/my_dataset
# または全てのデータセットを削除する場合
rm -rf /home/inoma/lerobot_datasets
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
### データセットのトレーニングでの使用

データセットは正しく保存されているため、トレーニングには問題なく使用できます：

```bash
# Colabやローカル環境でのトレーニング例
lerobot-train \
    policy=act \
    env=real_world \
    dataset.repo_id=username/my_dataset \
    dataset.root=~/lerobot_datasets
```

## 参考リンク

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [LeRobot Getting Started](https://huggingface.co/docs/lerobot/getting_started_real_world_robot#record-a-dataset)
