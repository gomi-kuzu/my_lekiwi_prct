# LeKiwi ROS2 Teleoperation - Copilot Development Instructions

このドキュメントは、LeKiwi ROS2 Teleoperation プロジェクトの開発支援を行うAIエージェントのための包括的なガイドラインです。

## 目次

1. [プロジェクト概要](#プロジェクト概要)
2. [アーキテクチャと設計思想](#アーキテクチャと設計思想)
3. [技術スタック](#技術スタック)
4. [開発ポリシー](#開発ポリシー)
5. [コーディング規約](#コーディング規約)
6. [ドキュメント規約](#ドキュメント規約)
7. [既存コンポーネント](#既存コンポーネント)
8. [トラブルシューティングパターン](#トラブルシューティングパターン)
9. [テスト戦略](#テスト戦略)
10. [デプロイメント](#デプロイメント)

---

## プロジェクト概要

### 目的
LeKiwiオムニホイールロボット（6自由度アーム搭載）のROS2制御システムとLeRobotフレームワークを統合し、テレオペレーション、データ収集、モデル推論を実現する。

### 主要機能
1. **テレオペレーション**: SO100 Leader Armとキーボードによる遠隔操作
2. **データ収集**: LeRobot Dataset v3形式での学習データ記録
3. **ポリシー推論**: 学習済みモデルによる自律制御（分散システム対応）
4. **VLA推論**: Vision-Language-Actionモデルによる言語指示ベース制御
5. **データセット管理**: 可視化、検証、Hugging Face Hubアップロード

### システム構成
- **ロボット側（Raspberry Pi）**: ロボット直接制御、センサーデータ配信
- **PC側（GPU搭載）**: 推論、データ記録、テレオペクライアント
- **通信**: ROS2トピック（DDS）による分散通信

---

## アーキテクチャと設計思想

### 設計原則

#### 1. 疎結合アーキテクチャ
- **原則**: 各ノードは独立して起動・停止可能
- **実装**: ROS2トピックベースの通信、直接的な依存関係を避ける
- **理由**: デバッグの容易さ、テストの独立性、段階的な機能追加

#### 2. 分散システム志向
- **原則**: 計算負荷の高い処理（推論）とリアルタイム制御を分離
- **実装**: 
  - Raspberry Pi: `lekiwi_teleop_node`（ロボット制御）
  - Desktop PC: `lekiwi_policy_node`、`lekiwi_vla_node`（推論）
- **理由**: GPUリソースの効率的活用、ロボット側の負荷軽減

#### 3. LeRobot互換性の維持
- **原則**: LeRobotのデータフォーマットとAPIに準拠
- **実装**: Dataset v3形式、`build_dataset_frame()`、`predict_action()`の使用
- **理由**: LeRobotエコシステムとの相互運用性、事前学習モデルの活用

#### 4. 段階的な機能提供
- **原則**: 基本機能から高度な機能へ段階的に実装
- **実装順序**:
  1. 基本テレオペ（`lekiwi_teleop_node`）
  2. データ記録（`lekiwi_data_recorder`）
  3. ポリシー推論（`lekiwi_policy_node`）
  4. VLA推論（`lekiwi_vla_node`）
- **理由**: 早期のフィードバック、リスク軽減

### トピック設計

#### パブリッシュトピック（ロボット → システム）
```python
/lekiwi/joint_states              # sensor_msgs/JointState - アーム・ベース状態
/lekiwi/camera/front/image_raw/compressed  # sensor_msgs/CompressedImage
/lekiwi/camera/wrist/image_raw/compressed  # sensor_msgs/CompressedImage
```

#### サブスクライブトピック（システム → ロボット）
```python
/lekiwi/cmd_vel                   # geometry_msgs/Twist - ベース速度指令
/lekiwi/arm_joint_commands        # sensor_msgs/JointState - アーム位置指令
```

#### サービス
```python
# データ記録
/lekiwi/data_recorder/start_episode    # std_srvs/Trigger
/lekiwi/data_recorder/stop_episode     # std_srvs/Trigger
/lekiwi/data_recorder/save_dataset     # std_srvs/Trigger

# ポリシー推論
/lekiwi/policy/start                   # std_srvs/Trigger
/lekiwi/policy/stop                    # std_srvs/Trigger
```

### 命名規約

#### ノード名
- パターン: `lekiwi_<機能>_node`
- 例: `lekiwi_teleop_node`, `lekiwi_policy_node`, `lekiwi_vla_node`

#### トピック名
- パターン: `/lekiwi/<カテゴリ>/<詳細>`
- 例: `/lekiwi/camera/front/image_raw/compressed`

#### ファイル名
- パターン: `lekiwi_<機能>_<種類>.py`
- 例: `lekiwi_data_recorder.py`, `lekiwi_policy_node.py`

---

## 技術スタック

### 必須コンポーネント

#### ROS2環境
- **ディストリビューション**: ROS2 Jazzy
- **Python**: 3.12
- **ビルドシステム**: colcon
- **パッケージタイプ**: ament_python

#### LeRobotフレームワーク
- **バージョン**: 最新（GitHub main）
- **環境変数**: `LEROBOT_PATH`（必須）
- **データ形式**: LeRobot Dataset v3
- **主要API**:
  ```python
  from lerobot.common.robot_devices.robots.factory import make_robot
  from lerobot.common.datasets.factory import make_dataset
  from lerobot.common.policies.factory import make_policy
  from lerobot.scripts.push_dataset_to_hub import push_dataset_to_hub
  ```

#### ハードウェアインターフェース
- **ロボット通信**: LeKiwi Python SDK（ZMQ/ROS2ブリッジ）
- **リーダーアーム**: SO100 Leader Arm（LeRobot互換）
- **カメラ**: USB Webカメラ × 2（フロント・リスト）

#### 推論エンジン
- **フレームワーク**: PyTorch
- **デバイス**: CUDA（推奨）、CPU、MPS
- **最適化**: AMP（自動混合精度）、TorchScript（オプション）

### オプショナルコンポーネント
- **可視化**: Rerun、rqt_image_view、rviz2
- **データアップロード**: Hugging Face Hub CLI
- **テレオペ**: teleop_twist_keyboard、teleop_twist_joy

---

## 開発ポリシー

### 1. コードの追加・修正時の原則

#### 1.1 既存コードの尊重
- **DO**: 既存の実装パターンを継承する
- **DON'T**: 理由なく構造を大きく変更しない
- **例**: 
  ```python
  # 既存パターン: ROS2ノードの初期化
  super().__init__('node_name')
  self.get_logger().info('Starting node...')
  ```

#### 1.2 参照実装の活用
- **必須参照**: LeRobotの公式スクリプト
  - `lerobot/scripts/lerobot_record.py` - データ記録の参考
  - `lerobot/scripts/control_robot.py` - ロボット制御の参考
- **DO**: LeRobotのAPIとデータ構造を正確に模倣する
- **DON'T**: 独自のデータ形式や制御フローを発明しない

#### 1.3 段階的な実装
1. **最小限の動作確認** → コンパイル・起動テスト
2. **コア機能の実装** → 基本動作の検証
3. **エラーハンドリング** → 例外処理・ログ追加
4. **ドキュメント更新** → README、専用ドキュメント
5. **最適化** → パフォーマンス改善

### 2. エラーハンドリング

#### 2.1 環境変数チェック（起動時必須）
```python
import os

# 必須環境変数のチェック
LEROBOT_PATH = os.environ.get('LEROBOT_PATH')
if not LEROBOT_PATH:
    raise EnvironmentError(
        "LEROBOT_PATH environment variable is not set. "
        "Please set it in ~/.bashrc or ~/.zshrc:\n"
        "  export LEROBOT_PATH=\"$HOME/study/lerobot/src\""
    )

# sys.pathへの追加
import sys
if LEROBOT_PATH not in sys.path:
    sys.path.insert(0, LEROBOT_PATH)
```

#### 2.2 ロバストなトピック購読
```python
# タイムアウト付きメッセージ待機
def wait_for_observations(self, timeout_sec=10.0):
    """Wait until all required observations are available."""
    start_time = self.get_clock().now()
    
    while not self._has_all_observations():
        if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
            self.get_logger().error('Timeout waiting for observations')
            return False
        rclpy.spin_once(self, timeout_sec=0.1)
    
    return True
```

#### 2.3 デバイスエラーの処理
```python
try:
    # ロボット接続試行
    robot = make_robot(robot_type, port=robot_port)
except Exception as e:
    self.get_logger().error(f'Failed to connect to robot: {e}')
    self.get_logger().info('Please check:')
    self.get_logger().info('  - Robot is powered on')
    self.get_logger().info(f'  - Port {robot_port} is correct')
    self.get_logger().info('  - User has permission: sudo chmod 666 /dev/ttyACM0')
    raise
```

### 3. ログ出力規約

#### 3.1 ログレベルの使い分け
```python
# DEBUG: 開発時の詳細情報
self.get_logger().debug(f'Received joint state: {msg.position}')

# INFO: 正常な動作の通知
self.get_logger().info('Policy inference started')

# WARN: 回復可能な問題
self.get_logger().warn('No observations received for 1 second')

# ERROR: 機能不全だが続行可能
self.get_logger().error(f'Failed to decode image: {e}')

# FATAL: 続行不可能な致命的エラー
self.get_logger().fatal('CUDA out of memory, exiting...')
```

#### 3.2 起動時の情報出力
```python
# ノード起動時に設定を明示
self.get_logger().info('=== Node Configuration ===')
self.get_logger().info(f'Policy path: {self.policy_path}')
self.get_logger().info(f'Dataset: {self.dataset_repo_id}')
self.get_logger().info(f'Device: {self.device}')
self.get_logger().info(f'Control frequency: {self.control_frequency} Hz')
self.get_logger().info('==========================')
```

### 4. パラメータ設計

#### 4.1 デフォルト値の設定
```python
# 安全で実用的なデフォルト値を設定
self.declare_parameter('control_frequency', 30.0)  # LeKiwiの実測値
self.declare_parameter('device', 'cuda')           # GPU優先
self.declare_parameter('use_amp', False)           # 安定性優先
self.declare_parameter('rotate_front_camera', True) # ハードウェア依存
```

#### 4.2 パラメータのバリデーション
```python
# 取得時に検証
policy_path = self.get_parameter('policy_path').value
if not policy_path:
    raise ValueError('policy_path parameter is required')

control_frequency = self.get_parameter('control_frequency').value
if control_frequency <= 0 or control_frequency > 100:
    raise ValueError(f'Invalid control_frequency: {control_frequency} (must be 0-100 Hz)')
```

### 5. データセット互換性

#### 5.1 フィーチャー定義の厳守
```python
# LeRobot Dataset v3のフィーチャー定義
features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (6,),  # アーム6自由度
        "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    },
    "observation.images.front": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channels"]  # ← 必須（トレーニング時のエラー防止）
    },
    # ...
}
```

#### 5.2 データ記録のベストプラクティス
```python
# エピソード単位で確実に保存
def stop_episode(self):
    """Stop current episode and save immediately."""
    if self.current_episode_data:
        # 即座に保存（Ctrl+C時のデータロス防止）
        self._save_episode_to_disk()
        self.episode_index += 1
        self.get_logger().info(f'Episode {self.episode_index - 1} saved')
```

### 6. 分散システムの注意点

#### 6.1 ROS2ドメイン設定
```bash
# 同一ネットワーク内の複数ロボットを区別
export ROS_DOMAIN_ID=42

# ローカルホスト専用モード（セキュリティ）
export ROS_LOCALHOST_ONLY=1

# 複数マシン間通信
export ROS_LOCALHOST_ONLY=0
```

#### 6.2 ネットワーク帯域の配慮
```python
# 画像はCompressedImageを使用（帯域削減）
from sensor_msgs.msg import CompressedImage

# 必要に応じて解像度を下げる
image_resized = cv2.resize(image, (640, 480))
```

---

## コーディング規約

### 1. Pythonスタイル

#### 1.1 基本規約
- **PEP 8準拠**: flake8、black、isortでフォーマット
- **型ヒント**: 関数シグネチャに型を明示（Python 3.12+）
```python
from typing import Optional, Dict, Any
import numpy as np

def predict_action(
    self,
    observation: Dict[str, np.ndarray],
    task: Optional[str] = None
) -> Dict[str, np.ndarray]:
    """Predict robot action from observation."""
    pass
```

#### 1.2 インポート順序
```python
# 1. 標準ライブラリ
import os
import sys
from typing import Dict, Optional

# 2. サードパーティライブラリ
import numpy as np
import torch
import cv2

# 3. ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 4. LeRobot
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.datasets.factory import make_dataset

# 5. ローカル（必要に応じて）
from .utils import transform_image
```

#### 1.3 クラス設計
```python
class LekiwiPolicyNode(Node):
    """
    LeKiwi policy inference node for distributed robot control.
    
    This node runs on a GPU-equipped desktop PC and publishes action
    commands to a lekiwi_teleop_node running on the robot's Raspberry Pi.
    
    Attributes:
        policy: Loaded policy model
        dataset: Dataset for preprocessing/postprocessing
        control_frequency: Control loop frequency in Hz
    """
    
    def __init__(self):
        """Initialize the policy inference node."""
        super().__init__('lekiwi_policy_node')
        
        # パラメータ宣言
        self._declare_parameters()
        
        # 環境変数チェック
        self._check_environment()
        
        # コンポーネント初期化
        self._initialize_policy()
        self._initialize_ros_interfaces()
        
        self.get_logger().info('LekiwiPolicyNode initialized successfully')
```

### 2. ROS2固有の規約

#### 2.1 ノードライフサイクル
```python
def __init__(self):
    """初期化: パラメータ宣言、環境チェック"""
    pass

def start(self):
    """開始: タイマー・サービスの有効化"""
    pass

def stop(self):
    """停止: リソース解放、安全な終了"""
    pass

def destroy_node(self):
    """破棄: 親クラスのクリーンアップ"""
    super().destroy_node()
```

#### 2.2 コールバック設計
```python
def joint_state_callback(self, msg: JointState) -> None:
    """
    Joint state callback.
    
    Args:
        msg: Joint state message containing positions and velocities
    """
    # スレッドセーフな状態更新
    with self._state_lock:
        self._latest_joint_state = msg
        self._joint_state_received = True
```

### 3. ドキュメンテーション

#### 3.1 Docstring形式（Google Style）
```python
def build_observation_from_topics(
    self,
    joint_state: JointState,
    front_image: np.ndarray,
    wrist_image: np.ndarray
) -> Dict[str, torch.Tensor]:
    """
    Build LeRobot observation dictionary from ROS2 topic data.
    
    Args:
        joint_state: Current joint positions and velocities
        front_image: RGB image from front camera (H, W, 3)
        wrist_image: RGB image from wrist camera (H, W, 3)
    
    Returns:
        Dictionary containing:
            - 'observation.state': Joint positions (6,)
            - 'observation.images.front': Front camera image
            - 'observation.images.wrist': Wrist camera image
    
    Raises:
        ValueError: If image shapes are invalid
    """
    pass
```

#### 3.2 インラインコメント
```python
# 明確でない処理には説明を追加
action = self.policy.predict_action(observation)

# LeRobotのaction形式: {'action': tensor([[x, y, z, ...]])}
# 最初の次元はバッチ、2番目は時間ステップ（temporal ensemble用）
action_array = action['action'][0, -1].cpu().numpy()

# アクション分割: [base_x, base_y, base_theta, arm_j1, ..., arm_j6]
base_action = action_array[:3]
arm_action = action_array[3:9]
```

---

## ドキュメント規約

### 1. ドキュメント構成

#### 必須ドキュメント
```
docs/
├── README.md              # プロジェクト概要、クイックスタート
├── RECORDING.md           # データ記録の詳細ガイド
├── POLICY_INFERENCE.md    # ポリシー推論の詳細ガイド
├── custom_joy_setup.md    # ジョイスティック設定
└── copilot-instructions.md # このファイル
```

### 2. ドキュメント記述ルール

#### 2.1 構成要素
各ドキュメントには以下を含める：
1. **概要**: 機能の簡潔な説明
2. **前提条件**: 必要な環境・ハードウェア
3. **セットアップ**: 段階的な設定手順
4. **使用方法**: 具体的なコマンド例
5. **パラメータ**: 全パラメータの説明と推奨値
6. **トラブルシューティング**: よくある問題と解決策
7. **参考リンク**: 関連ドキュメント・外部リンク

#### 2.2 コマンド例の書き方
```markdown
# 悪い例
推論を開始してください

# 良い例
デスクトップPC上で推論ノードを起動：
\`\`\`bash
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=~/outputs/train/2025-01-15/14-30-00/checkpoints/last \
    dataset_repo_id:=gomi-kuzu/lekiwi_pick_place \
    single_task:="Pick and place the cube"
\`\`\`
```

#### 2.3 パラメータドキュメント
```markdown
### パラメータ詳細

#### 必須パラメータ

- `policy_path` (string): 学習済みモデルのディレクトリパス
  - 例: `~/outputs/train/2025-01-15/14-30-00/checkpoints/last`
  - 注意: ディレクトリ内に `config.json` が必要

- `dataset_repo_id` (string): データセットのリポジトリID
  - 形式: `username/dataset_name`
  - 例: `gomi-kuzu/lekiwi_pick_place`
  - 注意: スラッシュ `/` を含む形式必須

#### オプションパラメータ

- `control_frequency` (float, default: 30.0): 制御周波数（Hz）
  - 範囲: 1.0 - 100.0
  - 推奨: 30.0（LeKiwiの実測最適値）
```

#### 2.4 トラブルシューティングセクション
```markdown
## トラブルシューティング

### エラー: "LEROBOT_PATH environment variable is not set"

**症状**: ノード起動時にEnvironmentErrorが発生

**原因**: LEROBOT_PATH環境変数が未設定

**解決方法**:
1. `~/.bashrc`（または`~/.zshrc`）を編集:
   \`\`\`bash
   export LEROBOT_PATH="$HOME/study/lerobot/src"
   \`\`\`
2. 設定を反映:
   \`\`\`bash
   source ~/.bashrc
   \`\`\`
3. 確認:
   \`\`\`bash
   echo $LEROBOT_PATH
   \`\`\`
```

---

## 既存コンポーネント

### 1. lekiwi_teleop_node.py

**責務**: ロボット直接制御、センサーデータ配信

**主要機能**:
- シリアル通信によるLeKiwiロボット制御
- ROS2トピックからベース・アーム指令を受信
- 関節状態、カメラ画像をROS2トピックに配信
- ウォッチドッグによる安全停止

**実行環境**: Raspberry Pi（ロボット搭載）

**起動例**:
```bash
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args -p robot_port:=/dev/ttyACM0
```

**重要な実装パターン**:
```python
# ウォッチドッグ実装
def _check_command_timeout(self):
    """Stop base if no command received within timeout."""
    if (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e6 > self._watchdog_timeout_ms:
        self.robot.send_action({'base': [0.0, 0.0, 0.0]})
```

### 2. lekiwi_policy_node.py

**責務**: GPU推論、分散制御システムのクライアント

**主要機能**:
- LeRobotポリシーモデルのロード・推論
- ROS2トピックから観測データ取得（分散システム）
- 予測アクションをROS2トピックに配信
- サービスによる推論開始/停止制御

**実行環境**: Desktop PC（GPU搭載）

**アーキテクチャ上の重要点**:
- **直接ロボット接続なし**: ROS2トピック経由のみ
- **観測はトピックから受信**: `/lekiwi/joint_states`, `/lekiwi/camera/*`
- **指令はトピックに配信**: `/lekiwi/cmd_vel`, `/lekiwi/arm_joint_commands`

**起動例**:
```bash
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=~/outputs/train/.../checkpoints/last \
    dataset_repo_id:=gomi-kuzu/lekiwi_pick_place
```

### 3. lekiwi_data_recorder.py

**責務**: テレオペデータのLeRobot Dataset v3形式での記録

**主要機能**:
- ROS2トピックから観測・指令を記録
- エピソード単位での管理（start/stop/save）
- データセットメタデータ自動生成
- 動画エンコード（オプション）

**実行環境**: Desktop PC

**重要な実装パターン**:
```python
# エピソード保存の確実性
def stop_episode_callback(self, request, response):
    """Stop episode and save immediately (prevent data loss on Ctrl+C)."""
    if self.is_recording:
        self._save_episode_to_disk()  # 即座に保存
        response.success = True
    return response
```

**データセット形式の注意点**:
- `dataset_repo_id`は`username/dataset_name`形式必須
- 画像フィーチャーに`names: ["height", "width", "channels"]`必須
- `resume=True`で既存データセットに追記可能

### 4. lekiwi_vla_node.py

**責務**: Vision-Language-Actionモデルによる言語指示ベース制御

**主要機能**:
- SmolVLA、X-VLAなどのVLAモデル推論
- 言語タスク指示の処理
- 時間的アンサンブルによる滑らかな制御

**実行環境**: Desktop PC（GPU推奨）

### 5. lekiwi_ros2_teleop_client.py

**責務**: テレオペレーション入力のROS2トピック変換

**主要機能**:
- SO100 Leader Armからアーム指令取得
- キーボード/Joystickからベース指令取得
- ROS2トピックへの指令配信

**実行環境**: Desktop PC

### 6. サポートスクリプト

#### upload_dataset.py
- データセットをHugging Face Hubにアップロード
- コマンド: `lekiwi_upload_dataset`

#### lekiwi_dataset_viz.py
- データセット可視化（LeRobotバグ回避版）
- エピソード指定時のIndexError修正済み

---

## トラブルシューティングパターン

### 1. ビルドエラー

#### パターン1: SyntaxError during build
**症状**: `colcon build`時にPythonファイルの構文エラー

**よくある原因**:
- 未閉じの括弧・引用符
- インデント不整合
- 不完全なメソッド定義

**診断方法**:
```bash
# Pythonファイルの構文チェック
python3 -m py_compile src/lekiwi_ros2_teleop/lekiwi_ros2_teleop/<file>.py

# エラー行の確認
grep -n "問題のコード" <file>.py
```

**解決策**:
1. エラー行周辺を`read_file`で確認
2. 小さい範囲で`replace_string_in_file`を使用
3. 複数箇所エラーの場合は`multi_replace_string_in_file`
4. **ファイル破損がひどい場合**: 完全再作成（`rm` → `create_file`）

#### パターン2: モジュールインポートエラー
**症状**: `ModuleNotFoundError: No module named 'lerobot'`

**原因**: LEROBOT_PATH未設定またはLeRobot未インストール

**解決策**:
```bash
# 環境変数確認
echo $LEROBOT_PATH

# 設定
export LEROBOT_PATH="$HOME/study/lerobot/src"
source ~/.bashrc

# LeRobotインストール確認
python3 -c "import lerobot; print(lerobot.__version__)"
```

### 2. 実行時エラー

#### パターン1: トピック未受信
**症状**: `Waiting for observations...`が続く

**診断**:
```bash
# トピック一覧確認
ros2 topic list

# 特定トピックの確認
ros2 topic echo /lekiwi/joint_states
ros2 topic hz /lekiwi/camera/front/image_raw/compressed

# ノード一覧確認
ros2 node list
```

**原因と解決**:
1. **パブリッシャーノード未起動** → `lekiwi_teleop_node`を起動
2. **ROS_DOMAIN_ID不一致** → 両マシンで同じIDに設定
3. **ネットワーク問題** → `ROS_LOCALHOST_ONLY=0`を設定、ファイアウォール確認

#### パターン2: CUDA out of memory
**症状**: RuntimeError: CUDA out of memory

**解決策**:
```bash
# CPUにフォールバック
-p device:=cpu

# AMPを無効化（メモリ節約）
-p use_amp:=false

# 別プロセスを終了してVRAMを解放
pkill python
```

### 3. データセットエラー

#### パターン1: KeyError: 'names'
**症状**: トレーニング時に`dataset_to_policy_features`でエラー

**原因**: 古いメタデータ（画像フィーチャーに`names`なし）

**解決策**:
1. 最新の`lekiwi_data_recorder.py`を使用
2. 既存データセット削除: `rm -rf ~/lerobot_datasets/username/dataset`
3. 再記録

#### パターン2: FileExistsError during dataset creation
**症状**: 新規データセット作成時にエラー

**原因**: 既存ディレクトリが残っている

**解決策**:
```bash
# データセットディレクトリ削除
rm -rf ~/lerobot_datasets/gomi-kuzu/lekiwi_pick_place

# または resume=true で追記モード
ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py resume:=true ...
```

### 4. ネットワーク通信エラー

#### パターン1: 複数マシン間で通信できない
**診断**:
```bash
# Raspberry Pi側のIPを確認
hostname -I

# Desktop PC側からping
ping <raspberry_pi_ip>

# ROS2の発見確認
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**解決策**:
```bash
# 両マシンで同じドメインID
export ROS_DOMAIN_ID=42

# ローカルホスト専用モードを無効化
export ROS_LOCALHOST_ONLY=0

# ファイアウォール無効化（テスト用）
sudo ufw disable
```

---

## テスト戦略

### 1. 単体テスト

#### 環境変数チェックのテスト
```python
# tests/test_environment.py
import os
import pytest

def test_lerobot_path_set():
    """Test that LEROBOT_PATH is set."""
    assert os.environ.get('LEROBOT_PATH') is not None
```

#### ノード初期化テスト
```python
# tests/test_nodes.py
import rclpy
from lekiwi_ros2_teleop.lekiwi_policy_node import LekiwiPolicyNode

def test_policy_node_initialization():
    """Test policy node can be initialized."""
    rclpy.init()
    node = LekiwiPolicyNode()
    assert node is not None
    node.destroy_node()
    rclpy.shutdown()
```

### 2. 統合テスト

#### トピック通信テスト
```bash
# ターミナル1: パブリッシャー起動
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node

# ターミナル2: サブスクライバー確認
ros2 topic hz /lekiwi/joint_states
# 期待: 50 Hz程度

# ターミナル3: コマンド送信
ros2 topic pub /lekiwi/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# 期待: ロボットが前進
```

#### エンドツーエンドテスト（分散システム）
```bash
# 1. Raspberry Pi: ロボット制御起動
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node

# 2. Desktop PC: 推論ノード起動
ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
    policy_path:=<trained_model> \
    dataset_repo_id:=<dataset>

# 3. Desktop PC: 推論開始
ros2 service call /lekiwi/policy/start std_srvs/srv/Trigger

# 期待: ロボットが自律動作
```

### 3. データ検証

#### データセット検証
```bash
# メタデータ確認
cat ~/lerobot_datasets/username/dataset/meta/info.json | jq

# エピソード数確認
ls ~/lerobot_datasets/username/dataset/data/*.parquet | wc -l

# データロードテスト
python3 -c "
from lerobot.common.datasets.factory import make_dataset
dataset = make_dataset('username/dataset', root='~/lerobot_datasets')
print(f'Episodes: {dataset.num_episodes}')
print(f'Frames: {dataset.num_frames}')
"
```

---

## デプロイメント

### 1. 環境セットアップ

#### Raspberry Pi側
```bash
# ROS2 Jazzy インストール
# LeRobot インストール（ロボット制御のみ必要）
# 環境変数設定
export LEROBOT_PATH="$HOME/lerobot/src"
export ROS_DOMAIN_ID=42

# パッケージビルド
cd ~/jazzy_ws
colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash

# 自動起動設定（systemd）
sudo systemctl enable lekiwi-teleop.service
```

#### Desktop PC側
```bash
# ROS2 Jazzy + CUDA + PyTorch インストール
# LeRobot完全インストール
# 環境変数設定
export LEROBOT_PATH="$HOME/study/lerobot/src"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0  # 複数マシン通信用

# パッケージビルド
cd ~/jazzy_ws
colcon build --packages-select lekiwi_ros2_teleop
source install/setup.bash
```

### 2. 起動スクリプト

#### Raspberry Pi起動スクリプト
```bash
#!/bin/bash
# ~/jazzy_ws/scripts/start_robot.sh

source ~/jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args \
  -p robot_port:=/dev/ttyACM0 \
  -p control_frequency:=30.0
```

#### Desktop PC推論起動スクリプト
```bash
#!/bin/bash
# ~/jazzy_ws/scripts/start_policy.sh

source ~/jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
  policy_path:="$HOME/outputs/train/latest/checkpoints/best" \
  dataset_repo_id:=gomi-kuzu/lekiwi_pick_place \
  control_frequency:=30.0 \
  device:=cuda
```

### 3. デバッグツール

#### リモートログ確認
```bash
# Raspberry Pi側のログをDesktop PCで確認
ssh pi@<raspberry_pi_ip> "ros2 topic echo /rosout"

# または journalctl（systemd使用時）
ssh pi@<raspberry_pi_ip> "journalctl -u lekiwi-teleop.service -f"
```

#### パフォーマンスモニタリング
```bash
# トピック周波数
ros2 topic hz /lekiwi/joint_states

# ノードリソース使用
top -p $(pgrep -f lekiwi_policy_node)

# GPUメモリ
nvidia-smi -l 1
```

---

## 開発ワークフロー

### 新機能追加時の手順

1. **要件定義**
   - 既存アーキテクチャとの整合性確認
   - 必要なトピック・サービスの設計
   - パフォーマンス要件の明確化

2. **実装**
   - LeRobot公式実装を参照
   - 既存パターンを継承
   - 段階的に実装（最小動作 → 完全機能）

3. **ドキュメント更新**
   - README.mdに概要追加
   - 専用ドキュメント作成（必要に応じて）
   - コード内Docstring記述

4. **テスト**
   - 単体テスト追加
   - 統合テスト実施
   - 実機テスト

5. **レビュー**
   - コーディング規約準拠確認
   - ドキュメント完全性確認
   - パフォーマンス検証

### バグ修正時の手順

1. **再現確認**
   - エラーメッセージの完全取得
   - 再現手順の文書化
   - ログレベルをDEBUGに設定して詳細確認

2. **原因調査**
   - 関連コードの読解
   - LeRobot公式実装との比較
   - 類似の既知問題を検索

3. **修正実装**
   - 最小限の変更で修正
   - エラーハンドリング追加
   - ログ出力強化

4. **検証**
   - 元の問題が解決されたか確認
   - 副作用がないか確認
   - 複数シナリオでテスト

5. **ドキュメント更新**
   - トラブルシューティングセクションに追加
   - 関連パラメータの説明更新

---

## 参考リソース

### 公式ドキュメント
- **LeRobot**: https://huggingface.co/docs/lerobot
- **ROS2 Jazzy**: https://docs.ros.org/en/jazzy/
- **LeKiwi SDK**: （プロジェクト内ドキュメント参照）

### 重要なLeRobotスクリプト
- `lerobot/scripts/lerobot_record.py` - データ記録の参照実装
- `lerobot/scripts/control_robot.py` - ロボット制御の参照実装
- `lerobot/scripts/train.py` - トレーニングの参照実装

### コミュニティ
- **LeRobot Discord**: https://discord.gg/s3KuuzsPFb
- **GitHub Issues**: 既知の問題と解決策

---

## バージョン管理

### このドキュメントのバージョン
- **Version**: 1.0
- **Last Updated**: 2026-01-17
- **Author**: LeKiwi ROS2 Development Team

### 更新履歴
- **1.0 (2026-01-17)**: 初版作成
  - プロジェクト全体のガイドライン確立
  - 既存コンポーネントのドキュメント化
  - トラブルシューティングパターン集約

---

## まとめ

このドキュメントは、LeKiwi ROS2 Teleoperation プロジェクトの開発を継続するための包括的なガイドラインです。

**最も重要な原則**:
1. **LeRobotとの互換性を最優先**
2. **分散システムアーキテクチャを維持**
3. **段階的実装と早期検証**
4. **包括的なドキュメント維持**

新しい開発者やAIエージェントは、このドキュメントを基準に、既存の品質とポリシーを維持しながら機能拡張を行ってください。

不明点や矛盾がある場合は、既存の実装とLeRobot公式ドキュメントを優先し、必要に応じてこのドキュメントを更新してください。
