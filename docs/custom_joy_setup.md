# カスタムJoyコントローラ設定ガイド

## 概要

teleop_twist_joyでカスタムコントローラを使用するための設定方法です。

## ファイル構成

- **設定ファイル**: `config/custom_joy.config.yaml`
- **Launchファイル**: `launch/custom_teleop.launch.py`

## ステップ1: コントローラのボタンマッピングを確認

まず、あなたのコントローラのボタンと軸のマッピングを確認します。

```bash
# 環境をアクティベート
conda activate leros_jazzy
source /home/inoma/jazzy_ws/install/setup.bash

# joyノードを起動
ros2 run joy joy_node

# 別のターミナルで
conda activate leros_jazzy
source /home/inoma/jazzy_ws/install/setup.bash

# Joyトピックを監視
ros2 topic echo /joy
```

コントローラのボタンやスティックを動かして、どの番号が反応するか確認してください：

- **axes配列**: スティックの値（-1.0 ~ 1.0）
  - 例: axes[0], axes[1] など
- **buttons配列**: ボタンの値（0 or 1）
  - 例: buttons[0], buttons[1] など

## ステップ2: 設定ファイルを編集

`config/custom_joy.config.yaml`を編集して、あなたのコントローラに合わせます：

```yaml
teleop_twist_joy_node:
  ros__parameters:
    # 前後移動に使うスティック軸（縦方向）
    axis_linear:
      x: 1  # ← 確認した軸番号に変更（前後移動）
      y: 0  # ← 確認した軸番号に変更（左右移動）
    
    # 回転に使うスティック軸
    axis_angular:
      yaw: 2  # ← 確認した軸番号に変更
    
    # 移動速度（m/s）
    scale_linear:
      x: 0.1  # 前後移動の最大速度
      y: 0.1  # 左右移動の最大速度
    
    # ターボモード時の移動速度（m/s）
    scale_linear_turbo:
      x: 0.2
      y: 0.2
    
    # 回転速度（rad/s）
    scale_angular:
      yaw: 0.4
    
    # イネーブルボタン（押しながら操作）
    enable_button: 6  # ← 確認したボタン番号に変更
    
    # ターボモードボタン
    enable_turbo_button: 4  # ← 確認したボタン番号に変更
    
    # イネーブルボタンを必須とするか
    require_enable_button: false  # ← true にすると、enable_buttonを押しながらでないと動かない
```

### 主要なパラメータ

| パラメータ | 説明 | 推奨値 |
|-----------|------|--------|
| `axis_linear.x` | 前後移動に使う軸番号 | 通常1（左スティック縦） |
| `axis_linear.y` | 左右移動に使う軸番号 | 通常0（左スティック横） |
| `axis_angular.yaw` | 回転に使う軸番号 | 通常2（右スティック横） |
| `enable_button` | イネーブルボタンの番号 | コントローラに応じて |
| `enable_turbo_button` | ターボモードボタンの番号 | コントローラに応じて |
| `require_enable_button` | イネーブルボタンを必須とするか | false（簡単）/true（安全） |
| `scale_linear.x` | 最大前後移動速度 (m/s) | 0.1 |
| `scale_linear.y` | 最大左右移動速度 (m/s) | 0.1 |
| `scale_linear_turbo.x` | ターボ時の前後移動速度 (m/s) | 0.2 |
| `scale_linear_turbo.y` | ターボ時の左右移動速度 (m/s) | 0.2 |
| `scale_angular.yaw` | 最大回転速度 (rad/s) | 0.4 |
| `scale_angular_turbo.yaw` | ターボ時の回転速度 (rad/s) | 0.8 |

## ステップ3: カスタム設定で起動

```bash
conda activate leros_jazzy
source /home/inoma/jazzy_ws/install/setup.bash

# カスタム設定で起動
ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py
```

## ステップ4: 動作確認

別のターミナルで`/lekiwi/cmd_vel`トピックを確認：

```bash
conda activate leros_jazzy
source /home/inoma/jazzy_ws/install/setup.bash

# cmd_velトピックを監視
ros2 topic echo /lekiwi/cmd_vel
```

コントローラを操作して、以下の値が変化することを確認してください：
- `linear.x`: 前後移動速度
- `linear.y`: 左右移動速度（LeKiwiは全方向移動可能）
- `angular.z`: 回転速度

## 参考: 他のコントローラ設定例

標準的なコントローラ設定は以下にあります：

```bash
ls /home/inoma/miniforge3/envs/leros_jazzy/share/teleop_twist_joy/config/
```

- `ps3.config.yaml` - PlayStation 3コントローラ
- `ps5.config.yaml` - PlayStation 5コントローラ
- `xbox.config.yaml` - Xboxコントローラ
- `atk3.config.yaml` - ATK3コントローラ

これらを参考にすることもできます。

## トラブルシューティング

### `/cmd_vel`が流れない

1. **イネーブルボタンの確認**
   ```bash
   ros2 param get /teleop_twist_joy_node require_enable_button
   ```
   - `true`の場合、`enable_button`を押しながら操作する必要があります
   - 一時的に無効化: `ros2 param set /teleop_twist_joy_node require_enable_button false`

2. **軸マッピングの確認**
   ```bash
   ros2 param get /teleop_twist_joy_node axis_linear.x
   ros2 param get /teleop_twist_joy_node axis_angular.yaw
   ```

3. **joyトピックの確認**
   ```bash
   ros2 topic echo /joy
   ```
   スティックを動かして値が変化するか確認

### 動きが逆向き

軸の符号を反転させる必要があります。設定ファイルで軸番号を負の値にします：

```yaml
axis_linear:
  x: -1  # 軸1の符号を反転（前後が逆の場合）
  y: -0  # 軸0の符号を反転（左右が逆の場合）

axis_angular:
  yaw: -2  # 軸2の符号を反転（回転方向が逆の場合）
```

### 速度を調整したい

`scale_linear`、`scale_linear_turbo`、`scale_angular`の値を調整してください：

```yaml
scale_linear:
  x: 0.05  # 前後移動をより遅く
  y: 0.05  # 左右移動をより遅く

scale_linear_turbo:
  x: 0.3  # ターボ時はより速く
  y: 0.3

scale_angular:
  yaw: 0.8  # 回転をより速く

scale_angular_turbo:
  yaw: 1.2  # ターボ時の回転をより速く
```
