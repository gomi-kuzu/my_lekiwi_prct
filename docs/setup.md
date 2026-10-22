# セットアップガイド

`lekiwi_ros2_teleop` を動かすための環境構築・デバイス設定・環境変数のまとめ。
ビルドとクイックスタートは [README](../README.md) を参照すること。

---

## 1. インストール

### ROS2 Jazzy

ROS2 Jazzy をインストールする。公式のネイティブインストールのほか、
[RoboStack](https://robostack.github.io/GettingStarted.html) で conda / micromamba 環境に
導入する方法でもよい（RoboStack を使う場合は、以降のコマンドを環境をアクティベートした状態で実行する）。

### LeRobot

本パッケージは [LeRobot](https://github.com/huggingface/lerobot) ライブラリ（**v0.6.2 以上**）に依存する。

```bash
# リポジトリをクローン
git clone https://github.com/huggingface/lerobot.git ~/lerobot

# 動画エンコード用に ffmpeg を用意（ディストリ標準の apt でも可:sudo apt install ffmpeg）

cd ~/lerobot
pip install -e ".[core_scripts,lekiwi,feetech]"
pip install -e ".[smolvla]"
pip install -e ".[dataset,training]"
```

インストール後、`LEROBOT_PATH` 環境変数を設定する（[§3](#3-環境変数)）。

---

## 2. カメラデバイスパスの確認と設定（必須）

カメラの `/dev/v4l/by-path/` パスは **USB ポートの物理的な接続位置に依存する**。
カメラを別ポートに挿し直したり、ラズパイを再起動するとパスが変わる場合がある。
初回セットアップ時、および接続を変更した際は必ず以下を確認すること。

### ステップ1: 接続されているカメラのパスを確認

```bash
# by-path の一覧（カメラデバイスのみ）
ls -la /dev/v4l/by-path/ | grep video-index0

# USB トポロジーの確認（バス帯域の確認に有用）
lsusb -t
```

出力例:

```
lrwxrwxrwx ... platform-xhci-hcd.1-usb-0:2:1.0-video-index0 -> ../../video0
lrwxrwxrwx ... platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0 -> ../../video2
```

### ステップ2: ノードのパラメータを設定

`lekiwi_teleop_node` 起動時にカメラデバイスを指定する:

```bash
ros2 run lekiwi_ros2_teleop lekiwi_teleop_node \
  --ros-args \
  -p robot_port:=/dev/ttyACM0 \
  -p front_camera_device:='/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0,/dev/video0' \
  -p wrist_camera_device:='/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0,/dev/video2'
```

> **注意**: カンマ区切りで複数のパスを指定できる。最初のパスで失敗した場合、次のパスが試される（フォールバック）。

### USB 帯域の注意事項

2 台のカメラを同じ USB バスに接続すると、**帯域不足でフレームタイムアウトが発生**する場合がある。

```
[ERROR] Timed out waiting for frame from camera OpenCVCamera(...) after 200 ms.
```

対処法:

- 2 台のカメラを**異なる USB コントローラー**（例 `xhci-hcd.0` と `xhci-hcd.1`）に分けて接続する。
- `lsusb -t` でバス構成を確認し、カメラが異なる `Bus` に接続されていることが望ましい。

---

## 3. 環境変数

### `LEROBOT_PATH`（必須）

LeRobot の `src` ディレクトリへのパス。シェル設定ファイル（`~/.bashrc` 等）に追記する:

```bash
export LEROBOT_PATH="/path/to/your/lerobot/src"
```

### `LEKIWI_REMOTE_IP`（`lekiwi_ros2_teleop_client` で必須）

テレオペレーションクライアントを使う場合、LeKiwi の IP アドレスを設定する:

```bash
export LEKIWI_REMOTE_IP="<LEKIWI_IP>"   # 例: LEKIWI_REMOTE_IP="172.18.11.100"
```

> **なぜ必須か**: このIPはROS2トピックの通信（同一ネットワークなら DDS が自動検出するため不要）には使われない。
> クライアントは LeRobot の `LeKiwiClient` を通じて、ラズパイ側の LeKiwi host に **ZeroMQ で直接接続**する。
> ZeroMQ は接続先ホストを明示的に指定する必要があるため、IP の設定が必須となる。

設定を反映:

```bash
source ~/.bashrc   # 使用シェルに合わせて
```

> **注意**: 環境変数が未設定だと、各ノードは起動時にエラーを出力する。

---

## 関連ドキュメント

- ノード・トピック・パラメータのリファレンス: [nodes.md](nodes.md)
- ジョイスティック設定: [custom_joy_setup.md](custom_joy_setup.md)
- データパイプライン（収集 → 変換）: [data_pipeline.md](data_pipeline.md)
- ポリシー推論: [policy_inference.md](policy_inference.md)
