# LeKiwi クラウド学習ガイド（SmolVLA）

このドキュメントは、LeKiwi用にSmolVLAを転移学習するときの参考文書。
lerobotライブラリの `lerobot-train` コマンドの解説と、実際にハマったポイント・Tips をまとめたもの。

---

## 目次

1. [データ・モデルの受け渡し方針](#1-データモデルの受け渡し方針)
2. [最終的に使った学習コマンド](#2-最終的に使った学習コマンド)
3. [オプション解説](#3-オプション解説)
4. [最重要: input_features は手で指定しない](#4-最重要-input_features-は手で指定しない)
5. [ベース config の作り方（smolvla_base_lekiwi2cam）](#5-ベース-config-の作り方smolvla_base_lekiwi2cam)
6. [rename_map の役割](#6-rename_map-の役割)
7. [HuggingFace ログインの要否](#7-huggingface-ログインの要否)
8. [学習後の検証](#8-学習後の検証)
9. [Tips / トラブルシューティング](#9-tips--トラブルシューティング)

---

## 1. データ・モデルの受け渡し方針

クラウド学習では「何を HuggingFace 経由にするか」を先に決めておく。
本プロジェクトの運用方針は以下。

| 成果物 | 受け渡し経路 | HuggingFace 認証 |
| --- | --- | --- |
| ベース config + 重み（`smolvla_base_lekiwi2cam`, **private**） | HuggingFace  | **read トークン必須** |
| 学習用データセット | Google Drive | 不要 |
| 学習済みモデル（出力） | Google Drive（`push_to_hub=false`） | 不要（write 不要） |
| VLM バックボーン（`lerobot/smolvla_base`, public） | HuggingFace Hub（自動 DL） | 不要（public） |

- config だけ HuggingFace 経由
- **データセットと学習済みモデルは Google Drive** に置くことにする。

---

## 2. 最終的に使った学習コマンド

Colab 等のノートブックセルで実行した実際のコマンド（`!` はノートブックのシェル実行プレフィックス）。

```bash
!rm -rf ./outputs/train/smolvla_lekiwi_PandP_retry
!lerobot-train \
  --dataset.repo_id=lekiwi_PandP_retry \
  --policy.path=<hf_user_name>/smolvla_base_lekiwi2cam \
  --output_dir=outputs/train/smolvla_lekiwi_PandP_retry \
  --job_name=261019_smolVLA_lekiwi_RePandP \
  --steps=30000 --save_freq=5000 --batch_size=32 \
  --policy.chunk_size=64 --policy.device=cuda \
  --policy.push_to_hub=false \
  --wandb.enable=true \
  --wandb.disable_artifact=true \
  --rename_map='{"observation.images.front":"observation.images.camera1","observation.images.wrist":"observation.images.camera2"}'
```

> **注意**: `--dataset.repo_id=lekiwi_PandP_retry` はローカル（`$HF_HOME` 以下、または作業ディレクトリ基準）に
> 置いた Google Drive 由来のデータセットを指す。HuggingFace 上の同名リポジトリを探しに行かせたくない場合は、
> データセットの実体パスが正しく解決されることを事前に確認する（[§9](#9-tips--トラブルシューティング)）。

---

## 3. オプション解説

LeKiwi クラウド学習で特に意味を持つオプションのみ抜粋（汎用オプションは
[lerobot-train コマンド使用方法](../../../docs/lerobot-train-usage.md) 参照）。

| オプション | 意味・LeKiwi での使い方 |
| --- | --- |
| `--policy.path=<hf_user_name>/smolvla_base_lekiwi2cam` | **正しい入出力定義（2 カメラ・state[9]・action[9]）を持つベース config** を指定して Fine-Tuning。`--policy.type` は併用しない。 |
| `--dataset.repo_id=lekiwi_PandP_retry` | 学習データセット。ローカルにあればそれを使用、無ければ HuggingFace を探しに行く。 |
| `--output_dir` | 出力先。再実行時は衝突を避けるため直前に `rm -rf` している。 |
| `--job_name` | wandb 上のジョブ名。日付＋内容で一意にする。 |
| `--steps=30000 --save_freq=5000` | 3 万ステップ、5000 ごとにチェックポイント保存。 |
| `--batch_size=32` | クラウド GPU の VRAM に合わせて調整（OOM が出たら下げる）。 |
| `--policy.chunk_size=64` | アクションチャンク長。推論時の `n_action_steps` はこれ以下に設定する。 |
| `--policy.device=cuda` | GPU 学習。 |
| `--policy.push_to_hub=false` | 学習済みモデルを HuggingFace に上げない（Google Drive 運用のため）。 |
| `--wandb.enable=true` | 学習曲線を wandb で監視。 |
| `--wandb.disable_artifact=true` | **wandb にモデル成果物をアップロードしない**。通信量・ストレージ節約。チェックポイントはローカル（Drive）に残る。 |
| `--rename_map='{...}'` | **データセットのカメラ名**（`front`/`wrist`）を**モデルが期待する名前**（`camera1`/`camera2`）へ対応づける。 |

---

## 4. 重要: input_features は手で指定しない

最初、`--policy.input_features='{...}'` を手で指定して、学習後の推論がうまく回せなかった。

### 何が起きたか

- draccus（config パーサ）の `--policy.input_features='{...}'` は、ベース config の既存
  `input_features` を**置き換えるのではなく、辞書を追加マージ（additive merge）**する。
- その結果、`smolvla_base` デフォルトの `camera3` と `state[6]` が残り、
  `camera1`/`camera2` だけ指定したつもりが **「camera1,2,3 + state[6]」** に汚染された。

### 正しいやり方

- `--policy.input_features` は**書かない**。
- 代わりに **ベース config 自体を正しい内容**（2 カメラ・state[9]・camera3 なし）で用意し、
  `--policy.path` で渡す（[§5](#5-ベース-config-の作り方smolvla_base_lekiwi2cam)）。
- データセットのカメラ名のズレは [`--rename_map`](#6-rename_map-の役割) で吸収する。

> 教訓: **SmolVLA の入出力定義はコマンドラインではなくベース config で管理する。**

---

## 5. ベース config の作り方（smolvla_base_lekiwi2cam）

`lerobot/smolvla_base` のデフォルト config は 3 カメラ・state[6] を前提としており、
LeKiwi（2 カメラ・台車速度込み state[9]）と合わない。そこで、**重みはそのまま／config だけ LeKiwi 用に
書き換えた**ベースを作り、HuggingHub に private で上げておく。

### 手順の要点

1. `lerobot/smolvla_base` のスナップショットをローカルへ実体コピー（`cp -rL` でシンボリックリンクを実体化）。
2. `config.json` の `input_features` / `output_features` / `empty_cameras` を LeKiwi 用に編集。
3. `model.safetensors`（重み）は**変更しない**。
4. HuggingFace に private でアップロード（`hf upload`）。

### 目標の config.json（抜粋）

```jsonc
{
  "type": "smolvla",
  "input_features": {
    "observation.state":          { "type": "STATE",  "shape": [9] },
    "observation.images.camera1": { "type": "VISUAL", "shape": [3, 480, 640] },
    "observation.images.camera2": { "type": "VISUAL", "shape": [3, 640, 480] }
  },
  "output_features": {
    "action": { "type": "ACTION", "shape": [9] }
  },
  "empty_cameras": 0
}
```

- `observation.state` の `[9]` = アーム 6 自由度 + 台車速度 3 次元（`x.vel`, `y.vel`, `theta.vel`）。
- `camera3` が**無い**こと、`empty_cameras` が `0` であることを必ず確認。

### HuggingFaceへのアップロード

```bash
hf upload <hf_user_name>/smolvla_base_lekiwi2cam \
  models/smolvla_base_lekiwi2cam . \
  --repo-type=model --private
```

> `huggingface-cli` は非推奨。`hf` コマンドを使う。403 が出る場合は write トークンで再ログイン。

---

## 6. rename_map の役割

データセットのカメラ名（LeKiwi 収録は `front` / `wrist`）と、モデル config が期待する名前
（`camera1` / `camera2`）が異なるため、`--rename_map` で対応づける。

```bash
--rename_map='{"observation.images.front":"observation.images.camera1","observation.images.wrist":"observation.images.camera2"}'
```

| データセット側 | → | モデル config 側 |
| --- | --- | --- |
| `observation.images.front` | → | `observation.images.camera1` |
| `observation.images.wrist` | → | `observation.images.camera2` |

- **学習時**と**推論時**の両方で同じ `rename_map` を使う（推論ノードは `rename_map` パラメータで指定）。
- これを忘れると `Feature mismatch ... Missing: camera1,camera2 / Extra: front,wrist` エラーになる。

---

## 7. HuggingFace ログインの要否

ベース config を **private** で置いているため、クラウド側でも**ダウンロードに認証が必要**。

```bash
hf auth login --token hf_xxxxxxxx   # read 権限でよい
```

- `--policy.push_to_hub=false` なので **write トークンは不要**、**read トークンで十分**。
- 認証を完全に避けたい場合の代替:
  - ベースを **public** にする（config・重みを公開してよい場合）、または
  - ベースも Google Drive 経由にして `--policy.path=<ローカルパス>` + `HF_HUB_OFFLINE=1`。
- VLM バックボーン `lerobot/smolvla_base` は public なので認証不要。ただしオフライン環境では事前キャッシュが必要。

---

## 8. 学習後の検証

チェックポイントの `pretrained_model/config.json` を開き、以下を**必ず**確認する。

- [ ] `observation.state` の `shape` が `[9]`
- [ ] `observation.images.camera1` と `observation.images.camera2` が存在
- [ ] `observation.images.camera3` が**存在しない**
- [ ] `output_features.action` の `shape` が `[9]`
- [ ] `empty_cameras` が `0`
- [ ] `pretrained_path` が `<hf_user_name>/smolvla_base_lekiwi2cam`

推論ノードでの読み込み確認は [policy_inference.md](policy_inference.md) を参照。起動ログに
`Policy loaded successfully!` と `Policy requires image features: ['...camera1', '...camera2']` が出れば成功。

---

## 9. Tips / トラブルシューティング

### lerobot のバージョンを揃える

- 学習（クラウド）と推論（ローカル）で **lerobot のコミットを一致**させる。
- `__version__` の文字列は git の状態を反映しないため、`git -C <lerobot> describe` / `git log -1` で実体を確認する。
- config に新フィールド（例: `dtype`）が増えると、古い lerobot 側で
  `The fields dtype are not valid for SmolVLAConfig` のようなロードエラーになる。

### データセットのパス解決

- `--dataset.repo_id` がローカルに見つからないと HuggingFace を探しに行く。Google Drive から DL したデータセットを使うときは、
  `$HF_HOME` / 作業ディレクトリ基準で正しい場所に展開されているか確認する。
- 推論側は `{dataset_root}/{dataset_repo_id}` で解決する（[policy_inference.md](policy_inference.md) 参照）。

### `Unknown fields in DatasetInfo: ['x_lekiwi']` という警告

- **無害**。LeKiwi 独自メタデータ（プロヴェナンス）を lerobot が無視しているだけ。
  詳細は [data_pipeline.md](data_pipeline.md) の `x_lekiwi` 設計を参照。

### OOM（CUDA out of memory）

- `--batch_size` を下げる（32 → 16 → 8）。
- `--policy.chunk_size` を下げると系列長が短くなりメモリも減るが、推論時の `n_action_steps` 上限も下がる点に注意。

### wandb の容量を抑える

- `--wandb.disable_artifact=true` でモデル成果物の wandb アップロードを無効化。学習曲線のログは残る。

### 再実行時の出力衝突

- 同じ `--output_dir` に再実行するとエラーになることがあるため、コマンド冒頭で `rm -rf ./outputs/train/<job>` している。
  **消す前に必要なチェックポイントを Google Drive に退避**しておくこと。

---

## 参考

- [lerobot-train コマンド使用方法](../../../docs/lerobot-train-usage.md) — 汎用的な学習・マルチ GPU・Resume
- [policy_inference.md](policy_inference.md) — 学習済みモデルでの推論ノード起動
- [data_pipeline.md](data_pipeline.md) — データ収集〜フォーマッティング（`x_lekiwi` メタデータ設計）
