# Jetson の物体検出（NanoOWL）

AI-CAR のカメラ画像を Jetson Orin Nano Super で NanoOWL（言葉で物を探す AI）にかけ、
「段ボール箱」「袋」のような名前と画面の位置を AI-CAR へ送ります。AI-CAR の YOLOv8m（80 種類）に
ない物も、`prompts.json` に英語の言葉を足せば探せます。

```
AI-CAR  GET /api/camera/snapshot ──→ Jetson（NanoOWL、約 35 ms / 枚）
AI-CAR  POST /api/jetson/detections ←── 名前・信頼度・位置（重なった枠は 1 つにまとめる）
        → /jetson_detections → perception_node が LiDAR の距離を付ける → ダッシュボードに水色の枠
```

表示だけで、AI-CAR の速度（停止・減速）には使いません。

## ファイル
- `owl_detector.py`: 本体。`dustynv/nanoowl:r36.3.0` コンテナの中で動かす
- `prompts.json`: 探す言葉（`prompt`、英語）と表示する名前（`label`）。`label` が空の言葉（窓・カーテン）は、
  まぎらわしい枠を吸わせるためだけに使い、送らない
- `jetson-owl.service`: Jetson の systemd サービス

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
sudo docker pull dustynv/nanoowl:r36.3.0
mkdir -p ~/owl/hf
# このフォルダを ~/owl_detector にコピーする
sudo install -m 600 /dev/null /etc/jetson-owl.env
sudoedit /etc/jetson-owl.env   # 次の 2 行を書く
#   AI_CAR_URL=http://100.70.35.31:8080
#   AI_CAR_API_TOKEN=<AI-CAR の dashboard_node の API トークン>
sudo cp ~/owl_detector/jetson-owl.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-owl.service
journalctl -u jetson-owl -f
```

1 枚だけ試すとき（結果を表示するだけで、AI-CAR には送らない）:
```bash
sudo docker run --rm --runtime nvidia --network host \
  -v ~/owl_detector:/owl_detector:ro -v ~/owl/hf:/root/.cache/huggingface \
  dustynv/nanoowl:r36.3.0 python3 /owl_detector/owl_detector.py --once
```

## 分かっていること（2026/10 実機）
- 960 × 540 の画像 1 枚で約 35 ms（言葉 4〜5 個）
- 袋 0.69、段ボール箱 0.62 で見つかった。白い箱も見つかるが「段ボール箱」（0.50）になる。
  「a white box」「a box」では段ボール箱より低い点しか出ず、箱の色の言い分けは苦手
- 何もしないと 1 つの物に枠が何個も重なるので、重なり（IoU）0.5 以上の枠と、80 % 以上がほかの枠の中に入る枠は捨て、点の高い 1 つだけ残す
