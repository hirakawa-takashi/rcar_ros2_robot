# Jetson の状態を AI-CAR のダッシュボードに出す

Jetson Orin Nano Super の温度・使用率・メモリ・電力・AI の部品（声の AI・物体検出）・
セキュリティ更新の状況を、5 秒ごとに AI-CAR へ送ります。ダッシュボードの
「Jetson Orin Nano Super」カードに出ます。

```
Jetson  /proc・/sys・systemctl・HTTP（whisper 8178 / Ollama 11434 / Kokoro 8880）
        → POST /api/jetson/status（API トークン）→ AI-CAR の dashboard_node → /api/status の jetson_host
```

15 秒以上届かないと、カードは「未接続」になります。表示だけで、AI-CAR の速度には使いません。

## 送るもの
- 温度（CPU・GPU・Tj）、CPU / GPU の使用率とクロック、ファン
- メモリ・スワップ・ディスク、ボード全体の入力電圧と消費電力（INA3221 の `VDD_IN`）、電源モード（`nvpmodel -q`）
- AI の部品が動いているか: `whisper-server`（声→文字）、`ollama`（会話の AI、読み込み中のモデルと、その大きさ・GPU に載っている量）、
  Kokoro（文字→声、Docker）、`jetson-owl`（NanoOWL の物体検出）
- 更新: 残りの更新とセキュリティ更新の数（`apt-check`、重いので 1 時間に 1 回）、再起動が必要か、
  最後に自動更新（unattended-upgrades）が動いた時刻

## ファイル
- `jetson_status.py`: 本体。標準ライブラリだけで動く（Jetson の `/usr/bin/python3`）
- `jetson-status.service`: Jetson の systemd サービス（ユーザー `jetson` で動かす）

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
# このフォルダを ~/status_reporter にコピーする
sudo install -m 600 /dev/null /etc/jetson-status.env
sudoedit /etc/jetson-status.env   # 次の 2 行を書く（jetson-owl と同じ値）
#   AI_CAR_URL=http://100.70.35.31:8080
#   AI_CAR_API_TOKEN=<AI-CAR の dashboard_node の API トークン>
sudo cp ~/status_reporter/jetson-status.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-status.service
journalctl -u jetson-status -f
```

1 回だけ集めて表示するとき（AI-CAR には送らない）:
```bash
python3 ~/status_reporter/jetson_status.py --once
```
