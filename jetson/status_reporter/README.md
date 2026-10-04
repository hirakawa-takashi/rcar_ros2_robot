# Jetson の状態を AI-CAR のダッシュボードに出す

Jetson Orin Nano Super の温度・使用率・メモリ・電力・AI の部品（声の AI・物体検出）・
セキュリティ更新の状況を、1 秒ごとに AI-CAR へ送ります。ダッシュボードの
「Jetson Orin Nano Super」カードに出ます。

```
Jetson  /proc・/sys・systemctl・HTTP（whisper 8178 / Ollama 11434 / Kokoro 8880）
        → POST /api/jetson/status（API トークン）→ AI-CAR の dashboard_node → /api/status の jetson_host
```

5 秒以上届かないと、カードは「未接続」になります。表示だけで、AI-CAR の速度には使いません。

## 送るもの
- 温度（CPU・GPU・Tj）、CPU / GPU の使用率とクロック、ファン
- メモリ・スワップ・ディスク、ボード全体の入力電圧と消費電力（INA3221 の `VDD_IN`）、電源モード（`nvpmodel -q`）
- AI の部品が動いているか: `whisper-server`（声→文字。`whisper-server.service` の `-m` から仕組みの名前 `stt_engine`、例: whisper.cpp small（GPU））、`ollama`（会話の AI、読み込み中のモデルと、その大きさ・GPU に載っている量）、
  Kokoro（文字→声、Docker。`/openapi.json` の題名と版から `tts_engine`、例: Kokoro TTS 1.0.0）、`jetson-owl`（NanoOWL の物体検出）
- 更新: 残りの更新とセキュリティ更新の数（`apt-check`、重いので 1 時間に 1 回と、ボタンの更新のあと）、再起動が必要か、
  最後に自動更新（unattended-upgrades）が動いた時刻。残りの更新の数には、NVIDIA の部品（JetPack: `nvidia-`・`cuda-`・
  `libcudnn`・`libnvinfer`・`tensorrt`）を入れない（その数は `updates_held`）。ボタンの更新が動いているかと、前回の結果
- IP アドレス（Tailscale・Wi-Fi など。lo・Docker・USB の `l4tbr0` は出さない。30 秒ごとに調べる）、再起動ボタンが使えるか（`sudo -n -l`）

## ファイル
- `jetson_status.py`: 本体。標準ライブラリだけで動く（Jetson の `/usr/bin/python3`）
- `jetson-status.service`: Jetson の systemd サービス（ユーザー `jetson` で動かす）
- `jetson-upgrade.sh`・`jetson-upgrade.service`・`jetson-upgrade.sudoers`: ダッシュボードの「Jetson を更新」ボタン用

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

## ダッシュボードの「再起動」「電源を切る」ボタン
Jetson カードの「Jetson を再起動」を押すと、AI-CAR は次に状態が届いたときの返事に `"reboot": true` を入れます。
`jetson_status.py` はそれを見て `sudo -n systemctl reboot` を実行します。「Jetson の電源を切る」は `"poweroff": true` で、`sudo -n systemctl poweroff` を実行します（入れ直すのは本体の電源ボタンか電源のつなぎ直し）。パスワードなしで再起動と電源を切ることだけを許す設定が要ります:
```bash
sudo visudo -cf ~/status_reporter/jetson-reboot.sudoers
sudo install -m 440 ~/status_reporter/jetson-reboot.sudoers /etc/sudoers.d/jetson-reboot
```
入っていないときはボタンを押しても「jetson-reboot.sudoers が入っていません」（電源は「… に poweroff が入っていません」）と出て、何もしません。

## ダッシュボードの「Jetson を更新」ボタン
自動の更新（unattended-upgrades）はセキュリティ更新だけを入れます。ほかの更新は、ボタンを押したときだけ入れます。
押すと、AI-CAR は次の返事に `"upgrade": true` を入れ、`jetson_status.py` が `sudo -n systemctl start --no-block jetson-upgrade.service` を実行します。
`jetson-upgrade.sh`（root）は `apt-get update` のあと、NVIDIA の部品を除いて `apt-get install --only-upgrade` します。
先に試しに動かして（`apt-get -s`）、NVIDIA の部品が変わるか、消える部品があるときは、更新せずに失敗にします。
動いているあいだはカードに「更新中 45 %」のように進み具合が出ます（`jetson-upgrade.sh` が `/run/jetson-upgrade.progress` に書く。ダウンロードが 0〜30 %、インストールが 30〜100 %）。終わると「前回は成功」か「前回は失敗」と出ます。記録は `journalctl -u jetson-upgrade` で見られます。
```bash
sudo install -m 755 ~/status_reporter/jetson-upgrade.sh /usr/local/sbin/jetson-upgrade
sudo cp ~/status_reporter/jetson-upgrade.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo visudo -cf ~/status_reporter/jetson-upgrade.sudoers
sudo install -m 440 ~/status_reporter/jetson-upgrade.sudoers /etc/sudoers.d/jetson-upgrade
sudo systemctl restart jetson-status.service
```

1 回だけ集めて表示するとき（AI-CAR には送らない）:
```bash
python3 ~/status_reporter/jetson_status.py --once
```
