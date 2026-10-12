# Jetson の記録（止まったときの原因を調べるため）

放っておくだけで Jetson が止まる（ping は返るが SSH・Tailscale・状態の送信が止まる）ことが 2 回あった。
前の起動の記録がのこらない設定だったので、原因がわからなかった。そのための 2 つ。

- **journal をのこす**: `journald-persistent.conf` を `/etc/systemd/journald.conf.d/persistent.conf` に入れ、
  記録を `/var/log/journal` に書く（最大 1 GB）。再起動しても `journalctl -b -1` で前の起動の記録を読める
- **1 分ごとの様子**: `jetson-health.timer` が `jetson_health.sh` を 1 分ごとに動かし、1 行を journal に出す
  ```
  load 0.51/0.53 mem_avail 1360M free 104M swap 164M tj 47C fan 193/255 4431rpm down: none top: uvicorn:1418M ollama:469M python3:155M
  ```
  `down:` は止まっているサービス（ssh・tailscaled・jetson-status・ollama・whisper-server・jetson-face・jetson-oled）

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
# このフォルダを ~/health_log にコピーする
sudo mkdir -p /etc/systemd/journald.conf.d /var/log/journal
sudo cp ~/health_log/journald-persistent.conf /etc/systemd/journald.conf.d/persistent.conf
sudo systemd-tmpfiles --create --prefix /var/log/journal
sudo systemctl restart systemd-journald
sudo journalctl --flush        # /run の記録を /var/log/journal へ移す（しないと次の起動まで /run のまま）
sudo cp ~/health_log/jetson-health.service ~/health_log/jetson-health.timer /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-health.timer
```

## 止まったあとに見る
```bash
journalctl -b -1 -u jetson-health --no-pager | tail -20      # 止まる前の 1 分ごとの様子
journalctl -b -1 -k --no-pager | grep -i -E "oom|killed process|hung|blocked for|watchdog|panic"
journalctl -b -1 --no-pager | tail -50                       # 最後に何が起きたか
```
