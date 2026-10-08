# power_saving（Jetson の省電力を切る）

Jetson が置いたままで止まる（ping は返るが SSH・AI・状態の送信が止まる）ことの対策として、2 つの省電力を切る。

| 設定 | 前 | 後 | 効き始め |
|---|---|---|---|
| SSD（MAXIO MAP1202）の APST | 0.1 秒使わないと PS3（0.75 W、起きるのに 10 ms） | 切（`nvme_core.default_ps_max_latency_us=0`） | 次の起動 |
| Wi-Fi（RTL8822CE）の省電力 | `wifi.powersave = 3`（オン） | `2`（切） | すぐ（Wi-Fi が数秒切れる） |

- 2026-10 の調べ: SSD は電源を入れた 20 回のうち 9 回が unsafe shutdown、読み書きのエラーは 0。Wi-Fi は 3 日 17 時間で 16 回切れていた（電波 -45 dBm）。止まった原因かどうかは、まだわかっていない（止まったときの記録は `../health_log` で取る）。
- SSD が起きたままなので、電気が少し増える（見込みで 1 W 弱）。

## 入れ方

```bash
sudo ./install.sh
sudo reboot
```

確かめ方:

```bash
cat /sys/module/nvme_core/parameters/default_ps_max_latency_us   # 0
sudo nvme get-feature /dev/nvme0 -f 0x0c -H | head -2            # APSTE: Disabled
iw dev wlP1p1s0 get power_save                                    # Power save: off
```

もとに戻す: `/boot/extlinux/extlinux.conf.orig-apst` を戻し、`/etc/NetworkManager/conf.d/zz-ai-car-wifi-powersave-off.conf` を消して再起動する。JetPack の更新で `extlinux.conf` が書き直されたら、`install.sh` をもう一度実行する。
