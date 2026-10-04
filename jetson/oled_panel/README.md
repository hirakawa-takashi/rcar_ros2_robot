# Jetson のケースの OLED と RGB の光

Yahboom の Jetson ケース（CUBE NANO）の横の小さな画面（OLED、SSD1306 128×32、白 1 色）に、Jetson の状態を出します。
Yahboom のイメージではなく JetPack をそのまま入れた Jetson では、この画面は何も出ないので、そのかわりです。

大きな文字（DejaVu Sans Bold 14 px）の 2 行で、3 秒ごとに画面を切り替えます（値は 1 秒ごとに新しくします）。
```
CPU   12% 45°C      CPU・GPU の負荷率と温度
GPU    3% 44°C
---
RAM    3.1/7.4G     メモリ（使用 / 全部、GiB）と / のディスク（GB）
SSD    120/866G
---
Wi-Fi               IP アドレス（Tailscale・Wi-Fi・LAN を 1 つずつ）
     192.168.11.23
```

ケースの RGB の光（I2C 0x0E、Yahboom の「I2C Communication Protocol」の 0x04 特効・0x05 速さ・0x06 色）は、
AI の負荷（GPU の負荷率の 5 秒平均）で、呼吸（特効 1）の色と速さを変えます。変わったときだけ書き、`journalctl -u jetson-oled` に出します。

| GPU の負荷率 | 色 | 速さ |
| --- | --- | --- |
| 10 % より下 | 青 | 低速 |
| 10〜40 % | 緑 | 中速 |
| 40〜70 % | 黄 | 中速 |
| 70 % 以上 | 赤 | 高速 |

- CPU が 70 °C 以上（`OLED_HOT_C` で変えられる）のときは、負荷にかかわらず赤の高速
- ファン（0x08）は変えません

- I2C の 7 番（40 ピンの 3・5 番）。`i2cdetect -y -r 7` で `3c`（OLED）と `0e`（ケースの基板）が見えれば、つながっています
- 標準ライブラリと Pillow だけで動きます（JetPack の `/usr/bin/python3`）。CPU は約 2 %、メモリ約 16 MB
- サービスを止めると画面を消します（RGB の光はそのまま）

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
# このフォルダを ~/oled_panel にコピーする（ユーザー jetson は i2c グループに入っている）
sudo cp ~/oled_panel/jetson-oled.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-oled.service
journalctl -u jetson-oled -f
```
