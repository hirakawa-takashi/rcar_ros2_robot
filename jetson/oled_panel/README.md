# Jetson のケースの OLED に状態を出す

Yahboom の Jetson ケース（CUBE NANO）の横の小さな画面（OLED、SSD1306 128×32）に、Jetson の状態を 1 秒ごとに出します。
Yahboom のイメージではなく JetPack をそのまま入れた Jetson では、この画面は何も出ないので、そのかわりです。

```
CPU: 12%  45.3C     CPU の負荷率と温度（cpu-thermal）
RAM:3.1/7.4G        メモリ（使用 / 全部、GiB）
SSD:120/866G        / のディスク（使用 / 全部、GB）
IP:100.111.231.54   IP アドレス（Tailscale を先に、5 秒ごとに切り替え）
```

- I2C の 7 番（40 ピンの 3・5 番）、アドレス 0x3C。`i2cdetect -y -r 7` で `3c` が見えれば、つながっています（`0e` はケースの基板のファンと RGB）
- 標準ライブラリと Pillow だけで動きます（JetPack の `/usr/bin/python3`）。CPU は約 2 %、メモリ約 16 MB
- ファンと RGB の光（0x0E）は変えません
- サービスを止めると画面を消します

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
# このフォルダを ~/oled_panel にコピーする（ユーザー jetson は i2c グループに入っている）
sudo cp ~/oled_panel/jetson-oled.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-oled.service
journalctl -u jetson-oled -f
```
