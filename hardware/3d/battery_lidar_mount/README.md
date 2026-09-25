# バッテリー 2 個 + RPLIDAR A1M8 マウント

天板（154 x 200 mm）の前側に、CIO SMARTCOBY Pro SLIM を 2 個重ねて収める箱と、その上に RPLIDAR A1M8 を載せるふたの 2 部品。

| ファイル | 内容 |
|---|---|
| `battery_lidar_mount.scad` | OpenSCAD ソース（寸法はすべて先頭のパラメーター） |
| `box.stl` | バッテリー箱（フランジを下にして印刷、サポート不要） |
| `lid.stl` | ふた + LiDAR 台（板を下にして印刷、サポート不要） |
| `assembly.png` / `top.png` | 組み立てイメージ / 真上から見た配置 |

## 主な寸法

- 箱の外形: 103.8 x 75.2 x 35.4 mm（内寸 98.8 x 70.2 x 33.4 mm）
- フランジ: 148 x 92 x 3 mm。天板の前側 4 穴（四隅 2 個 + 前端から 90 mm の 2 個）に合わせた Ø4.4 穴
- LiDAR 取付柱: 高さ 28 mm、M2.5 タッピング下穴 Ø2.2。穴位置はデータシート Figure 5-2（ヘッド側 56 mm 間隔、モーター側 40 mm 間隔）
- LiDAR 取付面の高さ: 天板上面から 67.4 mm。モーター側の先端は天板前端から約 5 mm はみ出す

## 組み立て

1. 天板の前側 4 本のネジを外し、フランジの厚さ（3 mm）ぶん長いネジで箱ごと留め直す
2. バッテリーを、ポートのある短辺を開口側（既定は右）に向けて 2 個重ねて差し込む
3. 開口側寄りの左右スロットに面ファスナー（幅 10 mm 程度）か結束バンドを通して、上下 2 個をまとめて固定
4. ふたを M3 x 8 mm のタッピングネジ 4 本で箱のボスに留める
5. LiDAR を M2.5 x 6〜8 mm のネジ 4 本で取付柱に留める

## 変更しやすいパラメーター

- `open_end`: バッテリー出し入れ側（`"right"` / `"left"`）
- `clr`: バッテリーとのすき間（片側 0.6 mm）
- `lidar_tower_h`: LiDAR の高さ
- `lidar_motor_front`: LiDAR のモーター側を前に向けるか
- `mount_hole_d`, `lidar_pilot_d`, `boss_pilot_d`: ネジ穴径

STL の再生成:

```
openscad -D 'part="box"' -o box.stl battery_lidar_mount.scad
openscad -D 'part="lid"' -o lid.stl battery_lidar_mount.scad
```
