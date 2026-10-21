# 声で覚えた物を探す（object_finder）

スタックちゃんに「覚えて」と言って物を覚えさせ、物を動かしてから「探して」と言うと、AI-CAR が自動走行で部屋を回って探し、見つけたら止まります。物の場所は覚えません（毎回、部屋じゅうを探します）。

```
「覚えて」 → stackchan_bridge → POST 127.0.0.1:8095/register {"name": ""}
    → AI-CAR の /api/camera/snapshot のまん中の物の特徴（NanoOWL の枠 1 つ分）を ~/objects に保存（最後に覚えた物）
（物をどこかへ動かす）
「探して」 → POST /search {"name": ""}（最後に覚えた物）→ AI-CAR を auto（自動走行）にする
    → 1 秒に 5 回画像を見て、点 0.9 以上が 3 回中 2 回 → /api/stop → manual
    → スタックちゃん「見つけました」
```

- 名前つきも使える: 「これは赤いコップ、覚えて」「赤いコップを探して」→「赤いコップを発見しました」
- 止まるとき: 見つけた・「止まって」・5 分たった・ラズパイが自動走行をやめた（段差・障害物など）・3 回続けてようすが分からない
- 覚えるときは、物をカメラ（車の前）から 30〜50 cm のまん中に置く。画面のはしにつく枠、画面の 1 % より小さい・45 % より大きい枠はえらばない
- 覚えた物の写真は `~/objects/<名前>.jpg` に残る（どこを覚えたか確かめられる）
- ラズパイのプログラムは変えていない。安全の判定（LiDAR・段差）はいつもどおり

## ファイル
- `object_finder.py`: 本体（`dustynv/nanoowl:r36.3.0` コンテナの中で動かす）。`--test-register 名前 画像` / `--test-score 名前 画像` で、画像ファイルで試せる
- `jetson-finder.service`: Jetson の systemd サービス（127.0.0.1:8095、Jetson の中だけ）

## 入れ方（Jetson）
```bash
mkdir -p ~/object_finder ~/objects ~/owl/hf
cp object_finder.py jetson-finder.service ~/object_finder/
sudo cp ~/object_finder/jetson-finder.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now jetson-finder.service
journalctl -u jetson-finder -f   # 「物さがし 開始」が出れば OK
```

## 分かっていること（2026/10 実機）
- 960 × 540 の画像 1 枚で約 55〜70 ms。メモリは約 340 MB 使う
- 机の上のビンを覚えて右へ 20 cm 動かすと、そのビンは 0.97〜0.995、ほかの物はいちばん高くて 0.72
- このコンテナでは `torch` より先に `transformers`・`nanoowl` を読みこむ（逆だと `regex` の読みこみで落ちる）
- まだ試していないこと: うしろ向き・2 m より遠い・暗いとき
