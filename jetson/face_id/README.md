# 家族の顔を見分ける（Jetson、face_id）

顔の画像を受け取り、登録した家族のだれかを返します。あとで M5 スタックちゃん（机の上、カメラ GC0308 640×480）から
顔の画像を受け取る予定です。届くまでは、AI-CAR の 2 眼カメラ（`GET /api/camera/snapshot`）で試します。

```
（今）AI-CAR GET /api/camera/snapshot ─┐
（予定）スタックちゃん POST /api/face/recognize ─┴→ YuNet（顔を見つける）→ SFace（顔の特ちょう 128 個）
                                         → 登録した人と比べる（cos の近さ）→ 名前 / 知らない人
```

- 表示とあいさつ用です。AI-CAR の速度や安全停止には使いません。かぎ（セキュリティ）にも使いません
- 顔の画像は保存しません。`data/people.json`（このユーザーだけが読める 600）に、名前と特ちょうの数字だけを残します。クラウドには送りません
- CPU だけで動きます（OpenCV の `FaceDetectorYN`・`FaceRecognizerSF`、JetPack の `/usr/bin/python3` の OpenCV 4.8 で動く）。GPU とメモリは声の AI に残します
- 同じ人とみなす近さは 0.40 以上（`--match-threshold`。OpenCV の SFace の目安は 0.363）。低いと「知らない人」
- 登録できるのは、1 人だけ写っていて、顔の幅が 60 px 以上のときだけ（`--min-face-px`）。ほかの人の顔に近すぎるときも登録しません（名前のまちがいを防ぐ）。1 人 20 枚まで（古いものから消す）
- AI-CAR のカメラは、登録の画面を開いているあいだだけ（最後に見てから 15 秒）、1 秒に 2 回見ます（`--rate`）

## 画面
`http://<Jetson>:8090/`（例: `http://100.111.231.54:8090/`）。下の「API トークン」に AI-CAR と同じトークンを入れます。
名前を入れて「今の顔を登録」を、顔の向きを少しずつ変えて 10 回くらい押します。

## API（`X-API-Token` が要る。`AI_CAR_API_TOKEN` が空のときは要らない）
| 入口 | 中身 |
|---|---|
| `GET /api/face/status` | 登録した人（名前・枚数）と最後の結果 |
| `GET /api/face/frame.jpg` | 最後に見た画像 |
| `POST /api/face/recognize`（本文 = JPEG） | `faces: [{box: [x0,y0,x1,y1]（0〜1）, width_px, score, name（知らない人は null）, nearest, similarity}]` |
| `POST /api/face/enroll?name=<名前>`（本文 = JPEG、空なら AI-CAR のカメラ） | `{name, samples}` |
| `DELETE /api/face/people?name=<名前>` | その人の登録を消す |

## 入れ方（Jetson、JetPack 6.2 / L4T 36.4.3 で確認）
```bash
# このフォルダを ~/face_id にコピーする。AI のファイル（OpenCV Zoo、Apache-2.0）を入れる
mkdir -p ~/face_id/models && cd ~/face_id/models
curl -fLO https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
curl -fLO https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx
sha256sum -c <<'SUM'
8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4  face_detection_yunet_2023mar.onnx
0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79  face_recognition_sface_2021dec.onnx
SUM
# AI_CAR_URL と AI_CAR_API_TOKEN は jetson-status と同じ /etc/jetson-status.env を使う
sudo cp ~/face_id/jetson-face.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-face.service
journalctl -u jetson-face -f
```
