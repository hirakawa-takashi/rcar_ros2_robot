# スタックちゃんを Jetson の会話の AI につなぐ（stackchan_bridge）

M5Stack の純正ファームウェア（StackChan 1.5.1、中の会話は XiaoZhi）の AI Agent を、
ネットの向こうの XiaoZhi のサーバーではなく、家の Jetson の会話の AI につなぎます。
表情・首振り・画面・タッチは純正のままです。AI-CAR の走行・安全停止には何も送りません。

```
スタックちゃん ──HTTP :8003 /xiaozhi/ota/──▶ stackchan_bridge（WebSocket の行き先だけ返す。新しい版は返さない）
             ──WebSocket :8000 /xiaozhi/v1/─▶ 声（Opus 16 kHz）→ 音の大きさで区切る
                                              → whisper-server :8178（声→文字）
                                              → Ollama :11434 qwen2.5:3b（返事、1 文ずつ）
                                              → Kokoro :8880 jf_alpha + sox pitch 300・norm -1 dB（文字→声）
             ◀── Opus 24 kHz・60 ms ──────────┘  stt / llm（表情）/ tts の JSON も送る
```

- 2026/10 の実機: 聞き取り 約 0.3〜0.7 秒、話し終わってから声が出るまで 約 1.6〜2.8 秒。メモリ 約 37 MB
- 返事の声: Kokoro のままだといちばん大きい所が上限の約 4 割（-7 dB）で小さかったので、文ごとに -1 dB にそろえる（`--peak-db`）
- 声の区切り: まわりの音の 3 倍（最低 RMS 300）より大きいと話し始め、0.8 秒静かなら話し終わり（`--ratio`・`--min-level`・`--end-silence`）。
  「(音楽)」のような文字だけの聞き取りは捨てる
- 会話は 3 往復まで覚える（WebSocket が切れると忘れる）。今の日時をシステムの指示に入れる
- `GET :8003/status`: つながり（`connected`）・今の様子（`state`: `listening` / `thinking` / `speaking` / `idle`）・最後の会話（`last`: 聞き取り・返事・かかった秒）・本体（`device`: MAC・IP・版）・会話の数（`turns`）を JSON で返す。AI-CAR のダッシュボードのスタックちゃんのカードが `GET /api/stackchan/status` 経由で 2 秒ごとに取る
- カメラ: つながると MCP の `initialize` で写真の送り先（`http://<Jetson>:8003/vision`）を教え、`tools/list` に `self.camera.take_photo` があれば `camera: true`。`POST :8003/photo` でその道具を呼ぶと、スタックちゃんが 320×240 の JPEG（約 12 KB）を `/vision` へ送り、`GET :8003/photo.jpg` でいちばん新しい 1 枚を返す（`photo_at` は撮った時刻）。ファイルには保存しない。撮るたびにスタックちゃんでシャッターの音が鳴る。ダッシュボードの「写真を撮る」が `POST /api/stackchan/photo` 経由で呼ぶ
- 認証はしない（`token` は空）。家の LAN・Tailscale の中だけで使う

## Jetson に入れる

```bash
mkdir -p ~/stackchan_bridge
cp stackchan_bridge.py ~/stackchan_bridge/
pip3 install --user websockets==13.1 opuslib==3.0.1   # libopus0 と sox も要る
sudo install -m 644 jetson-stackchan.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-stackchan.service
journalctl -u jetson-stackchan -f    # 聞き取り・返事・かかった秒数が出る
```

## スタックちゃんのファームウェア

公式 https://github.com/m5stack/StackChan の `firmware`（ESP-IDF v5.5.4）に、ここの 4 つを足してビルドします。

| ファイル | 置く所 | 中身 |
| --- | --- | --- |
| `firmware/sdkconfig.defaults.local` | `StackChan/firmware/` | `CONFIG_OTA_URL` を Jetson（`http://192.168.11.23:8003/xiaozhi/ota/`）にする |
| `firmware/xiaozhi-no-auto-upgrade.patch` | `StackChan/firmware/xiaozhi-esp32/` で `git apply` | AI Agent を開くたびの自動更新（`UpgradeFirmware`）をやめ、ログだけ出す（純正の版にもどらないように） |
| `firmware/xiaozhi-auto-reopen.patch` | `StackChan/firmware/xiaozhi-esp32/` で `git apply` | 会話が切れたら（画面をさわって止めたとき以外）、自分でつなぎ直す（3 秒後、だめなら 6・12…最大 60 秒ごと。つなぎ直しの失敗ではエラーの音を出さない）。AI Agent を開いたときも 2 秒後に自分でつなぐ（下の「呼びかけ」） |
| `firmware/stackchan-camera-stream.patch` | `StackChan/firmware/` で `git apply` | カメラの映像を OTA と同じ所の `/camera/frame` へ送り続ける（下の「カメラの映像」） |
| `firmware/stackchan-face-color.patch` | `StackChan/firmware/` で `git apply` | 顔（目と口）の色を変える MCP の道具 `self.robot.set_face_color`（`color`: 0xRRGGBB）。user only なので、ネットの AI からは見えない（下の「ネットとローカルの切り替え」） |
| `firmware/stackchan-countdown-shot.patch` | `StackChan/firmware/` で `git apply`（camera-stream のあと） | 「撮影して」の MCP の道具 `self.camera.countdown_photo`（`seconds`: 1〜9）。正面を向き、画面にカメラと数字を出して数え、撮って `/camera/shot` へ送る（下の「撮影と名前の登録」） |

```bash
cd StackChan/firmware
python3 ./fetch_repos.py                 # 依存の取得と公式のパッチ
cp <このフォルダ>/firmware/sdkconfig.defaults.local .
git -C xiaozhi-esp32 apply <このフォルダ>/firmware/xiaozhi-no-auto-upgrade.patch
git -C xiaozhi-esp32 apply <このフォルダ>/firmware/xiaozhi-auto-reopen.patch
git apply <このフォルダ>/firmware/stackchan-camera-stream.patch
git apply <このフォルダ>/firmware/stackchan-face-color.patch
git apply <このフォルダ>/firmware/stackchan-countdown-shot.patch
. ~/esp/esp-idf-v5.5.4/export.sh
idf.py build
# Jetson の USB-A につなぐと /dev/ttyACM0。Wi-Fi の設定（NVS）は消えない
python3 esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash $(cat build/flash_args)
```

- 書き込むのは ota_0（0x20000）・assets（0xa00000）・otadata（0xd000）など。NVS（0x9000）の Wi-Fi の設定と、スマホのアプリの登録はそのまま
- Jetson の IP アドレスが変わるとつながらない。ルーターで固定する（2026/10: Jetson 192.168.11.23、スタックちゃん 192.168.11.8）
- スマホのアプリで変える AI の設定（声・性格）は使わなくなる。声の大きさは本体の設定の「Volume」（0〜100、最初は 70）でも変えられる

## カメラの映像

`stackchan-camera-stream.patch` を入れると、AI Agent の間、スタックちゃんが会話とは別に（画面をさわらなくても）
`http://<OTA と同じ所>/camera/frame` に聞きに来ます。

```
スタックちゃん: GET  /camera/frame          → bridge: 「1」（ダッシュボードが 10 秒以内に見に来た）/「0」
                「1」なら JPEG（320×240、品質 30、シャッターの音なし）を POST /camera/frame、返事が「1」の間くり返す
                「0」なら 2 秒ごとに GET だけ（撮らない）、つながらないと 5 秒ごと
ダッシュボード: GET /api/stackchan/frame.jpg（0.25 秒ごと、ページを見ている間だけ）→ bridge GET /camera/frame.jpg
```

- 映像はいちばん新しい 1 コマだけをメモリに置きます（保存しない）。3 秒より古いと 404
- `/status` の `stream_at`（スタックちゃんが最後に聞きに来た時刻）・`frame_at`・`fps` をダッシュボードが出します。
  `stream_at` がないときは、前のとおり「写真を撮る」（MCP の `self.camera.take_photo`）だけ
- 「写真を撮る」と映像は同じカメラを使うので、ファームウェアの中で順番に使います（`frame_mutex_`）

## 撮影と名前の登録

```
「撮影して」「写真を撮って」（ローカルは whisper、ネットは XiaoZhi の stt を bridge が見る）
  → bridge「撮影します。こっちを向いてね。」→ MCP self.camera.countdown_photo {seconds: 5}
  → スタックちゃん: 首を正面（yaw 0）へ → bridge が self.robot.set_head_angles で上（pitch 30 度、--shot-pitch。数えているあいだ 1 秒ごとに送り直す）へ → 画面にカメラの映像と右上に 5〜1 の数字（1 秒ごとにピッ）
  → 0 でシャッターの音 → 撮った写真を 6 秒画面に出す → JPEG（320×240、品質 80）を POST /camera/shot
  → bridge: face_id の /api/face/recognize で顔を見る →「撮れたよ。ダッシュボードで名前を登録してね。」など
ダッシュボード: 写真の下の「名前」→「名前を登録」→ POST /api/stackchan/enroll?name= → bridge POST /enroll?name=
  → face_id の /api/face/enroll（顔が 1 つ・幅 60 px 以上・ほかの人と近すぎない）
```

- 数えるのと撮るのは、ファームウェアの映像の送り係（`cam_frame` の task）がします。MCP の返事はすぐ返り、
  会話の動き（main の task）は止めません。bridge は写真が `/camera/shot` に届くまで最大 20 秒待ちます
- ネットのときも同じです。そのときは XiaoZhi に `abort` を送り、撮り終わるまで XiaoZhi の声と文字はスタックちゃんへ流しません
- face_id の API トークンは、`jetson-stackchan.service` の `EnvironmentFile=/etc/jetson-status.env` の
  `AI_CAR_API_TOKEN` を使います。face_id の場所は `FACE_ID_URL`（既定 `http://127.0.0.1:8090`）
- 顔の見分けは、あいさつや表示だけに使います。鍵・走行・安全停止には使いません
- `/status` の `shot_tool`（ファームウェアに道具がある）・`shooting`（数えている）・`shot`（`faces`・`name`・`error`）

## 見守りとあいさつ

```
スタックちゃんの映像の送り係が 2 秒ごとに GET /camera/frame で聞きに来る
  → bridge が見守りの時刻なら「1」→ 次に 1 枚（320×240）を POST /camera/frame
  → bridge: face_id の /api/face/recognize → 登録した人がいて、30 分あいさつしていなければ
  → 「○○さん、こんにちは！」（4〜10 時 おはよう、17 時から こんばんは）
```

- 間隔は `--watch-interval`（既定 2 秒、0 で見守りをしない）。映像の 1 枚は約 4 KB で、2〜3 秒に 1 枚です。ファームウェアの書き換えはいりません
- あいさつは、スタックちゃんが会話でつながっているときだけです（つながっていないと声を出せません）。会話・撮影・ネットとの切り替えの最中もしません
- 撮影や名前の登録をした人は、そのときあいさつしたものとして数えます
- `/status` の `watch_on`・`watch`（`at`・`faces`・`names`・`error`）・`greet`（`at`・`names`）

## 呼びかけ（「スタックちゃん」、ローカルのとき）

スタックちゃんの中の呼びかけ（WakeNet の「Hi, Stack Chan」）は英語の読み上げ声の見本と比べるので、日本語の発音ではほとんど反応しません。
そこで、ローカルのときは Jetson が日本語の「スタックちゃん」を聞き分けます。ファームウェアの書き換えはいりません。

```
画面を 1 回さわる → いつもどおり話せる（呼びかけはいらない）
  → さわってから・最後に話してから 30 秒（--awake-seconds）たつと「呼びかけ待ち」（/status の state が waiting）
  → スタックちゃんは聞いているまま。声は whisper で文字にするが、「スタックちゃん」が入っているときだけ返事をする
     「スタックちゃん」だけ → 「はい、なあに？」→ そのあと 30 秒は名前なしで話せる
     「スタックちゃん、今日は何曜日？」→ そのまま返事
```

- 聞き分ける言葉は `--wake-words`（正規表現。whisper の書き方のゆれ「スダックちゃん」「スタッチャン」「Stack-chan」なども入れてある）。空にすると呼びかけ待ちにせず、今までどおり何にでも返事をする
- スタックちゃんは、Jetson から 120 秒何も届かないと会話が切れたとみなすので、60 秒ごとに `{"type": "ping"}` を送る（スタックちゃんは知らない種類として記録に 1 行出すだけ）
- 呼びかけ待ちのあいだも、聞こえた声は whisper で文字にします（1 回 約 0.4〜0.5 秒）。テレビの声なども文字にするので、Jetson が少し働きます。返事はしません
- 「ネットにして」「撮影して」も、呼びかけ待ちのときは「スタックちゃん、ネットにして」のように名前を付けて言います
- ネットのときは使いません（今までどおり画面をさわって話す）
- 呼びかけは、スタックちゃんが Jetson につながっている（ランプが緑）ときだけ聞こえます。ランプが消えている（待ち受け）ときは声を送らないので、`xiaozhi-auto-reopen.patch` で、会話が切れたら自分でつなぎ直します（画面をさわって止めたときはつなぎ直さない）。AI Agent を開いたときも、さわらなくてもつながります。つなぎ直したときは、さわったときと同じく 30 秒は名前なしで返事をします
- あいさつ（見守り）をしたあとも 30 秒は名前なしで返事できます

## ネットとローカルの切り替え

スタックちゃんはいつも Jetson につなぎ、ネットのときは bridge がネットの XiaoZhi へ中継します（ファームウェアの行き先は変えない）。

```
「ネットにして」（Jetson の whisper が聞き取る）→ 「ネットにつなぎます」→ bridge が XiaoZhi の OTA
  （https://api.tenclass.net/xiaozhi/ota/、スタックちゃんの Device-Id・Client-Id で）に聞いて wss につなぐ
  → スタックちゃんの hello を送り、声・JSON・MCP をそのまま中継（session_id だけ入れかえる）。顔は緑（0x00FF00）
「ローカルにして」（XiaoZhi の stt を bridge が見る）→ 中継をやめる → 顔は白 →「ローカルにもどりました」
```

- 言い方: 「ネット / インターネット / クラウド / NET」か「ローカル / ジェットソン / LOCAL」のあとに
  「モード」「にして」「に切り替えて」「に変えて」「につないで」「に戻して」。「ネットで調べて」などでは切りかわらない
- 今の行き先は `mode.txt`、スタックちゃんの最後の OTA の問い合わせ（ネットの OTA にも同じものを送る）は `device_ota.json`（どちらも bridge と同じフォルダ）に残す。bridge やスタックちゃんを起動し直しても同じ。`/status` の `mode`（`local` / `net`）
- ネットにつながらない・登録が要るときは、「ネットにつながりませんでした。ローカルで話します。」と言ってローカルのまま
- ネットの間も、カメラの映像（`/camera/frame`）は Jetson に来ます。「写真を撮る」は、XiaoZhi が写真の送り先を自分のほうに変えるので使えません（ローカルにもどると、送り先を Jetson にもどす）
- XiaoZhi が会話を切ると（しばらく話さないときなど）、bridge もスタックちゃんとの会話を切ります。画面を 1 回さわると、またネットにつながります
- ネットの間も Jetson を通るので、Jetson が止まると会話はできません（走行・安全停止には関係ありません）

## 純正にもどす

書き換える前に、純正の全部（16 MB、版 1.5.1）を Jetson の `~/stackchan_backup/stackchan_factory_16MB.bin`
（SHA-256 `7dcebeb7bfb540c758df5d1936169511d5b9ab4116ed8ccd0cdb90a96e02fbad`）にとってあります。約 4 分です。

```bash
python3 esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x0 ~/stackchan_backup/stackchan_factory_16MB.bin
```
