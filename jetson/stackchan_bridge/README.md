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

公式 https://github.com/m5stack/StackChan の `firmware`（ESP-IDF v5.5.4）に、ここの 2 つを足してビルドします。

| ファイル | 置く所 | 中身 |
| --- | --- | --- |
| `firmware/sdkconfig.defaults.local` | `StackChan/firmware/` | `CONFIG_OTA_URL` を Jetson（`http://192.168.11.23:8003/xiaozhi/ota/`）にする |
| `firmware/xiaozhi-no-auto-upgrade.patch` | `StackChan/firmware/xiaozhi-esp32/` で `git apply` | AI Agent を開くたびの自動更新（`UpgradeFirmware`）をやめ、ログだけ出す（純正の版にもどらないように） |

```bash
cd StackChan/firmware
python3 ./fetch_repos.py                 # 依存の取得と公式のパッチ
cp <このフォルダ>/firmware/sdkconfig.defaults.local .
git -C xiaozhi-esp32 apply <このフォルダ>/firmware/xiaozhi-no-auto-upgrade.patch
. ~/esp/esp-idf-v5.5.4/export.sh
idf.py build
# Jetson の USB-A につなぐと /dev/ttyACM0。Wi-Fi の設定（NVS）は消えない
python3 esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash $(cat build/flash_args)
```

- 書き込むのは ota_0（0x20000）・assets（0xa00000）・otadata（0xd000）など。NVS（0x9000）の Wi-Fi の設定と、スマホのアプリの登録はそのまま
- Jetson の IP アドレスが変わるとつながらない。ルーターで固定する（2026/10: Jetson 192.168.11.23、スタックちゃん 192.168.11.8）
- スマホのアプリで変える AI の設定（声・性格）は使わなくなる。声の大きさは本体の設定の「Volume」（0〜100、最初は 70）でも変えられる

## 純正にもどす

書き換える前に、純正の全部（16 MB、版 1.5.1）を Jetson の `~/stackchan_backup/stackchan_factory_16MB.bin`
（SHA-256 `7dcebeb7bfb540c758df5d1936169511d5b9ab4116ed8ccd0cdb90a96e02fbad`）にとってあります。約 4 分です。

```bash
python3 esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x0 ~/stackchan_backup/stackchan_factory_16MB.bin
```
