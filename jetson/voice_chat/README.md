# Jetson で声の会話（Bluetooth のスピーカー＆マイク）

Bluetooth のスピーカー＆マイク（Bose SoundLink Mini II、`04:52:C7:13:B4:EE`）で、Jetson の会話の AI と声で話します。
AI-CAR の走行には何も送りません。

```
Bose のマイク → PipeWire（bluez_input、HFP の CVSD）→ voice_chat.py（音の大きさで話の区切りを見つける）
  → whisper-server :8178（声→文字）→ Ollama :11434 qwen2.5:3b（返事、1 文ずつ）
  → Kokoro :8880 jf_alpha（文字→声）→ sox pitch +300（約 3 半音高く、かわいく）→ PipeWire（bluez_output）→ Bose のスピーカー
```

- 話し終わって 0.8 秒静かになったら、話の終わりとする。0.4 秒より短い音は捨てる
- 返事は 1 文できるごとに声にして出す（最初の文から先に話しはじめる）
- 返事を話しているあいだはマイクの音を捨てる（自分の声を聞いて答えないように）
- 前の 3 往復を覚えて話す（`--turns`）
- 声は Kokoro の `jf_alpha` を `--pitch 300`（セント）高くしたもの。jf_nezumi・jf_tebukuro・jf_gongitsune と聞き比べて選んだ。高くするのにかかる時間は 1 文 約 0.04 秒
- 実機（2026/10）: 「こんにちは。今日は何をしましょうか?」→ 聞き取り 0.60 秒、返事の声が出るまで 1.32 秒

## 気をつけること
- マイクを使うと、Bluetooth は電話の形（HFP）になり、スピーカーも電話くらいの音質（モノラル）になる
- この Jetson では mSBC（`headset-head-unit-msbc`）だとマイクが無音になるので、CVSD（`headset-head-unit-cvsd`）を使う
- デスクトップを止めているので、PipeWire がログインに合わせて Bluetooth を切らないようにする:
  `loginctl enable-linger jetson` と、`~/.config/pipewire/media-session.d/media-session.conf`
  （`/usr/share/pipewire/media-session.d/` からコピー）の `with-pulseaudio` から `logind` を消す
- Bose は 2 台まで同時につながる。PC にもつながっていると、PC で音を出したときに PC 側に切り替わることがある
- 聞こえた声にはいつも答える（呼びかけの言葉はまだない）。テレビの音などにも答えることがある

## 入れ方（Jetson）
```bash
# このフォルダを ~/voice_chat にコピーする
sudo cp ~/voice_chat/voice-chat.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now voice-chat.service
journalctl -u voice-chat -f
```

手で動かすとき: `XDG_RUNTIME_DIR=/run/user/1000 python3 -u ~/voice_chat/voice_chat.py --help`
