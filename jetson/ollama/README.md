# Ollama（会話の AI）の設定

Jetson の声のやりとりで使う会話の AI（`qwen2.5:3b`）を、いつもメモリに読み込んだままにします。
Ollama は、ふつうは 5 分使わないとモデルをメモリから外すので、その次の 1 回目の返事が数秒遅くなります。

- `voice.conf`: `ollama.service` の追加設定。同時に 1 人だけ答える（`NUM_PARALLEL=1`）、
  モデルは 1 つだけ（`MAX_LOADED_MODELS=1`）、メモリを減らす（`FLASH_ATTENTION`・`KV_CACHE_TYPE=q8_0`）、
  外さない（`KEEP_ALIVE=-1`）
- `ollama-preload.service`: 電源を入れたとき、Ollama が起動したら `qwen2.5:3b` を読み込む

メモリ（8 GB）のうち、約 2.5 GB を GPU 側でずっと使います。NanoOWL を止めた状態で、空きは約 1.2 GB です。

## 入れ方（Jetson）
```bash
sudo install -m 644 voice.conf /etc/systemd/system/ollama.service.d/voice.conf
sudo install -m 644 ollama-preload.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart ollama.service
sudo systemctl enable --now ollama-preload.service
ollama ps   # UNTIL が Forever になる
```

一時的にメモリを空けたいときは `ollama stop qwen2.5:3b`（次に話しかけると、また読み込みます）。
