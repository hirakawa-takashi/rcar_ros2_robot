#!/usr/bin/env python3
"""スタックちゃん（純正ファームウェアの AI Agent）を Jetson の会話の AI につなぐ受け口。

XiaoZhi の 2 つの口をまねる:
- HTTP（--ota-port、既定 8003）`/xiaozhi/ota/`: 起動のたびに聞かれる。WebSocket の行き先だけを返し、
  新しいファームウェアは返さない（自動更新させない）。
- WebSocket（--ws-port、既定 8000）`/xiaozhi/v1/`: マイクの声（Opus 16 kHz）を受け取り、
  声の区切りを音の大きさで見つける → whisper-server（声→文字）→ Ollama（返事、1 文ずつ）
  → Kokoro（文字→声）→ Opus 24 kHz でスタックちゃんへ返す。
- HTTP（同じ --ota-port）`GET /status`: AI-CAR のダッシュボードのスタックちゃんのカード用。
  つながり・今の様子（聞いている / 考えている / 話している）・最後の会話を JSON で返す。

AI-CAR の走行・安全停止には何も送らない。
"""
import argparse
import asyncio
import io
import json
import queue
import re
import subprocess
import threading
import time
import urllib.request
import uuid
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import opuslib
from websockets.asyncio.server import serve

WHISPER_URL = 'http://127.0.0.1:8178/inference'
OLLAMA_URL = 'http://127.0.0.1:11434/api/chat'
KOKORO_URL = 'http://127.0.0.1:8880/v1/audio/speech'
SYSTEM_PROMPT = ('あなたは家庭用ロボット「スタックちゃん」です。日本語で、やさしく、1〜2文で短く答えてください。'
                 '中国語や英語は使わず、日本語だけで話してください。')
IN_RATE = 16000
OUT_RATE = 24000
FRAME_MS = 60
OUT_FRAME = OUT_RATE * FRAME_MS // 1000
SENTENCE_END = re.compile(r'[。！？!?\n]')
NOISE_TEXT = re.compile(r'[\s\W]*|\(.*\)|\[.*\]|（.*）')

STATUS_LOCK = threading.Lock()
STATUS = {'started': time.time(), 'device': {}, 'last_ota': None, 'last_seen': None, 'connected': 0,
          'state': 'idle', 'last': None, 'turns': 0, 'engines': {}}


def set_status(**kw):
    with STATUS_LOCK:
        STATUS.update(kw)
        if STATUS['connected'] or 'last_ota' in kw:
            STATUS['last_seen'] = time.time()
        if not STATUS['connected']:
            STATUS['state'] = 'idle'


def set_device(**kw):
    with STATUS_LOCK:
        STATUS['device'] = {**STATUS['device'], **{k: v for k, v in kw.items() if v and v != '?'}}


def log(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


def to_wav(samples, rate):
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(samples.astype(np.int16).tobytes())
    return buf.getvalue()


def stt(samples):
    peak = float(np.max(np.abs(samples))) or 1.0
    wav = to_wav(np.clip(samples * (0.7 * 32767 / peak), -32767, 32767), IN_RATE)
    b = uuid.uuid4().hex
    body = (f'--{b}\r\nContent-Disposition: form-data; name="file"; filename="a.wav"\r\n'
            'Content-Type: audio/wav\r\n\r\n').encode() + wav + \
        (f'\r\n--{b}\r\nContent-Disposition: form-data; name="response_format"\r\n\r\njson\r\n--{b}--\r\n').encode()
    req = urllib.request.Request(WHISPER_URL, body, {'Content-Type': f'multipart/form-data; boundary={b}'})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())['text'].strip().replace('\n', '')


def llm_sentences(model, messages):
    body = json.dumps({'model': model, 'stream': True, 'keep_alive': -1, 'messages': messages,
                       'options': {'num_predict': 120}}).encode()
    buf = ''
    with urllib.request.urlopen(urllib.request.Request(OLLAMA_URL, body), timeout=60) as r:
        for line in r:
            buf += json.loads(line).get('message', {}).get('content', '')
            while (m := SENTENCE_END.search(buf)):
                s, buf = buf[:m.end()].strip(), buf[m.end():]
                if s:
                    yield s
    if buf.strip():
        yield buf.strip()


def tts_pcm(text, voice, pitch, peak_db):
    """文字を 24 kHz・16 bit・1 ch の PCM（bytes）にする。いちばん大きい所を peak_db（dBFS）にそろえる。"""
    body = json.dumps({'model': 'kokoro', 'input': text, 'voice': voice, 'response_format': 'wav',
                       'lang_code': 'j', 'speed': 1.0}).encode()
    req = urllib.request.Request(KOKORO_URL, body, {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as r:
        wav = r.read()
    sox = ['sox', '-t', 'wav', '-', '-t', 'wav', '-r', str(OUT_RATE), '-c', '1', '-b', '16', '-']
    if pitch:
        sox += ['pitch', str(pitch)]
    sox += ['norm', str(peak_db)]
    wav = subprocess.run(sox, input=wav, capture_output=True, check=True).stdout
    with wave.open(io.BytesIO(wav)) as w:
        return w.readframes(w.getnframes())


class OtaHandler(BaseHTTPRequestHandler):
    """起動のたびの「新しい版はある？」に答える。WebSocket の行き先だけを返す。"""
    ws_port = 8000

    def _send_json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.split('?')[0].rstrip('/') == '/status':
            with STATUS_LOCK:
                self._send_json({**STATUS, 'now': time.time()})
        else:
            self._reply()

    def _reply(self):
        n = int(self.headers.get('Content-Length') or 0)
        info = self.rfile.read(n) if n else b''
        version = ''
        try:
            version = json.loads(info).get('application', {}).get('version', '')
        except (ValueError, AttributeError):
            pass
        host = (self.headers.get('Host') or '').rsplit(':', 1)[0] or self.server.server_address[0]
        dev = self.headers.get('Device-Id', '?')
        set_device(id=dev, ip=self.client_address[0], version=version)
        set_status(last_ota=time.time())
        log(f'OTA: {dev} 版 {version or "?"} → ws://{host}:{self.ws_port}')
        self._send_json({
            'server_time': {'timestamp': int(time.time() * 1000), 'timezone_offset': 540},
            'websocket': {'url': f'ws://{host}:{self.ws_port}/xiaozhi/v1/', 'token': '', 'version': 1},
        })

    do_POST = _reply

    def log_message(self, fmt, *a):
        pass


class Session:
    """WebSocket 1 本ぶん（スタックちゃんが会話を始めてから終わるまで）。"""

    def __init__(self, ws, args):
        self.ws = ws
        self.args = args
        self.sid = uuid.uuid4().hex[:12]
        self.dec = opuslib.Decoder(IN_RATE, 1)
        self.enc = opuslib.Encoder(OUT_RATE, 1, opuslib.APPLICATION_VOIP)
        self.listening = False
        self.mode = 'auto'
        self.speech = []
        self.pre = []
        self.started = False
        self.silent = 0.0
        self.noise = 100.0
        self.history = []
        self.reply_task = None
        self.cancel = threading.Event()

    async def send(self, **msg):
        msg['session_id'] = self.sid
        await self.ws.send(json.dumps(msg, ensure_ascii=False))

    def reset_vad(self):
        self.speech, self.pre, self.started, self.silent = [], [], False, 0.0

    async def on_text(self, msg):
        t = msg.get('type')
        if t == 'hello':
            await self.ws.send(json.dumps({
                'type': 'hello', 'transport': 'websocket', 'session_id': self.sid,
                'audio_params': {'format': 'opus', 'sample_rate': OUT_RATE, 'channels': 1,
                                 'frame_duration': FRAME_MS}}))
        elif t == 'listen':
            state = msg.get('state')
            if state == 'start':
                self.mode = msg.get('mode', 'auto')
                self.listening = True
                self.reset_vad()
                if not self.reply_task:
                    set_status(state='listening')
            elif state == 'stop':
                self.listening = False
                if self.mode == 'manual' and self.speech:
                    self.start_reply(np.concatenate(self.speech))
                elif not self.reply_task:
                    set_status(state='idle')
                self.reset_vad()
            elif state == 'detect':
                log(f'ウェイクワード: {msg.get("text", "")}')
        elif t == 'abort':
            self.cancel.set()
        elif t != 'mcp':
            log(f'未対応のメッセージ: {msg}')

    def on_audio(self, data):
        if not self.listening or self.reply_task:
            return
        try:
            pcm = np.frombuffer(self.dec.decode(data, IN_RATE * FRAME_MS // 1000), dtype=np.int16)
        except opuslib.OpusError:
            return
        chunk = pcm.astype(np.float32)
        if self.mode == 'manual':
            self.speech.append(chunk)
            return
        a = self.args
        dur = len(chunk) / IN_RATE
        rms = float(np.sqrt(np.mean(chunk ** 2))) if len(chunk) else 0.0
        thr = max(a.min_level, self.noise * a.ratio)
        if not self.started:
            self.noise = 0.95 * self.noise + 0.05 * min(rms, thr)
            self.pre = (self.pre + [chunk])[-5:]
            if rms > thr:
                self.started, self.speech, self.silent = True, self.pre[:], 0.0
            return
        self.speech.append(chunk)
        self.silent = self.silent + dur if rms < thr else 0.0
        total = sum(len(c) for c in self.speech) / IN_RATE
        if self.silent >= a.end_silence or total >= a.max_seconds:
            samples = np.concatenate(self.speech)
            self.reset_vad()
            if total - self.silent >= a.min_seconds:
                self.start_reply(samples)

    def start_reply(self, samples):
        self.cancel.clear()
        self.reply_task = asyncio.create_task(self.reply(samples))

    async def reply(self, samples):
        a = self.args
        try:
            t0 = time.time()
            set_status(state='thinking')
            text = await asyncio.to_thread(stt, samples)
            stt_s = time.time() - t0
            log(f'聞き取り（{stt_s:.2f} 秒、声 {len(samples) / IN_RATE:.1f} 秒）: {text}')
            if not text or NOISE_TEXT.fullmatch(text):
                return
            await self.send(type='stt', text=text)
            await self.send(type='llm', emotion='thinking', text='🤔')
            now = time.strftime('今は %Y年%m月%d日 %H時%M分です。')
            messages = [{'role': 'system', 'content': SYSTEM_PROMPT + now}] + self.history + \
                [{'role': 'user', 'content': text}]
            q = queue.Queue(maxsize=4)

            def produce():
                try:
                    for s in llm_sentences(a.model, messages):
                        if self.cancel.is_set():
                            break
                        q.put((s, tts_pcm(s, a.voice, a.pitch, a.peak_db)))
                except Exception as e:  # noqa: BLE001 - 返事の途中のエラーでも会話を続ける
                    log(f'返事のエラー: {e}')
                q.put(None)

            threading.Thread(target=produce, daemon=True).start()
            reply, first, voice_s = [], True, None
            await self.send(type='tts', state='start')
            while (item := await asyncio.to_thread(q.get)) is not None:
                if self.cancel.is_set():
                    continue
                s, pcm = item
                if first:
                    voice_s = time.time() - t0
                    set_status(state='speaking')
                    log(f'声が出るまで {voice_s:.2f} 秒')
                    await self.send(type='llm', emotion='happy', text='😊')
                    first = False
                log(f'返事: {s}')
                reply.append(s)
                await self.send(type='tts', state='sentence_start', text=s)
                await self.play(pcm)
            await self.send(type='tts', state='stop')
            with STATUS_LOCK:
                STATUS['turns'] += 1
                STATUS['last'] = {'at': time.time(), 'heard': text, 'reply': ''.join(reply),
                                  'stt_s': round(stt_s, 2), 'voice_s': voice_s and round(voice_s, 2)}
            self.history = (self.history + [{'role': 'user', 'content': text},
                                            {'role': 'assistant', 'content': ''.join(reply)}])[-2 * a.turns:]
        except Exception as e:  # noqa: BLE001 - 1 回の失敗で止めない
            log(f'エラー: {e}')
        finally:
            self.reply_task = None
            self.reset_vad()
            set_status(state='listening' if self.listening else 'idle')

    async def play(self, pcm):
        """60 ms ずつ Opus にして、はじめの 5 枚のあとは実際の速さで送る。"""
        step = OUT_FRAME * 2
        start = time.monotonic()
        for i, off in enumerate(range(0, len(pcm), step)):
            if self.cancel.is_set():
                return
            frame = pcm[off:off + step].ljust(step, b'\0')
            await self.ws.send(self.enc.encode(frame, OUT_FRAME))
            ahead = start + (i - 4) * FRAME_MS / 1000 - time.monotonic()
            if ahead > 0:
                await asyncio.sleep(ahead)


async def handler(ws, args):
    peer = ws.remote_address[0] if ws.remote_address else '?'
    dev = ws.request.headers.get('Device-Id', '?')
    log(f'つながりました: {peer}（{dev}）{ws.request.path}')
    set_device(id=dev, ip=peer)
    with STATUS_LOCK:
        STATUS['connected'] += 1
    set_status()
    s = Session(ws, args)
    try:
        async for msg in ws:
            if isinstance(msg, bytes):
                s.on_audio(msg)
            else:
                try:
                    await s.on_text(json.loads(msg))
                except ValueError:
                    log(f'JSON でないメッセージ: {msg[:80]}')
    finally:
        s.cancel.set()
        with STATUS_LOCK:
            STATUS['connected'] = max(0, STATUS['connected'] - 1)
        set_status(last_seen=time.time())
        log(f'切れました: {peer}')


async def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--ws-port', type=int, default=8000)
    p.add_argument('--ota-port', type=int, default=8003)
    p.add_argument('--model', default='qwen2.5:3b')
    p.add_argument('--voice', default='jf_alpha')
    p.add_argument('--pitch', type=int, default=300, help='声の高さを上げる量（セント、100 で半音。0 でそのまま）')
    p.add_argument('--peak-db', type=float, default=-1.0,
                   help='返事の声のいちばん大きい所の大きさ（dBFS、0 が上限）。Kokoro のままだと約 -7 dB で小さい')
    p.add_argument('--min-level', type=float, default=300.0, help='声とみなす音の大きさの下限（16 bit の RMS）')
    p.add_argument('--ratio', type=float, default=3.0, help='まわりの音の何倍で声とみなすか')
    p.add_argument('--end-silence', type=float, default=0.8, help='この秒数静かなら話し終わり')
    p.add_argument('--min-seconds', type=float, default=0.4)
    p.add_argument('--max-seconds', type=float, default=15.0)
    p.add_argument('--turns', type=int, default=3, help='覚えておく会話の往復の数')
    args = p.parse_args()

    OtaHandler.ws_port = args.ws_port
    set_status(engines={'stt': 'whisper-server', 'llm': args.model, 'tts': f'Kokoro {args.voice}'})
    ota = ThreadingHTTPServer((args.host, args.ota_port), OtaHandler)
    threading.Thread(target=ota.serve_forever, daemon=True).start()
    async with serve(lambda ws: handler(ws, args), args.host, args.ws_port, max_size=2 ** 20):
        log(f'待ち受け: OTA http://{args.host}:{args.ota_port}/xiaozhi/ota/  WebSocket :{args.ws_port}/xiaozhi/v1/')
        await asyncio.Future()


if __name__ == '__main__':
    asyncio.run(main())
