#!/usr/bin/env python3
"""Bluetooth のスピーカー＆マイクで、Jetson の会話の AI と声で話す。

マイク（PipeWire / pactl の bluez_input）→ 声の区切りを音の大きさで見つける
→ whisper-server（声→文字）→ Ollama（返事の文、1 文ずつ）→ Kokoro（文字→声）
→ スピーカー（bluez_output）。話しているあいだはマイクの音を捨てて、自分の声を聞かないようにする。

標準ライブラリと numpy だけで動く（Jetson の /usr/bin/python3）。AI-CAR の走行には何も送らない。
"""
import argparse
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

import numpy as np

WHISPER_URL = 'http://127.0.0.1:8178/inference'
OLLAMA_URL = 'http://127.0.0.1:11434/api/chat'
KOKORO_URL = 'http://127.0.0.1:8880/v1/audio/speech'
SYSTEM_PROMPT = ('あなたは家庭用ロボット「スタックちゃん」です。日本語で、やさしく、1〜2文で短く答えてください。'
                 '中国語や英語は使わず、日本語だけで話してください。')
RATE = 16000
CHUNK = 480  # 30 ms
SENTENCE_END = re.compile(r'[。！？!?\n]')


def log(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


def pactl_names(kind, prefix):
    out = subprocess.run(['pactl', 'list', 'short', kind], capture_output=True, text=True).stdout
    return [line.split('\t')[1] for line in out.splitlines()
            if '\t' in line and line.split('\t')[1].startswith(prefix) and not line.split('\t')[1].endswith('.monitor')]


def find_bluetooth(mac, profile):
    """Bluetooth の機器の card / source / sink の名前。見つからなければ None。"""
    key = mac.replace(':', '_') if mac else ''
    cards = [c for c in pactl_names('cards', 'bluez_card.') if key in c]
    if not cards:
        return None
    subprocess.run(['pactl', 'set-card-profile', cards[0], profile], capture_output=True)
    time.sleep(1.0)
    sources = [s for s in pactl_names('sources', 'bluez_input.') if key in s]
    sinks = [s for s in pactl_names('sinks', 'bluez_output.') if key in s]
    if not sources or not sinks:
        return None
    return cards[0], sources[0], sinks[0]


def to_wav(samples, rate=RATE):
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(samples.astype(np.int16).tobytes())
    return buf.getvalue()


def stt(samples):
    peak = float(np.max(np.abs(samples))) or 1.0
    wav = to_wav(np.clip(samples * (0.7 * 32767 / peak), -32767, 32767))
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


def tts(text, voice, pitch=0):
    body = json.dumps({'model': 'kokoro', 'input': text, 'voice': voice, 'response_format': 'wav',
                       'lang_code': 'j', 'speed': 1.0}).encode()
    req = urllib.request.Request(KOKORO_URL, body, {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as r:
        wav = r.read()
    if not pitch:
        return wav
    return subprocess.run(['sox', '-t', 'wav', '-', '-t', 'wav', '-', 'pitch', str(pitch)],
                          input=wav, capture_output=True, check=True).stdout


class Mic(threading.Thread):
    """マイクの音を 30 ms ずつ読む。muted のあいだは捨てる。"""

    def __init__(self, source):
        super().__init__(daemon=True)
        self.proc = subprocess.Popen(['parec', '-d', source, '--rate', str(RATE), '--channels', '1',
                                      '--format', 's16le', '--raw', '--latency-msec', '30'],
                                     stdout=subprocess.PIPE)
        self.q = queue.Queue()
        self.muted = threading.Event()

    def run(self):
        n = CHUNK * 2
        while True:
            data = self.proc.stdout.read(n)
            if not data:
                self.q.put(None)
                return
            if not self.muted.is_set():
                self.q.put(np.frombuffer(data, dtype=np.int16).astype(np.float32))

    def flush(self):
        while not self.q.empty():
            self.q.get_nowait()


def listen(mic, args):
    """声の始まりから、静かになるまでの音を返す。"""
    noise = 100.0
    pre, speech, silent, started = [], [], 0, False
    while True:
        chunk = mic.q.get()
        if chunk is None:
            raise RuntimeError('マイクが止まりました')
        rms = float(np.sqrt(np.mean(chunk ** 2)))
        thr = max(args.min_level, noise * args.ratio)
        if not started:
            noise = 0.95 * noise + 0.05 * min(rms, thr)
            pre = (pre + [chunk])[-10:]
            if rms > thr:
                started, speech, silent = True, pre[:], 0
            continue
        speech.append(chunk)
        silent = silent + 1 if rms < thr else 0
        dur = len(speech) * CHUNK / RATE
        if silent * CHUNK / RATE >= args.end_silence or dur >= args.max_seconds:
            voiced = (len(speech) - silent) * CHUNK / RATE
            if voiced >= args.min_seconds:
                return np.concatenate(speech)
            pre, speech, silent, started = [], [], 0, False


def play(wav, sink):
    subprocess.run(['paplay', '-d', sink], input=wav, check=False)


def respond(text, history, args, sink):
    messages = [{'role': 'system', 'content': SYSTEM_PROMPT}] + history + [{'role': 'user', 'content': text}]
    audio = queue.Queue(maxsize=4)
    t0 = time.time()
    reply = []

    def produce():
        try:
            for s in llm_sentences(args.model, messages):
                reply.append(s)
                audio.put((s, tts(s, args.voice, args.pitch), time.time() - t0))
        except Exception as e:  # noqa: BLE001 - 返事の途中のエラーでも会話を続ける
            log(f'返事のエラー: {e}')
        audio.put(None)

    threading.Thread(target=produce, daemon=True).start()
    first = True
    while (item := audio.get()) is not None:
        s, wav, t = item
        if first:
            log(f'声が出るまで {t:.2f} 秒')
            first = False
        log(f'返事: {s}')
        play(wav, sink)
    return ''.join(reply)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--mac', default='04:52:C7:13:B4:EE', help='Bluetooth の機器のアドレス')
    p.add_argument('--profile', default='headset-head-unit-cvsd',
                   help='マイクを使う形（mSBC はこの Jetson ではマイクが無音になる）')
    p.add_argument('--model', default='qwen2.5:3b')
    p.add_argument('--voice', default='jf_alpha')
    p.add_argument('--pitch', type=int, default=300, help='声の高さを上げる量（セント、100 で半音。0 でそのまま）')
    p.add_argument('--min-level', type=float, default=150.0, help='声とみなす音の大きさの下限（16 bit の RMS）')
    p.add_argument('--ratio', type=float, default=4.0, help='まわりの音の何倍で声とみなすか')
    p.add_argument('--end-silence', type=float, default=0.8, help='この秒数静かなら話し終わり')
    p.add_argument('--min-seconds', type=float, default=0.4)
    p.add_argument('--max-seconds', type=float, default=15.0)
    p.add_argument('--turns', type=int, default=3, help='覚えておく会話の往復の数')
    args = p.parse_args()

    while (dev := find_bluetooth(args.mac, args.profile)) is None:
        log(f'Bluetooth の機器（{args.mac}）が見つかりません。10 秒後にもう一度探します')
        subprocess.run(['bluetoothctl', 'connect', args.mac], capture_output=True, timeout=30)
        time.sleep(10)
    card, source, sink = dev
    log(f'マイク: {source} / スピーカー: {sink}')
    mic = Mic(source)
    mic.start()
    history = []
    log('話しかけてください')
    while True:
        samples = listen(mic, args)
        mic.muted.set()
        try:
            t = time.time()
            text = stt(samples)
            log(f'聞き取り（{time.time() - t:.2f} 秒、声 {len(samples) / RATE:.1f} 秒）: {text}')
            if text and not re.fullmatch(r'[\s\W]*|\(.*\)|\[.*\]', text):
                reply = respond(text, history, args, sink)
                history = (history + [{'role': 'user', 'content': text},
                                      {'role': 'assistant', 'content': reply}])[-2 * args.turns:]
        except Exception as e:  # noqa: BLE001 - 1 回の失敗で止めない
            log(f'エラー: {e}')
        time.sleep(0.3)
        mic.flush()
        mic.muted.clear()


if __name__ == '__main__':
    main()
