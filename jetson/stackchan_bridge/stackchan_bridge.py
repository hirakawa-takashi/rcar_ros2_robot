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
- カメラ: つながったら MCP の initialize で写真の送り先（`POST /vision`）を教える。`POST /photo` で
  MCP の `self.camera.take_photo` を呼ぶと、スタックちゃんが JPEG を `/vision` へ送ってくるので、
  いちばん新しい 1 枚を `GET /photo.jpg` で返す（撮るたびにシャッターの音が鳴る）。
- ネット / ローカルの切り替え: 「ネットにして」と言うと、この受け口がスタックちゃんのかわりにネットの XiaoZhi
  （api.tenclass.net）につなぎ、声と文字をそのまま中継する。「ローカルにして」で Jetson の会話にもどる。
  今の行き先は mode.txt に残す（起動し直しても同じ）。ネットのあいだは顔（目と口）を緑にする
  （ファームウェアの MCP `self.robot.set_face_color`、stackchan-face-color.patch）。
- 調べもの（ネットのときだけ）: MCP の道具 `self.web.search` を足して、ニュース・天気・今の役職など新しいことを
  XiaoZhi から呼んでもらう。その呼び出しはスタックちゃんへ流さず、この受け口がインターネット（Google ニュースの
  見出しと DuckDuckGo）で調べて答える。ローカルのときは調べない（Jetson の AI だけで答える）。
- 映像: ダッシュボードの「映像を撮る」（`POST /camera/stream?on=1`）を押したときだけ送ってもらう。
- 撮影: 「撮影するよ」と言うと、MCP の `self.camera.countdown_photo`（stackchan-countdown-shot.patch）で
  正面を向いて 5 秒数え、画面にカメラを出してから撮る。写真は `POST /camera/shot` に届く。face_id で顔を見て
  （`shot`）、ダッシュボードから `POST /enroll?name=` で名前を付けて face_id に登録する（ネットのときも同じ）。

AI-CAR の走行・安全停止には何も送らない。
"""
import argparse
import asyncio
import html
import io
import json
import os
import queue
import re
import subprocess
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import opuslib
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

WHISPER_URL = 'http://127.0.0.1:8178/inference'
OLLAMA_URL = 'http://127.0.0.1:11434/api/chat'
KOKORO_URL = 'http://127.0.0.1:8880/v1/audio/speech'
SYSTEM_PROMPT = ('あなたは家庭用ロボット「スタックちゃん」です。日本語で、やさしく、1〜2文で短く答えてください。'
                 '中国語や英語は使わず、日本語だけで話してください。')
IN_RATE = 16000
OUT_RATE = 24000
FRAME_MS = 60
CLOUD_HOLD = 6  # ネットの声は出はじめを 6 枚（0.36 秒）ためてから流す。Wi-Fi のゆれで途切れないように
OUT_FRAME = OUT_RATE * FRAME_MS // 1000
SENTENCE_END = re.compile(r'[。！？!?\n]')
NOISE_TEXT = re.compile(r'[\s\W]*|\(.*\)|\[.*\]|（.*）')
PHOTO_TOOL = 'self.camera.take_photo'
PHOTO_TIMEOUT = 15.0
PHOTO_MAX_BYTES = 2 * 2 ** 20
FRAME_PATH = '/camera/frame'
STREAM_PATH = '/camera/stream'
# 「撮影するよ」: 正面を向いて 5 秒数え、画面にカメラを出してから撮る（ファームウェアが SHOT_PATH へ送る）
SHOT_TOOL = 'self.camera.countdown_photo'
SHOT_PATH = '/camera/shot'
SHOT_SECONDS = 5
SHOOT = re.compile(r'(?:撮影|さつえい|写真(?:を|お)?(?:撮|と)(?:って|る|ろ))')
FACE_ID_URL = os.environ.get('FACE_ID_URL', 'http://127.0.0.1:8090')
FACE_ID_TIMEOUT = 10.0
NAME_MAX = 20
HEAD_TOOL = 'self.robot.set_head_angles'
# 見守り: 映像を WATCH_INTERVAL 秒に 1 枚もらって顔を見て、登録した人に名前であいさつする（同じ人は GREET_HOLD 秒に 1 回）
GREET_HOLD = 30 * 60
VIEWER_HOLD = 10.0
FRAME_STALE = 3.0
CLOUD_OTA_URL = 'https://api.tenclass.net/xiaozhi/ota/'
FACE_TOOL = 'self.robot.set_face_color'
FACE_COLOR = {'local': 0xFFFFFF, 'net': 0x00FF00}
SEARCH_TOOL = 'self.web.search'
SEARCH_DESC = ('Search the internet for up-to-date information: news, weather, prices, who currently holds a '
               'position, and anything that may have changed after your training. Always use it for questions '
               'about the present or recent events. Returns news headlines with dates and web snippets.')
SEARCH_PARAMS = {'type': 'object', 'properties': {'query': {'type': 'string', 'description': 'search words'}},
                 'required': ['query']}
MODE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mode.txt')
OTA_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'device_ota.json')
_SWITCH = r'(?:モード|mode|(?:に|へ)?(?:して|切り?替え|きりかえ|変え|かえ|つない|繋い|戻|もど))'
TO_NET = re.compile(r'(?:インターネット|ネット|クラウド|net)' + _SWITCH, re.I)
TO_LOCAL = re.compile(r'(?:ローカル|ジェットソン|local|jetson)' + _SWITCH, re.I)

STATUS_LOCK = threading.Lock()
STATUS = {'started': time.time(), 'device': {}, 'last_ota': None, 'last_seen': None, 'connected': 0,
          'state': 'idle', 'last': None, 'turns': 0, 'engines': {}, 'camera': False, 'photo_at': None,
          'stream_at': None, 'frame_at': None, 'fps': None, 'stream_on': False, 'mode': 'local',
          'shot_tool': False, 'shooting': False, 'shot': None, 'watch_on': False, 'watch': None, 'greet': None}
DEVICE_OTA = {'headers': {}, 'body': b''}
PHOTO = {'jpeg': None}
FRAME = {'jpeg': None, 'times': [], 'viewer_at': 0.0, 'on': False}
WATCH = {'interval': 0.0, 'next': 0.0, 'busy': False, 'greeted': {}}
SESSIONS = []
LOOP = None


def set_status(**kw):
    with STATUS_LOCK:
        STATUS.update(kw)
        if STATUS['connected'] or 'last_ota' in kw:
            STATUS['last_seen'] = time.time()
        if not STATUS['connected']:
            STATUS['state'] = 'idle'


def want_frames():
    """「映像を撮る」が押されていて、ダッシュボードが 10 秒以内に取りに来ているか、見守りの次の 1 枚の時刻なら '1'。"""
    now = time.time()
    if FRAME['on'] and now - FRAME['viewer_at'] >= VIEWER_HOLD:
        FRAME['on'] = False
        set_status(stream_on=False)
    watch = WATCH['interval'] > 0 and not WATCH['busy'] and now >= WATCH['next']
    return '1' if FRAME['on'] or watch else '0'


def jpeg_from_multipart(body, ctype):
    """multipart/form-data の file の中身（JPEG）を取り出す。なければ None。"""
    m = re.search(r'boundary="?([^";]+)"?', ctype or '')
    if not m:
        return None
    for part in body.split(b'--' + m.group(1).encode()):
        head, _, data = part.partition(b'\r\n\r\n')
        if b'name="file"' in head:
            data = data[:-2] if data.endswith(b'\r\n') else data
            return data if data.startswith(b'\xff\xd8') else None
    return None


def set_device(**kw):
    with STATUS_LOCK:
        STATUS['device'] = {**STATUS['device'], **{k: v for k, v in kw.items() if v and v != '?'}}


def load_mode():
    try:
        with open(MODE_FILE) as f:
            return 'net' if f.read().strip() == 'net' else 'local'
    except OSError:
        return 'local'


def save_mode(mode):
    with open(MODE_FILE, 'w') as f:
        f.write(mode + '\n')
    set_status(mode=mode)


def load_device_ota():
    """最後にスタックちゃんから来た OTA の問い合わせ（ネットの OTA に同じものを送る）。"""
    try:
        with open(OTA_FILE) as f:
            d = json.load(f)
        DEVICE_OTA.update(headers=d['headers'], body=d['body'].encode())
    except (OSError, ValueError, KeyError):
        pass


def save_device_ota(headers, body):
    DEVICE_OTA.update(headers=headers, body=body)
    try:
        with open(OTA_FILE, 'w') as f:
            json.dump({'headers': headers, 'body': body.decode(errors='replace')}, f, ensure_ascii=False)
    except OSError as e:
        log(f'OTA の問い合わせを残せません: {e}')


def is_switch(pattern, text):
    return bool(pattern.search(re.sub(r'\s', '', text or '')))


def face_id(path, jpeg):
    """Jetson の face_id に JPEG を送って JSON を返す。だめなら ValueError（face_id の理由）か OSError。"""
    req = urllib.request.Request(FACE_ID_URL + path, jpeg, {
        'Content-Type': 'image/jpeg', 'X-API-Token': os.environ.get('AI_CAR_API_TOKEN', '')})
    try:
        with urllib.request.urlopen(req, timeout=FACE_ID_TIMEOUT) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            msg = json.loads(e.read()).get('error')
        except (ValueError, AttributeError):
            msg = None
        raise ValueError(msg or f'顔の見分けが {e.code} を返しました')


def face_check(jpeg):
    """撮った写真の顔を見て、ダッシュボード用の様子と、スタックちゃんが話す 1 文を返す。"""
    shot = {'at': time.time(), 'faces': None, 'name': None, 'width_px': None, 'error': None}
    try:
        faces = face_id('/api/face/recognize', jpeg).get('faces') or []
    except (ValueError, OSError) as e:
        shot['error'] = str(e) if isinstance(e, ValueError) else '顔の見分け（face_id）につながりません'
        return shot, '撮れたよ。ダッシュボードを見てね。'
    shot['faces'] = len(faces)
    if len(faces) != 1:
        return shot, ('撮れたよ。でも、顔が見つからなかったよ。' if not faces
                      else f'撮れたよ。顔が {len(faces)} つ写っているよ。登録は 1 人ずつしてね。')
    shot.update(name=faces[0].get('name'), width_px=faces[0].get('width_px'))
    if shot['name']:
        return shot, f'撮れたよ。{shot["name"]}さんだね。'
    return shot, '撮れたよ。ダッシュボードで名前を登録してね。'


def greeting_word():
    h = time.localtime().tm_hour
    return 'おはよう' if 4 <= h < 10 else 'こんにちは' if h < 17 else 'こんばんは'


def mark_greeted(name):
    if name:
        WATCH['greeted'][name] = time.time()


def watch_check(jpeg):
    """見守りの 1 枚の顔を見て、まだあいさつしていない登録した人がいれば、つながっているスタックちゃんにあいさつしてもらう。"""
    try:
        watch = {'at': time.time(), 'faces': None, 'names': [], 'error': None}
        try:
            faces = face_id('/api/face/recognize', jpeg).get('faces') or []
        except (ValueError, OSError) as e:
            watch['error'] = str(e) if isinstance(e, ValueError) else '顔の見分け（face_id）につながりません'
            set_status(watch=watch)
            return
        watch.update(faces=len(faces), names=sorted({f['name'] for f in faces if f.get('name')}))
        set_status(watch=watch)
        now = time.time()
        new = [n for n in watch['names'] if now - WATCH['greeted'].get(n, 0) >= GREET_HOLD]
        sess = SESSIONS[-1] if SESSIONS else None
        if new and sess and sess.can_greet():
            asyncio.run_coroutine_threadsafe(sess.greet(new), LOOP).result(30)
    except Exception as e:  # noqa: BLE001 - 見守りの失敗で受け口を止めない
        log(f'見守りのエラー: {e}')
    finally:
        WATCH['busy'] = False


def cloud_ota(device_id, client_id):
    """スタックちゃんのかわりにネットの XiaoZhi の OTA に聞いて、WebSocket の行き先と token を返す。"""
    version = STATUS['device'].get('version', '')
    headers = {'Activation-Version': '1', 'User-Agent': f'm5stack-stack-chan/{version}',
               **DEVICE_OTA['headers'], 'Device-Id': device_id, 'Client-Id': client_id,
               'Content-Type': 'application/json'}
    body = DEVICE_OTA['body'] or json.dumps({'application': {'name': 'xiaozhi', 'version': version}}).encode()
    with urllib.request.urlopen(urllib.request.Request(CLOUD_OTA_URL, body, headers), timeout=10) as r:
        res = json.loads(r.read())
    if res.get('activation'):
        raise RuntimeError('ネットのほうで、スタックちゃんの登録が要ります（アプリで登録し直す）')
    ws = res.get('websocket') or {}
    if not ws.get('url'):
        raise RuntimeError('ネットの返事に WebSocket の行き先がありません')
    return ws['url'], ws.get('token', '')


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


def _get(url, timeout=8):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _plain(s):
    return html.unescape(re.sub(r'<[^>]+>', '', s)).strip()


def web_search(query, n=5):
    """ネットの XiaoZhi に頼まれて調べる（Google ニュースの見出しと日付・DuckDuckGo の説明）。文字にして返す。"""
    q = urllib.parse.quote(query)
    out = []
    try:
        root = ET.fromstring(_get(f'https://news.google.com/rss/search?q={q}&hl=ja&gl=JP&ceid=JP:ja'))
        for item in root.iter('item'):
            if len(out) >= n:
                break
            out.append(f'ニュース（{item.findtext("pubDate", "")[:16]}）: {item.findtext("title", "")}')
    except Exception as e:  # noqa: BLE001 - 片方だけでも答える
        log(f'調べもの（ニュース）のエラー: {e}')
    try:
        page = _get(f'https://html.duckduckgo.com/html/?q={q}').decode('utf-8', 'replace')
        titles = re.findall(r'class="result__a"[^>]*>(.*?)</a>', page, re.S)
        snips = re.findall(r'class="result__snippet"[^>]*>(.*?)</a>', page, re.S)
        out += [f'ウェブ: {_plain(t)}: {_plain(x)}' for t, x in zip(titles, snips)][:n]
    except Exception as e:  # noqa: BLE001
        log(f'調べもの（ウェブ）のエラー: {e}')
    log(f'調べもの「{query}」: {len(out)} 件')
    return '\n'.join(out) if out else '見つかりませんでした'


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
    # スタックちゃんの HttpClient は相手から先に切られると落ちることがあるので、
    # スタックちゃんとの通信は「Connection: close」でも向こうが切るのを待つ。
    protocol_version = 'HTTP/1.1'
    timeout = 30

    def parse_request(self):
        ok = super().parse_request()
        if ok and self.headers.get('Device-Id'):
            self.close_connection = False
        return ok

    def _send_json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_body(self, body, ctype):
        body = body.encode() if isinstance(body, str) else body
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, code, detail):
        body = json.dumps({'detail': detail}, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self, limit):
        if self.headers.get('Transfer-Encoding', '').lower() == 'chunked':
            out = b''
            while (size := int(self.rfile.readline().split(b';')[0].strip() or b'0', 16)):
                out += self.rfile.read(size)
                self.rfile.readline()
                if len(out) > limit:
                    raise ValueError('大きすぎます')
            while self.rfile.readline() not in (b'\r\n', b'\n', b''):
                pass
            return out
        n = int(self.headers.get('Content-Length') or 0)
        if n > limit:
            raise ValueError('大きすぎます')
        return self.rfile.read(n) if n else b''

    def do_GET(self):
        path = self.path.split('?')[0].rstrip('/')
        if path == '/status':
            with STATUS_LOCK:
                self._send_json({**STATUS, 'now': time.time()})
        elif path == '/photo.jpg':
            if not PHOTO['jpeg']:
                return self._send_error(404, 'まだ写真がありません')
            self._send_body(PHOTO['jpeg'], 'image/jpeg')
        elif path == FRAME_PATH:
            set_status(stream_at=time.time())
            self._send_body(want_frames(), 'text/plain')
        elif path == FRAME_PATH + '.jpg':
            FRAME['viewer_at'] = time.time()
            if not FRAME['jpeg'] or time.time() - (STATUS['frame_at'] or 0) > FRAME_STALE:
                return self._send_error(404, '映像を待っています')
            self._send_body(FRAME['jpeg'], 'image/jpeg')
        else:
            self._reply()

    def do_POST(self):
        path = self.path.split('?')[0].rstrip('/')
        if path == '/vision':
            self._vision()
        elif path == '/photo':
            self._photo()
        elif path == FRAME_PATH:
            self._frame()
        elif path == SHOT_PATH:
            self._shot()
        elif path == '/enroll':
            self._enroll()
        elif path == STREAM_PATH:
            on = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('on', ['0'])[0] == '1'
            FRAME.update(on=on, viewer_at=time.time())
            set_status(stream_on=on)
            log(f'映像: {"撮る" if on else "止める"}')
            self._send_json({'ok': True, 'stream_on': on})
        else:
            self._reply()

    def _vision(self):
        """スタックちゃんが撮った写真（multipart の file）を受け取る。"""
        try:
            jpeg = jpeg_from_multipart(self._read_body(PHOTO_MAX_BYTES), self.headers.get('Content-Type'))
        except ValueError as e:
            return self._send_error(413, str(e))
        if not jpeg:
            log('写真: JPEG がありません')
            return self._send_json({'success': False, 'message': 'no jpeg'})
        PHOTO['jpeg'] = jpeg
        set_status(photo_at=time.time())
        log(f'写真: {len(jpeg) // 1024} KB')
        self._send_json({'success': True, 'result': 'ダッシュボードに出しました'})

    def _frame(self):
        """スタックちゃんが続けて送ってくる映像の 1 コマ（image/jpeg）。返事の「1」で次を送ってもらう。"""
        try:
            jpeg = self._read_body(PHOTO_MAX_BYTES)
        except ValueError as e:
            return self._send_error(413, str(e))
        now = time.time()
        if jpeg.startswith(b'\xff\xd8'):
            FRAME['jpeg'] = jpeg
            t = FRAME['times'] = [x for x in FRAME['times'][-9:] if now - x < FRAME_STALE] + [now]
            fps = round((len(t) - 1) / (t[-1] - t[0]), 1) if len(t) > 1 else None
            set_status(stream_at=now, frame_at=now, fps=fps)
            if WATCH['interval'] > 0 and not WATCH['busy'] and now >= WATCH['next']:
                WATCH.update(busy=True, next=now + WATCH['interval'])
                threading.Thread(target=watch_check, args=(jpeg,), daemon=True).start()
        self._send_body(want_frames(), 'text/plain')

    def _shot(self):
        """「撮影するよ」で数えて撮った写真（image/jpeg）。"""
        try:
            jpeg = self._read_body(PHOTO_MAX_BYTES)
        except ValueError as e:
            return self._send_error(413, str(e))
        if not jpeg.startswith(b'\xff\xd8'):
            return self._send_error(400, 'JPEG ではありません')
        PHOTO['jpeg'] = jpeg
        set_status(photo_at=time.time(), shot=None)
        log(f'撮影: {len(jpeg) // 1024} KB')
        self._send_json({'ok': True})

    def _enroll(self):
        """いちばん新しい写真の顔を、名前を付けて face_id に登録する。"""
        q = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
        name = (q.get('name') or [''])[0].strip()
        if not name or len(name) > NAME_MAX:
            return self._send_error(400, f'名前を 1〜{NAME_MAX} 文字で入れてください')
        if not PHOTO['jpeg']:
            return self._send_error(404, 'まだ写真がありません')
        try:
            res = face_id('/api/face/enroll?name=' + urllib.parse.quote(name), PHOTO['jpeg'])
        except ValueError as e:
            return self._send_error(409, str(e))
        except OSError:
            return self._send_error(503, '顔の見分け（face_id）につながりません')
        with STATUS_LOCK:
            if STATUS['shot']:
                STATUS['shot'] = {**STATUS['shot'], 'name': name}
        mark_greeted(name)
        log(f'顔を登録: {name}（{res.get("samples")} 枚目）')
        self._send_json({'ok': True, 'name': name, 'samples': res.get('samples')})

    def _photo(self):
        """いちばん新しくつながったスタックちゃんに写真を撮ってもらう（終わるまで待つ）。"""
        sess = SESSIONS[-1] if SESSIONS else None
        if not sess or not sess.camera:
            return self._send_error(409, 'スタックちゃんがつながっていません（画面を 1 回さわってください）')
        before = STATUS['photo_at']
        try:
            err = asyncio.run_coroutine_threadsafe(sess.take_photo(), LOOP).result(PHOTO_TIMEOUT + 2)
        except Exception as e:  # noqa: BLE001 - 返事がない・切れた
            err = f'返事がありません（{type(e).__name__}）'
        if STATUS['photo_at'] == before:
            log(f'写真に失敗: {err}')
            return self._send_error(502, err or '写真が届きませんでした')
        with STATUS_LOCK:
            self._send_json({'ok': True, 'photo_at': STATUS['photo_at']})

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
        if self.headers.get('Client-Id'):
            save_device_ota({k: self.headers[k] for k in ('Activation-Version', 'Serial-Number', 'User-Agent',
                                                          'Accept-Language') if self.headers.get(k)}, info)
        set_device(id=dev, ip=self.client_address[0], version=version)
        set_status(last_ota=time.time())
        log(f'OTA: {dev} 版 {version or "?"} → ws://{host}:{self.ws_port}')
        self._send_json({
            'server_time': {'timestamp': int(time.time() * 1000), 'timezone_offset': 540},
            'websocket': {'url': f'ws://{host}:{self.ws_port}/xiaozhi/v1/', 'token': '', 'version': 1},
        })

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
        self.camera = False
        self.face = False
        self.shot = False
        self.shooting = False
        self.vision_url = ''
        # ネットの XiaoZhi も MCP の id を 1 から使うので、こちらの id はぶつからない所から始める
        self.mcp_id = 10000
        self.mcp_wait = {}
        self.device_id = ws.request.headers.get('Device-Id', '')
        self.client_id = ws.request.headers.get('Client-Id', '')
        self.hello = None
        self.cloud = None
        self.cloud_sid = ''
        self.search_added = False
        self.switching = False
        self.turn = {}

    async def send(self, **msg):
        msg['session_id'] = self.sid
        await self.ws.send(json.dumps(msg, ensure_ascii=False))

    def mcp_call(self, method, params):
        self.mcp_id += 1
        fut = asyncio.get_running_loop().create_future()
        self.mcp_wait[self.mcp_id] = fut
        payload = {'jsonrpc': '2.0', 'id': self.mcp_id, 'method': method, 'params': params}
        return fut, self.send(type='mcp', payload=payload)

    async def mcp_init(self, vision_url):
        fut, sending = self.mcp_call('initialize', {
            'protocolVersion': '2024-11-05', 'capabilities': {'vision': {'url': vision_url, 'token': ''}},
            'clientInfo': {'name': 'stackchan_bridge', 'version': '1'}})
        await sending
        try:
            await asyncio.wait_for(fut, 10)
        except asyncio.TimeoutError:
            log('MCP: initialize の返事がありません（カメラは使えません）')
            return
        fut, sending = self.mcp_call('tools/list', {'withUserTools': True})
        await sending
        try:
            tools = (await asyncio.wait_for(fut, 10)).get('result', {}).get('tools', [])
        except asyncio.TimeoutError:
            tools = []
        names = {t.get('name') for t in tools}
        self.camera = PHOTO_TOOL in names
        self.face = FACE_TOOL in names
        self.shot = SHOT_TOOL in names
        set_status(camera=self.camera, shot_tool=self.shot)
        log(f'MCP: カメラ {"あり" if self.camera else "なし"} → {vision_url}、顔の色 {"あり" if self.face else "なし"}')

    async def after_hello(self, vision_url):
        self.vision_url = vision_url
        if vision_url:
            await self.mcp_init(vision_url)
        if STATUS['mode'] == 'net':
            await self.to_net(announce=False)
        else:
            await self.set_face('local')

    async def set_face(self, mode):
        if not self.face:
            return
        fut, sending = self.mcp_call('tools/call', {'name': FACE_TOOL, 'arguments': {'color': FACE_COLOR[mode]}})
        await sending
        try:
            await asyncio.wait_for(fut, 5)
        except asyncio.TimeoutError:
            log('MCP: 顔の色の返事がありません')

    async def say(self, text):
        """Jetson の声で 1 文だけ話す（切り替えのお知らせ）。"""
        a = self.args
        try:
            pcm = await asyncio.to_thread(tts_pcm, text, a.voice, a.pitch, a.peak_db)
        except Exception as e:  # noqa: BLE001 - 声が作れなくても切り替えは続ける
            log(f'声のエラー: {e}')
            return
        self.cancel.clear()
        await self.send(type='tts', state='start')
        await self.send(type='tts', state='sentence_start', text=text)
        await self.play(pcm)
        await self.send(type='tts', state='stop')

    def can_greet(self):
        """会話・撮影・切り替えのじゃまにならないときだけ True。"""
        return not (self.reply_task or self.shooting or self.switching or self.turn
                    or STATUS['state'] in ('thinking', 'speaking'))

    async def greet(self, names):
        if not self.can_greet():
            return
        for n in names:
            mark_greeted(n)
        words = f'{"、".join(n + "さん" for n in names)}、{greeting_word()}！'
        log(f'あいさつ: {words}')
        set_status(greet={'at': time.time(), 'names': names})
        await self.say(words)

    async def to_net(self, announce=True):
        """ネットの XiaoZhi につなぎ、このセッションの声と文字を中継する。だめならローカルのまま。"""
        self.switching = True
        cloud = None
        try:
            if announce:
                await self.say('ネットにつなぎます。')
            url, token = await asyncio.to_thread(cloud_ota, self.device_id, self.client_id)
            cloud = await connect(url, open_timeout=10, max_size=2 ** 20, additional_headers={
                'Authorization': f'Bearer {token}', 'Protocol-Version': '1',
                'Device-Id': self.device_id, 'Client-Id': self.client_id})
            await cloud.send(json.dumps(self.hello))
            hello = json.loads(await asyncio.wait_for(cloud.recv(), 10))
            rate = (hello.get('audio_params') or {}).get('sample_rate')
            if hello.get('type') != 'hello' or rate not in (None, OUT_RATE):
                raise RuntimeError(f'ネットの hello がちがいます: {hello}')
            self.cloud_sid = hello.get('session_id', '')
            self.search_added = False
            self.cloud = cloud
            save_mode('net')
            await self.set_face('net')
            if self.listening:
                await self.to_cloud({'type': 'listen', 'state': 'start', 'mode': self.mode})
            asyncio.create_task(self.relay_cloud(cloud))
            set_status(state='listening' if self.listening else 'idle')
            log(f'ネットにつなぎました: {url}')
        except Exception as e:  # noqa: BLE001 - ネットにつながらなくてもローカルで話せるようにする
            log(f'ネットにつながりません: {e}')
            self.cloud = None
            if cloud:
                await cloud.close()
            save_mode('local')
            await self.set_face('local')
            await self.say('ネットにつながりませんでした。ローカルで話します。')
        finally:
            self.switching = False

    async def to_local(self, cloud):
        self.cloud = None
        self.switching = True
        try:
            await cloud.close()
            save_mode('local')
            if self.vision_url:
                await self.mcp_init(self.vision_url)
            await self.set_face('local')
            await self.say('ローカルにもどりました。')
            log('ローカルにもどりました')
        finally:
            self.switching = False
            set_status(state='listening' if self.listening else 'idle')

    async def to_cloud(self, msg):
        try:
            await self.cloud.send(json.dumps({**msg, 'session_id': self.cloud_sid}, ensure_ascii=False))
        except (ConnectionClosed, AttributeError):
            pass

    async def cloud_audio(self, data):
        try:
            await self.cloud.send(data)
        except (ConnectionClosed, AttributeError):
            pass

    async def relay_cloud(self, cloud):
        """ネットからの声と文字をスタックちゃんへ流す。「ローカルにして」が聞こえたらもどる。"""
        hold = None
        try:
            async for m in cloud:
                if cloud is not self.cloud:
                    return
                if isinstance(m, bytes):
                    if self.shooting:
                        continue
                    if hold is None:
                        await self.ws.send(m)
                        continue
                    hold.append(m)
                    if len(hold) >= CLOUD_HOLD:
                        for f in hold:
                            await self.ws.send(f)
                        hold = None
                    continue
                try:
                    msg = json.loads(m)
                except ValueError:
                    continue
                if msg.get('type') == 'hello':
                    continue
                call = (msg.get('payload') or {}) if msg.get('type') == 'mcp' else {}
                if call.get('method') == 'tools/call' and (call.get('params') or {}).get('name') == SEARCH_TOOL:
                    asyncio.create_task(self.cloud_search(call))
                    continue
                if self.shooting and msg.get('type') in ('tts', 'llm'):
                    continue
                if msg.get('type') == 'tts' and msg.get('state') == 'start':
                    hold = []
                elif msg.get('type') == 'tts' and msg.get('state') == 'stop' and hold:
                    for f in hold:
                        await self.ws.send(f)
                    hold = None
                msg['session_id'] = self.sid
                await self.ws.send(json.dumps(msg, ensure_ascii=False))
                self.note_cloud(msg)
                if msg.get('type') == 'stt' and is_switch(TO_LOCAL, msg.get('text')):
                    await self.to_local(cloud)
                    return
                if msg.get('type') == 'stt' and not self.shooting and is_switch(SHOOT, msg.get('text')):
                    self.shooting = True
                    hold = None
                    await self.to_cloud({'type': 'abort'})
                    asyncio.create_task(self.countdown_shot())
        except ConnectionClosed:
            pass
        except Exception as e:  # noqa: BLE001 - 中継の失敗はつなぎ直しで直す
            log(f'ネットの中継のエラー: {e}')
        if cloud is self.cloud:
            log('ネットの会話が切れました（画面を 1 回さわると、またつながります）')
            self.cloud = None
            await self.ws.close()

    async def cloud_search(self, call):
        """ネットの XiaoZhi が呼んだ調べものに、スタックちゃんのかわりに答える。"""
        query = str(((call.get('params') or {}).get('arguments') or {}).get('query', ''))
        found = await asyncio.to_thread(web_search, query)
        await self.to_cloud({'type': 'mcp', 'payload': {'jsonrpc': '2.0', 'id': call.get('id'), 'result': {
            'content': [{'type': 'text', 'text': found}], 'isError': False}}})

    def note_cloud(self, msg):
        """ネットの会話の様子をダッシュボード用に残す。"""
        t, now = msg.get('type'), time.time()
        if t == 'stt':
            self.turn = {'at': now, 'heard': msg.get('text', ''), 'reply': [], 'voice': None}
            set_status(state='thinking')
        elif t == 'tts' and msg.get('state') == 'sentence_start' and self.turn:
            if self.turn['voice'] is None:
                self.turn['voice'] = now - self.turn['at']
                set_status(state='speaking')
            self.turn['reply'].append(msg.get('text', ''))
        elif t == 'tts' and msg.get('state') == 'stop':
            if self.turn:
                v = self.turn['voice']
                with STATUS_LOCK:
                    STATUS['turns'] += 1
                    STATUS['last'] = {'at': self.turn['at'], 'heard': self.turn['heard'],
                                      'reply': ''.join(self.turn['reply']), 'stt_s': None,
                                      'voice_s': v and round(v, 2)}
            self.turn = {}
            set_status(state='listening' if self.listening else 'idle')

    async def look_up(self):
        """撮影の首の向き。待ち受けの首ふりで動かされても、数えているあいだ 1 秒ごとにもどす。"""
        head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': {
            'yaw': 0, 'pitch': self.args.shot_pitch, 'speed': 300}})
        await sending
        head.cancel()

    async def countdown_shot(self):
        """「撮影するよ」: 正面を向いて SHOT_SECONDS 秒数えて撮ってもらい、だれの顔かを見る。"""
        self.shooting = True
        set_status(shooting=True)
        try:
            if not self.shot:
                await self.say('ごめんね。今のプログラムでは、撮影できないよ。')
                return
            await self.say('撮影するよ。こっちを向いてね。')
            before = STATUS['photo_at']
            fut, sending = self.mcp_call('tools/call', {'name': SHOT_TOOL, 'arguments': {'seconds': SHOT_SECONDS}})
            await sending
            start = time.time()
            deadline = start + SHOT_SECONDS + PHOTO_TIMEOUT
            next_head = start
            while STATUS['photo_at'] == before and time.time() < deadline:
                if next_head is not None and time.time() >= next_head:
                    await self.look_up()
                    next_head += 1.0
                    if next_head > start + SHOT_SECONDS:
                        next_head = None
                await asyncio.sleep(0.2)
            fut.cancel()
            self.mcp_wait = {k: v for k, v in self.mcp_wait.items() if v is not fut}
            if STATUS['photo_at'] == before:
                log('撮影: 写真が届きませんでした')
                await self.say('ごめんね。うまく撮れなかったよ。')
                return
            shot, words = await asyncio.to_thread(face_check, PHOTO['jpeg'])
            set_status(shot=shot)
            mark_greeted(shot['name'])
            log(f'撮影: 顔 {shot["faces"]} 個、{shot["name"] or shot["error"] or "知らない人"}')
            await self.say(words)
        finally:
            self.shooting = False
            set_status(shooting=False)

    async def take_photo(self):
        """写真を撮ってもらう。うまくいけば None、だめなら理由。"""
        fut, sending = self.mcp_call('tools/call', {'name': PHOTO_TOOL,
                                                    'arguments': {'question': 'ダッシュボードに写真を出す'}})
        await sending
        res = await asyncio.wait_for(fut, PHOTO_TIMEOUT)
        if 'error' in res:
            return str(res['error'].get('message', res['error']))
        r = res.get('result') or {}
        return ''.join(c.get('text', '') for c in r.get('content', [])) if r.get('isError') else None

    def on_mcp(self, payload):
        """こちらが出した MCP の返事なら受け取って True。"""
        fut = self.mcp_wait.pop(payload.get('id'), None) if isinstance(payload, dict) else None
        if fut and not fut.done():
            fut.set_result(payload)
        return fut is not None

    def reset_vad(self):
        self.speech, self.pre, self.started, self.silent = [], [], False, 0.0

    async def on_text(self, msg):
        t = msg.get('type')
        if t == 'mcp' and self.on_mcp(msg.get('payload')):
            return
        if self.cloud and t != 'hello':
            tools = ((msg.get('payload') or {}).get('result') or {}).get('tools') if t == 'mcp' else None
            if isinstance(tools, list) and not self.search_added:
                tools.append({'name': SEARCH_TOOL, 'description': SEARCH_DESC, 'inputSchema': SEARCH_PARAMS})
                self.search_added = True
            if t == 'listen' and msg.get('state') in ('start', 'stop'):
                self.listening = msg['state'] == 'start'
                self.mode = msg.get('mode', self.mode)
            await self.to_cloud(msg)
            return
        if t == 'hello':
            self.hello = msg
            await self.ws.send(json.dumps({
                'type': 'hello', 'transport': 'websocket', 'session_id': self.sid,
                'audio_params': {'format': 'opus', 'sample_rate': OUT_RATE, 'channels': 1,
                                 'frame_duration': FRAME_MS}}))
            vision_url = None
            if (msg.get('features') or {}).get('mcp'):
                host = (self.ws.request.headers.get('Host') or '127.0.0.1').rsplit(':', 1)[0]
                vision_url = f'http://{host}:{self.args.ota_port}/vision'
            asyncio.create_task(self.after_hello(vision_url))
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
        elif t == 'mcp':
            pass
        else:
            log(f'未対応のメッセージ: {msg}')

    def on_audio(self, data):
        if not self.listening or self.reply_task or self.switching:
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
            if is_switch(TO_NET, text):
                await self.to_net()
                return
            if is_switch(TO_LOCAL, text):
                await self.say('今はローカルです。')
                return
            if is_switch(SHOOT, text):
                await self.countdown_shot()
                return
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
    SESSIONS.append(s)
    try:
        async for msg in ws:
            if isinstance(msg, bytes):
                if s.cloud:
                    await s.cloud_audio(msg)
                else:
                    s.on_audio(msg)
            else:
                try:
                    await s.on_text(json.loads(msg))
                except ValueError:
                    log(f'JSON でないメッセージ: {msg[:80]}')
    finally:
        s.cancel.set()
        cloud, s.cloud = s.cloud, None
        if cloud:
            await cloud.close()
        SESSIONS.remove(s)
        for fut in s.mcp_wait.values():
            fut.cancel()
        set_status(camera=any(x.camera for x in SESSIONS))
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
    p.add_argument('--shot-pitch', type=int, default=30,
                   help='「撮影するよ」で数えるときの首の上向きの角度（0〜90 度、0 が水平）')
    p.add_argument('--watch-interval', type=float, default=2.0,
                   help='見守りで顔を見る間隔（秒）。0 で見守りをしない')
    args = p.parse_args()

    global LOOP
    LOOP = asyncio.get_running_loop()
    OtaHandler.ws_port = args.ws_port
    WATCH['interval'] = max(0.0, args.watch_interval)
    load_device_ota()
    set_status(mode=load_mode(), watch_on=WATCH['interval'] > 0,
               engines={'stt': 'whisper-server', 'llm': args.model, 'tts': f'Kokoro {args.voice}'})
    ota = ThreadingHTTPServer((args.host, args.ota_port), OtaHandler)
    threading.Thread(target=ota.serve_forever, daemon=True).start()
    async with serve(lambda ws: handler(ws, args), args.host, args.ws_port, max_size=2 ** 20):
        log(f'待ち受け: OTA http://{args.host}:{args.ota_port}/xiaozhi/ota/  WebSocket :{args.ws_port}/xiaozhi/v1/')
        await asyncio.Future()


if __name__ == '__main__':
    asyncio.run(main())
