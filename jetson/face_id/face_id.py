"""Jetson で顔を見つけて、登録した家族のだれかを見分ける。

顔を見つけるのは YuNet、顔の特ちょう（128 個の数字）を出すのは SFace（どちらも OpenCV の
FaceDetectorYN / FaceRecognizerSF、CPU）。登録した人の特ちょうとの近さ（cos）がいちばん高い人を選び、
match_threshold より低いときは「知らない人」（name: null）にする。

顔の画像は保存しない。people.json には名前と特ちょうだけを書く。

入口（AI_CAR_API_TOKEN を設定したときは、/api は X-API-Token が要る）
  GET    /                      登録・確認の画面（face_id.html）
  GET    /api/face/status       登録した人と、最後の結果。呼ばれているあいだだけ AI-CAR のカメラを見る
  GET    /api/face/frame.jpg    最後に見た画像
  POST   /api/face/recognize    JPEG を送ると、顔とだれかを返す（スタックちゃん用）
  POST   /api/face/enroll?name= JPEG を送るか、空なら AI-CAR のカメラの今の画像で、1 人の顔を登録する
  DELETE /api/face/people?name= その人の登録を消す
"""

import argparse
import hmac
import json
import os
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_BODY = 4 * 1024 * 1024
MAX_SAMPLES = 20
DETECT_WIDTH = 640


class FaceEngine:
    def __init__(self, det_path, rec_path, score_threshold):
        self._det = cv2.FaceDetectorYN.create(det_path, '', (320, 320), score_threshold, 0.3, 20)
        self._rec = cv2.FaceRecognizerSF.create(rec_path, '')
        self._lock = threading.Lock()

    def faces(self, image):
        """顔ごとに（枠 [x0, y0, x1, y1] 0〜1、元の画像での幅 px、確からしさ、特ちょう）を返す。"""
        h, w = image.shape[:2]
        scale = min(1.0, DETECT_WIDTH / w)
        small = cv2.resize(image, (round(w * scale), round(h * scale))) if scale < 1.0 else image
        sh, sw = small.shape[:2]
        out = []
        with self._lock:
            self._det.setInputSize((sw, sh))
            _, found = self._det.detect(small)
            for row in (found if found is not None else []):
                feat = self._rec.feature(self._rec.alignCrop(small, row)).flatten().astype(np.float32)
                feat /= float(np.linalg.norm(feat)) or 1.0
                x, y, bw, bh = (float(v) for v in row[:4])
                box = [max(0.0, x / sw), max(0.0, y / sh),
                       min(1.0, (x + bw) / sw), min(1.0, (y + bh) / sh)]
                out.append((box, bw / scale, float(row[14]), feat))
        return out


class People:
    """名前 → 特ちょうのリスト。people.json に保存する（このユーザーだけが読める）。"""

    def __init__(self, path):
        self._path = path
        self._lock = threading.Lock()
        self._people = {}
        if os.path.exists(path):
            with open(path, encoding='utf-8') as f:
                self._people = {k: [np.asarray(v, np.float32) for v in vs]
                                for k, vs in json.load(f).items()}

    def _save(self):
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        tmp = self._path + '.tmp'
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump({k: [[round(float(x), 5) for x in v] for v in vs]
                       for k, vs in self._people.items()}, f, ensure_ascii=False)
        os.replace(tmp, self._path)

    def add(self, name, feat):
        with self._lock:
            samples = self._people.setdefault(name, [])
            samples.append(feat)
            del samples[:-MAX_SAMPLES]
            self._save()
            return len(samples)

    def remove(self, name):
        with self._lock:
            found = self._people.pop(name, None) is not None
            if found:
                self._save()
            return found

    def summary(self):
        with self._lock:
            return [{'name': k, 'samples': len(v)} for k, v in sorted(self._people.items())]

    def match(self, feat, exclude=None):
        """いちばん近い人の名前と近さ（その人の登録の中で最も近いもの）。"""
        best, best_sim = None, -1.0
        with self._lock:
            for name, samples in self._people.items():
                if name == exclude:
                    continue
                sim = max(float(np.dot(feat, s)) for s in samples)
                if sim > best_sim:
                    best, best_sim = name, sim
        return best, best_sim


class FaceService:
    def __init__(self, args):
        self.args = args
        self.engine = FaceEngine(args.detector, args.recognizer, args.score_threshold)
        self.people = People(args.people)
        self.token = os.environ.get('AI_CAR_API_TOKEN', '')
        self.snapshot_url = os.environ.get('AI_CAR_URL', '').rstrip('/') + '/api/camera/snapshot'
        self._lock = threading.Lock()
        self.latest = {'stamp': None, 'source': None, 'faces': [], 'note': ''}
        self.latest_jpeg = None
        self.last_view = 0.0

    def recognize(self, jpeg, source):
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('JPEG を読めません')
        start = time.time()
        faces = []
        for box, width_px, score, feat in self.engine.faces(image):
            name, sim = self.people.match(feat)
            faces.append({
                'box': [round(v, 4) for v in box],
                'width_px': round(width_px),
                'score': round(score, 3),
                'name': name if sim >= self.args.match_threshold else None,
                'nearest': name,
                'similarity': round(sim, 3) if name else None,
            })
        result = {'stamp': time.time(), 'source': source, 'faces': faces,
                  'elapsed_ms': round((time.time() - start) * 1000, 1),
                  'size': [image.shape[1], image.shape[0]], 'note': ''}
        with self._lock:
            self.latest, self.latest_jpeg = result, jpeg
        return result, image

    def enroll(self, name, jpeg):
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('JPEG を読めません')
        faces = self.engine.faces(image)
        if len(faces) != 1:
            raise ValueError(f'顔が 1 つだけ写っていません（{len(faces)} 個）')
        _, width_px, _, feat = faces[0]
        if width_px < self.args.min_face_px:
            raise ValueError(f'顔が小さすぎます（{round(width_px)} px、{self.args.min_face_px} px 以上）。近づいてください')
        other, sim = self.people.match(feat, exclude=name)
        if other is not None and sim >= self.args.match_threshold:
            raise ValueError(f'{other} さんの顔に近すぎます（近さ {sim:.2f}）。名前がちがわないか確かめてください')
        return self.people.add(name, feat)

    def fetch_snapshot(self):
        with urllib.request.urlopen(self.snapshot_url, timeout=3.0) as res:
            return res.read()

    def camera_loop(self):
        """画面が開いているあいだだけ、AI-CAR のカメラを rate 回/秒 見る。"""
        while True:
            if time.time() - self.last_view < 15.0 and self.snapshot_url.startswith('http'):
                try:
                    self.recognize(self.fetch_snapshot(), 'ai_car')
                except Exception as exc:  # noqa: BLE001 - カメラが止まっていても続ける
                    with self._lock:
                        self.latest = dict(self.latest, note=f'AI-CAR のカメラ: {exc}')
            time.sleep(1.0 / self.args.rate)


def make_handler(svc):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _authorized(self):
            if not svc.token:
                return True
            if hmac.compare_digest(self.headers.get('X-API-Token', ''), svc.token):
                return True
            self._send(401, {'error': 'API トークンがちがいます'})
            return False

        def _body(self):
            length = int(self.headers.get('Content-Length') or 0)
            if length > MAX_BODY:
                raise ValueError('画像が大きすぎます')
            return self.rfile.read(length) if length else b''

        def _name(self, query):
            name = (query.get('name') or [''])[0].strip()
            if not 1 <= len(name) <= 20 or not name.isprintable():
                raise ValueError('名前は 1〜20 文字にしてください')
            return name

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == '/':
                with open(os.path.join(HERE, 'face_id.html'), 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            if not self._authorized():
                return None
            if url.path == '/api/face/status':
                svc.last_view = time.time()
                return self._send(200, {
                    'people': svc.people.summary(),
                    'latest': svc.latest,
                    'match_threshold': svc.args.match_threshold,
                    'min_face_px': svc.args.min_face_px,
                    'auth_required': bool(svc.token),
                })
            if url.path == '/api/face/frame.jpg':
                if svc.latest_jpeg is None:
                    return self._send(404, {'error': 'まだ画像がありません'})
                return self._send(200, svc.latest_jpeg, 'image/jpeg')
            return self._send(404, {'error': 'not found'})

        def do_POST(self):
            url = urlparse(self.path)
            if not self._authorized():
                return None
            try:
                if url.path == '/api/face/recognize':
                    body = self._body()
                    if not body:
                        raise ValueError('JPEG を送ってください')
                    result, _ = svc.recognize(body, 'upload')
                    return self._send(200, result)
                if url.path == '/api/face/enroll':
                    name = self._name(parse_qs(url.query))
                    body = self._body() or svc.fetch_snapshot()
                    samples = svc.enroll(name, body)
                    return self._send(200, {'name': name, 'samples': samples})
            except ValueError as exc:
                return self._send(400, {'error': str(exc)})
            except OSError as exc:
                return self._send(502, {'error': f'AI-CAR のカメラ: {exc}'})
            return self._send(404, {'error': 'not found'})

        def do_DELETE(self):
            url = urlparse(self.path)
            if not self._authorized():
                return None
            if url.path == '/api/face/people':
                try:
                    name = self._name(parse_qs(url.query))
                except ValueError as exc:
                    return self._send(400, {'error': str(exc)})
                if svc.people.remove(name):
                    return self._send(200, {'name': name, 'removed': True})
                return self._send(404, {'error': f'{name} は登録されていません'})
            return self._send(404, {'error': 'not found'})

    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--detector', default=os.path.join(HERE, 'models', 'face_detection_yunet_2023mar.onnx'))
    p.add_argument('--recognizer', default=os.path.join(HERE, 'models', 'face_recognition_sface_2021dec.onnx'))
    p.add_argument('--people', default=os.path.join(HERE, 'data', 'people.json'))
    p.add_argument('--score-threshold', type=float, default=0.8, help='顔とみなす確からしさ')
    p.add_argument('--match-threshold', type=float, default=0.40, help='同じ人とみなす近さ（cos）')
    p.add_argument('--min-face-px', type=int, default=60, help='登録できる顔の幅の下限')
    p.add_argument('--rate', type=float, default=2.0, help='AI-CAR のカメラを見る回数 / 秒')
    args = p.parse_args()

    svc = FaceService(args)
    if not svc.token:
        print('AI_CAR_API_TOKEN が未設定: 同じネットワークのだれでも登録・削除できます', flush=True)
    threading.Thread(target=svc.camera_loop, daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(svc))
    print(f'face_id: http://{args.host}:{args.port}/ （登録 {len(svc.people.summary())} 人）', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
