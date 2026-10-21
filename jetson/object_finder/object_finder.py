"""声で覚えた物を、AI-CAR のカメラ画像から探す（Jetson、NanoOWL の「画像で探す」）。

登録: カメラの前のまん中に置いた物の見た目（NanoOWL の枠 1 つ分の特徴）を名前といっしょに保存する。場所は保存しない。
探す: 1 秒に --rate 回 AI-CAR の /api/camera/snapshot を見て、その特徴に似た枠の点（0〜1）を出す。
      --threshold 以上が --window 回中 --need 回あれば「見つけた」。車を止める・話すのは stackchan_bridge。

HTTP（--host:--port、ふつうは 127.0.0.1:8095。Jetson の中だけ）:
  GET  /objects               覚えている名前
  POST /register {"name"}     今の画像のまん中の物を覚える
  POST /search {"name"}       探しはじめる（前の探しはやめる）
  GET  /search                探しているようす {"name", "active", "found", "score", "box", "frames"}
  POST /search/stop           探すのをやめる
"""
# 先に transformers と nanoowl を読みこむ（torch を先に読むと、このコンテナでは regex の読みこみで落ちることがある）
import transformers  # noqa: F401,I001
from nanoowl.owl_predictor import OwlPredictor

import argparse
import io
import json
import os
import re
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import PIL.Image
import torch
import torch.nn.functional as F

# 登録のとき「物らしさ」を測る言葉（いちばん高い点を使う）
OBJECT_PROMPTS = ['a photo of an object', 'a bottle', 'a cup', 'a toy', 'a box', 'a bag', 'a can']


def fetch(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as res:
        return PIL.Image.open(io.BytesIO(res.read())).convert('RGB')


def slug(name):
    return re.sub(r'[^\w]', '_', name).strip('_')[:40] or 'object'


class Finder:
    def __init__(self, args):
        self.args = args
        self.predictor = OwlPredictor(args.model, image_encoder_engine=args.engine)
        self.object_text = F.normalize(self.predictor.encode_text(OBJECT_PROMPTS).text_embeds, dim=-1)
        self.snapshot_url = args.car_url.rstrip('/') + '/api/camera/snapshot'
        self.lock = threading.Lock()      # 推論は 1 つずつ
        self.state_lock = threading.Lock()
        self.state = {'name': None, 'active': False, 'found': False, 'score': None, 'box': None, 'frames': 0}
        self.search_id = 0
        os.makedirs(args.data, exist_ok=True)

    def encode(self, image):
        """枠ごとの (特徴 [N, D]（長さ 1）、枠 [N, 4]（画素 xyxy）、logit_shift [N]、logit_scale [N])。"""
        p = self.predictor
        with self.lock, torch.no_grad():
            t = p.image_preprocessor.preprocess_pil_image(image)
            rois = torch.tensor([[0, 0, image.width, image.height]], dtype=t.dtype, device=t.device)
            e = p.encode_rois(t, rois, pad_square=True)
            return (F.normalize(e.image_class_embeds[0], dim=-1), e.pred_boxes[0],
                    e.logit_shift[0, :, 0], e.logit_scale[0, :, 0])

    def objects(self):
        path = os.path.join(self.args.data, 'objects.json')
        try:
            with open(path, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def save_objects(self, objs):
        path = os.path.join(self.args.data, 'objects.json')
        with open(path + '.tmp', 'w', encoding='utf-8') as f:
            json.dump(objs, f, ensure_ascii=False, indent=1)
        os.replace(path + '.tmp', path)

    def pick_center(self, image):
        """画面のまん中にあって、はしにつかない、物らしい枠をえらぶ。(番号, 特徴, 枠, 物らしさ)。"""
        emb, box, shift, scale = self.encode(image)
        w, h = image.width, image.height
        cx, cy = (box[:, 0] + box[:, 2]) / 2, (box[:, 1] + box[:, 3]) / 2
        area = ((box[:, 2] - box[:, 0]) * (box[:, 3] - box[:, 1])) / (w * h)
        mx, my = w * 0.02, h * 0.02
        ok = ((box[:, 0] > mx) & (box[:, 1] > my) & (box[:, 2] < w - mx) & (box[:, 3] < h - my)
              & ((cx - w / 2).abs() < w * 0.2) & ((cy - h / 2).abs() < h * 0.3)
              & (area > 0.01) & (area < 0.45))
        obj = torch.sigmoid((emb @ self.object_text.T + shift[:, None]) * scale[:, None]).max(-1).values
        if not bool(ok.any()):
            return None
        i = int(torch.where(ok, obj, torch.full_like(obj, -1.0)).argmax())
        return i, emb[i], [round(float(v)) for v in box[i].tolist()], float(obj[i])

    def register(self, name, image=None):
        image = image or fetch(self.snapshot_url)
        picked = self.pick_center(image)
        if picked is None:
            return {'ok': False, 'error': 'まん中に物が見つかりません'}
        _, emb, box, objness = picked
        key = slug(name)
        torch.save(emb.cpu(), os.path.join(self.args.data, key + '.pt'))
        image.crop(box).save(os.path.join(self.args.data, key + '.jpg'), quality=90)
        objs = self.objects()
        objs[name] = {'file': key, 'box': box, 'objectness': round(objness, 3), 'at': time.time()}
        self.save_objects(objs)
        print(f'登録: {name} 枠 {box} 物らしさ {objness:.2f}', flush=True)
        return {'ok': True, 'name': name, 'box': box, 'objectness': round(objness, 3)}

    def score(self, query, image):
        emb, box, shift, scale = self.encode(image)
        s = torch.sigmoid((emb @ query + shift) * scale)
        i = int(s.argmax())
        return float(s[i]), [round(float(v)) for v in box[i].tolist()]

    def start_search(self, name):
        objs = self.objects()
        if name not in objs:
            return {'ok': False, 'error': f'{name} は覚えていません'}
        query = torch.load(os.path.join(self.args.data, objs[name]['file'] + '.pt')).cuda()
        with self.state_lock:
            self.search_id += 1
            sid = self.search_id
            self.state = {'name': name, 'active': True, 'found': False, 'score': None, 'box': None,
                          'frames': 0, 'started': time.time()}
        threading.Thread(target=self.search_loop, args=(sid, query), daemon=True).start()
        print(f'探す: {name}', flush=True)
        return {'ok': True, 'name': name}

    def stop_search(self):
        with self.state_lock:
            self.search_id += 1
            self.state['active'] = False
        return {'ok': True}

    def search_loop(self, sid, query):
        a = self.args
        hits = []
        interval = 1.0 / max(a.rate, 0.1)
        while True:
            t0 = time.monotonic()
            with self.state_lock:
                if sid != self.search_id:
                    return
            try:
                score, box = self.score(query, fetch(self.snapshot_url))
            except Exception as e:  # noqa: BLE001 - 画像が来なければ次の回にまた見る
                print(f'画像のエラー: {e}', flush=True)
                time.sleep(1.0)
                continue
            hits = (hits + [score >= a.threshold])[-a.window:]
            found = sum(hits) >= a.need
            with self.state_lock:
                if sid != self.search_id:
                    return
                self.state.update(score=round(score, 3), box=box, frames=self.state['frames'] + 1)
                if found:
                    self.state.update(found=True, active=False)
                    print(f'見つけた: {self.state["name"]} 点 {score:.3f} 枠 {box}', flush=True)
                    return
            time.sleep(max(0.0, interval - (time.monotonic() - t0)))

    def status(self):
        with self.state_lock:
            return dict(self.state)


class Handler(BaseHTTPRequestHandler):
    finder = None

    def _send(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        try:
            return json.loads(self.rfile.read(min(n, 4096)) or b'{}')
        except ValueError:
            return {}

    def do_GET(self):
        if self.path == '/objects':
            self._send(self.finder.objects())
        elif self.path == '/search':
            self._send(self.finder.status())
        else:
            self._send({'error': 'not found'}, 404)

    def do_POST(self):
        f = self.finder
        if self.path == '/search/stop':
            self._send(f.stop_search())
            return
        name = str(self._body().get('name') or '').strip()
        if not name or self.path not in ('/register', '/search'):
            self._send({'ok': False, 'error': 'name がありません'}, 400)
            return
        try:
            res = f.register(name) if self.path == '/register' else f.start_search(name)
        except Exception as e:  # noqa: BLE001 - 失敗はそのまま返す
            res = {'ok': False, 'error': str(e)}
        self._send(res, 200 if res.get('ok') else 409)

    def log_message(self, fmt, *a):
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--car-url', default=os.environ.get('AI_CAR_URL', 'http://100.70.35.31:8080'))
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=8095)
    p.add_argument('--data', default='/data/objects', help='覚えた物（特徴 .pt・写真 .jpg・objects.json）')
    p.add_argument('--threshold', type=float, default=0.9, help='この点以上なら「似ている」')
    p.add_argument('--window', type=int, default=3)
    p.add_argument('--need', type=int, default=2, help='--window 回中この回数「似ている」なら見つけた')
    p.add_argument('--rate', type=float, default=5.0, help='探すとき 1 秒に見る回数の上限')
    p.add_argument('--model', default='google/owlvit-base-patch32')
    p.add_argument('--engine', default='/opt/nanoowl/data/owl_image_encoder_patch32.engine')
    p.add_argument('--test-register', nargs=2, metavar=('NAME', 'IMAGE'),
                   help='画像ファイルで登録だけ試して終わる')
    p.add_argument('--test-score', nargs=2, metavar=('NAME', 'IMAGE'),
                   help='画像ファイルで点を出して終わる')
    args = p.parse_args()
    finder = Finder(args)
    if args.test_register:
        print(json.dumps(finder.register(args.test_register[0], PIL.Image.open(args.test_register[1]).convert('RGB')),
                         ensure_ascii=False), flush=True)
        os._exit(0)
    if args.test_score:
        objs = finder.objects()
        q = torch.load(os.path.join(args.data, objs[args.test_score[0]]['file'] + '.pt')).cuda()
        t0 = time.perf_counter()
        s, b = finder.score(q, PIL.Image.open(args.test_score[1]).convert('RGB'))
        print(json.dumps({'score': round(s, 3), 'box': b, 'ms': round((time.perf_counter() - t0) * 1000)}), flush=True)
        os._exit(0)
    Handler.finder = finder
    print(f'物さがし 開始: http://{args.host}:{args.port}  画像 {finder.snapshot_url}  覚えた物 {list(finder.objects())}',
          flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
