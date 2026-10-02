"""Jetson で AI-CAR のカメラ画像を NanoOWL にかけ、物体の名前と位置を AI-CAR へ送る。

AI-CAR の GET /api/camera/snapshot で最新の JPEG を取り、prompts.json の言葉（英語）で
物体を探す。同じ物体に重なった枠は 1 つにまとめ（NMS）、日本語の名前を付けて
POST /api/jetson/detections へ送る。距離は AI-CAR の perception_node が LiDAR から付ける。

dustynv/nanoowl コンテナの中で動かす（jetson-owl.service を参照）。
"""

import argparse
import http.client
import io
import json
import os
import sys
import time
import urllib.error
import urllib.request

import PIL.Image
from nanoowl.owl_predictor import OwlPredictor


def load_prompts(path):
    """prompts.json を読む。label が空の言葉は、まぎらわしい物を吸わせるためだけに使い、送らない。"""
    with open(path, encoding='utf-8') as f:
        items = json.load(f)
    return [str(i['prompt']) for i in items], [str(i.get('label', '')) for i in items]


def overlap(a, b):
    """2 つの枠の IoU と、小さい方の枠が大きい方に入っている割合を返す。"""
    inter_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = inter_w * inter_h
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    smaller = min(area_a, area_b)
    return (inter / union if union > 0 else 0.0), (inter / smaller if smaller > 0 else 0.0)


def nms(candidates, iou_threshold, contain_threshold):
    """スコアの高い順に、残した枠と重なりすぎる枠・ほぼ中に入る枠を捨てる（名前によらない）。"""
    kept = []
    for c in sorted(candidates, key=lambda c: -c['score']):
        if all(i < iou_threshold and r < contain_threshold
               for i, r in (overlap(c['box'], k['box']) for k in kept)):
            kept.append(c)
    return kept


def fetch(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as res:
        return res.read()


def post(url, payload, token, timeout):
    req = urllib.request.Request(
        url, data=json.dumps(payload).encode('utf-8'), method='POST',
        headers={'Content-Type': 'application/json', 'X-API-Token': token})
    with urllib.request.urlopen(req, timeout=timeout) as res:
        return res.read()


def detect(predictor, image, prompts, labels, encodings, threshold, iou_threshold,
           contain_threshold):
    w, h = image.size
    out = predictor.predict(image=image, text=prompts, text_encodings=encodings,
                            threshold=threshold, pad_square=False)
    candidates = []
    for idx, score, box in zip(out.labels.tolist(), out.scores.tolist(), out.boxes.tolist()):
        x0, y0, x1, y1 = box
        candidates.append({
            'label': labels[idx],
            'prompt': prompts[idx],
            'score': round(float(score), 3),
            'box': [max(0.0, min(1.0, x0 / w)), max(0.0, min(1.0, y0 / h)),
                    max(0.0, min(1.0, x1 / w)), max(0.0, min(1.0, y1 / h))],
        })
    return [c for c in nms(candidates, iou_threshold, contain_threshold) if c['label']]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--car-url',
                        default=os.environ.get('AI_CAR_URL', 'http://100.70.35.31:8080'))
    parser.add_argument('--prompts', default=os.path.join(here, 'prompts.json'))
    parser.add_argument('--threshold', type=float, default=0.3)
    parser.add_argument('--nms-iou', type=float, default=0.5)
    parser.add_argument('--nms-contain', type=float, default=0.8,
                        help='小さい枠がこの割合以上ほかの枠の中にあれば捨てる')
    parser.add_argument('--rate', type=float, default=5.0, help='1 秒あたりの推論回数の上限')
    parser.add_argument('--model', default='google/owlvit-base-patch32')
    parser.add_argument('--engine', default='/opt/nanoowl/data/owl_image_encoder_patch32.engine')
    parser.add_argument('--once', action='store_true', help='1 枚だけ推論して結果を表示する（送らない）')
    args = parser.parse_args()

    token = os.environ.get('AI_CAR_API_TOKEN', '')
    prompts, labels = load_prompts(args.prompts)
    predictor = OwlPredictor(args.model, image_encoder_engine=args.engine)
    encodings = predictor.encode_text(prompts)
    snapshot_url = args.car_url.rstrip('/') + '/api/camera/snapshot'
    post_url = args.car_url.rstrip('/') + '/api/jetson/detections'
    model_name = 'NanoOWL ' + args.model.split('/')[-1]
    interval = 1.0 / max(args.rate, 0.1)
    print(f'NanoOWL 開始: {snapshot_url} → {post_url}、言葉 {prompts}', flush=True)

    last_error = ''
    while True:
        start = time.monotonic()
        try:
            image = PIL.Image.open(io.BytesIO(fetch(snapshot_url, 3.0))).convert('RGB')
            t0 = time.perf_counter()
            detections = detect(predictor, image, prompts, labels, encodings,
                                args.threshold, args.nms_iou, args.nms_contain)
            infer_ms = round((time.perf_counter() - t0) * 1000.0, 1)
            if args.once:
                print(json.dumps({'inference_ms': infer_ms, 'detections': detections},
                                 ensure_ascii=False, indent=1))
                sys.stdout.flush()
                os._exit(0)
            post(post_url, {'model': model_name, 'inference_ms': infer_ms,
                            'detections': detections}, token, 3.0)
            if last_error:
                print('AI-CAR との通信が戻りました', flush=True)
                last_error = ''
        except (urllib.error.URLError, http.client.HTTPException, OSError, ValueError) as exc:
            if str(exc) != last_error:
                print(f'AI-CAR と通信できません: {exc}', flush=True)
                last_error = str(exc)
            time.sleep(1.0)
        time.sleep(max(0.0, interval - (time.monotonic() - start)))


if __name__ == '__main__':
    main()
