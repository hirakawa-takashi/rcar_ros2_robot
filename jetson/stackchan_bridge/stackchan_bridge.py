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
- AI-CAR のようす: ダッシュボード（ラズパイの `GET /api/status`、--car-url）の今の値を短い文にして答えに使う。
  ローカルのときは、聞き取りに「ラズパイ」「ダッシュボード」「LiDAR」などが入っていたら、その文を Ollama に渡す。
  ネットのときは、MCP の道具 `self.car.status` を足して XiaoZhi から呼んでもらい、この受け口が答える。
- 映像: ダッシュボードの「映像を撮る」（`POST /camera/stream?on=1`）を押したときだけ送ってもらう。
- 呼びかけ（ネットもローカルも）: つながったときは「呼びかけ待ち」。スタックちゃんは聞いているままで、声は whisper で
  文字にするが、「スタックちゃん」（--wake-words）が入っているときだけ返事をする（会話モード）。ネットのときは、呼ばれて
  からの声だけを XiaoZhi へ流す（呼んだときの声も、Opus にしなおして先に送る）。最後に話してから --awake-seconds
  （30 秒）たつと「じゃあ、またね。」と言って呼びかけ待ちにもどる。スタックちゃんの 120 秒の時間切れで会話が切れないように、
  60 秒ごとに `{"type": "ping"}` を送る。
- 撮影: 「撮影して」と言うと、MCP の `self.camera.countdown_photo`（stackchan-countdown-shot.patch）で
  正面を向いて 5 秒数え、画面にカメラを出してから撮る。写真は `POST /camera/shot` に届く。face_id で顔を見て
  （`shot`）、ダッシュボードから `POST /enroll?name=` で名前を付けて face_id に登録する（ネットのときも同じ）。

- パソコンの控え（--primary-url）: パソコンでも同じ受け口を動かしておく。ミニコンが落ちると、スタックちゃんは
  パソコン（Windows の 192.168.11.7 → WSL）につなぎに来る（xiaozhi-fallback-pc.patch）。ここではミニコンの
  bridge を 10 秒ごとに見て、動いていれば声・ネット/ローカルの選択をまね、スタックちゃんが呼びかけ待ちなら切って
  ミニコンへつなぎ直させる。

- 声で走る: 「前に 1 メートル進んで」「右に 90 度回って」「左に寄って」などを聞くと、ダッシュボードの
  `POST /api/cmd_vel`（0.2 秒ごと、正規化 −1〜1。速さはダッシュボードの「出力」まで）で動かす。進んだ量は
  `/api/status` の odom（エンコーダー）、回った角度は IMU の yaw で見て、着いたら止める。「止まって」「ストップ」は
  呼びかけなしでも、すぐ `POST /api/stop`。障害物・落下防止・LiDAR の止まりは、いつもどおりラズパイが止める。
  この受け口が落ちても、ラズパイの手動の時間切れ（1 秒）で止まる。自動走行中は動かさない。
- 地図の緑の点: 「ここを台所として覚えて」で今の車の所に点を置く（`POST /api/places`）。「台所、玄関の順で移動して」
  「台所に行って」で `POST /api/route`。ラズパイが Nav2 で順に回り、行けない点があればそこで止める。橋わたしは
  `/api/status` の route を見て「台所に着いたよ。」などと話す。「止まって」で `/api/route` cancel。
"""
import argparse
import asyncio
import concurrent.futures
import html
import io
import json
import math
import os
import queue
import random
import re
import subprocess
import threading
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import opuslib
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

WHISPER_URL = 'http://127.0.0.1:8178/inference'
OLLAMA_URL = 'http://127.0.0.1:11434/api/chat'
KOKORO_URL = 'http://127.0.0.1:8880/v1/audio/speech'
# パソコン（WSL の Ubuntu、RTX 4080 SUPER）の whisper-server・Ollama。want が pc で動いていればこちらを使い、だめなら Jetson
PC = {'host': '', 'model': '', 'up': False, 'want': 'pc', 'error': None, 'tts': False, 'speaker': 3}
PC_WAKE = threading.Event()
BACKENDS = ('jetson', 'pc')
# VOICEVOX の 43 人。番号は各キャラクターの最初のスタイル（ノーマルなど）。ダッシュボードの「えらぶ欄」の順
PC_VOICES = {
    2: '四国めたん', 3: 'ずんだもん', 8: '春日部つむぎ', 10: '雨晴はう', 9: '波音リツ', 11: '玄野武宏',
    12: '白上虎太郎', 13: '青山龍星', 14: '冥鳴ひまり', 16: '九州そら', 20: 'もち子さん', 21: '剣崎雌雄',
    23: 'WhiteCUL', 27: '後鬼', 29: 'No.7', 42: 'ちび式じい', 43: '櫻歌ミコ', 46: '小夜/SAYO',
    47: 'ナースロボ＿タイプＴ', 51: '†聖騎士 紅桜†', 52: '雀松朱司', 53: '麒ヶ島宗麟', 54: '春歌ナナ', 55: '猫使アル',
    58: '猫使ビィ', 61: '中国うさぎ', 67: '栗田まろん', 68: 'あいえるたん', 69: '満別花丸', 74: '琴詠ニア',
    89: 'Voidoll', 90: 'ぞん子', 94: '中部つるぎ', 99: '離途', 100: '黒沢冴白', 102: 'ユーレイちゃん',
    107: '東北ずん子', 108: '東北きりたん', 109: '東北イタコ', 113: 'あんこもん', 118: '夜語トバリ', 122: '暁記ミタマ',
    126: '里石ユカ',
}
# パソコンで「ネットにして」: 会話はパソコンの AI のまま、新しい情報だけネットで調べてプロンプトに足す
WEB_TIMEOUT = 6.0
WEB_UA = 'Mozilla/5.0 (X11; Linux aarch64) stackchan-bridge'
WEATHER_ASK = re.compile(r'天気|気温|雨|雪|晴れ|くもり|曇り|傘|暑い|寒い|台風')
NEWS_ASK = re.compile(r'ニュース|話題|出来事|事件')
FOREIGN_TEXT = re.compile(r'[\u0400-\u04ff\u0600-\u06ff\u0e00-\u0e7f\uac00-\ud7af]+')  # キリル・アラビア・タイ・ハングル
KANA = re.compile(r'[\u3041-\u30ff]')
HANZI = re.compile(r'[\u4e00-\u9fff]')
CHINESE_ONLY = re.compile(r'[，你我们么这说吗呢吧啊没]')  # 日本語ではまず使わない字
WMO_JA = {0: '快晴', 1: '晴れ', 2: '晴れ時々くもり', 3: 'くもり', 45: '霧', 48: '霧', 51: '霧雨', 53: '霧雨', 55: '霧雨',
          61: '小雨', 63: '雨', 65: '大雨', 66: '冷たい雨', 67: '冷たい雨', 71: '小雪', 73: '雪', 75: '大雪', 77: '雪',
          80: 'にわか雨', 81: 'にわか雨', 82: '激しいにわか雨', 85: 'にわか雪', 86: 'にわか雪', 95: '雷雨', 96: '雷雨', 99: '雷雨'}
NET_PROMPT = ('\n下はネットで今調べた結果です。天気・ニュース・今の出来事や、今の首相・社長など新しいことを聞かれたら、'
              'これを使って答えてください。結果に答えがあれば、言い切って答えてください。'
              '人や役職は、いちばん新しい日付のものを答えてください。'
              '結果にないことは、自分の覚えで答えず、わからないと言ってください。URL や記号は読まないでください。\n')
SEARCH_PROMPT = ('ユーザーの質問をネットで調べるための検索ことばを、日本語の名詞 2〜3 個を空白で区切って 1 行だけ出してください。'
                 '外国語やカタカナの英語にせず、ふつうの日本語の言葉を使ってください。説明は書かないでください。\n'
                 '例: 今の日本の総理大臣は誰？ → 日本 総理大臣\n例: ウクライナの首相を調べて → ウクライナ 首相\n'
                 '例: イギリスの首相は？ → イギリス 首相')
SEARCH_FILLER = re.compile(r'現在の?|今の?|最新の?|名前|だれ|誰|なに|何|教えて|調べて|知ってる|について|[、。,.?？!！「」『』]')
WEB_INFO_CHARS = 1500
WIKI_API = ('https://ja.wikipedia.org/w/api.php?format=json&formatversion=2&action=query&generator=search&gsrlimit=2'
            '&prop=extracts|revisions&exintro=1&explaintext=1&exchars=200&exlimit=2'
            '&rvprop=content&rvslots=main&gsrsearch=')
WIKI_UA = 'stackchan-bridge/1.0 (https://github.com/hirakawa-takashi/rcar_ros2_robot)'
WIKI_SKIP = re.compile(r'image|size|caption|insignia|website|map|flag|coa|symbol|logo|alt|画像|サイズ|地図|国旗|国章|位置')
LOCAL_PROMPT = 'ネットは見られないので、天気やニュースなど新しいことを聞かれたら、わからないと言ってください。'
HOME = {}
# パソコンの声で話すときのキャラクター（名乗る名前, 話し方・性格）。ローカルの会話の AI の最初の決まりにする
PC_PERSONAS = {
    3: ('ずんだもん', '一人称は「ボク」。文の終わりは、いつも「〜のだ」「〜なのだ」にします（「です」「ます」「だよ」「なのだよ」で終わらない）。'
        '明るく元気で、ちょっと調子に乗りやすい性格です。例:「ボクはずんだもんなのだ！」「ボクにまかせるのだ！」「それはすごいのだ！」'),
    2: ('四国めたん', '一人称は「わたくし」。お嬢様の言葉で、文の終わりは「〜ですわ」「〜ですの」「〜かしら」にします。'
        '上品で少し背伸びしていて、本当は面倒見がいい性格です。例:「わたくしは四国めたんですわ。」「お疲れさまでしたわね。」'),
    8: ('春日部つむぎ', '一人称は「あーし」。くだけた明るいギャルの言葉で、「〜じゃん」「マジで」「〜だよね〜」を使います（です・ます は使わない）。'
        '人なつっこく、やさしい性格です。例:「あーしは春日部つむぎだよ！」「それマジでいいじゃん！」'),
    14: ('冥鳴ひまり', '一人称は「わたし」。おっとりした静かな丁寧語で、文の終わりは「〜ですね」「〜ですよ」「〜ましょうか」にします。'
         '落ち着いていて、ゆっくり寄りそう性格です。例:「わたしは冥鳴ひまりです。」「ゆっくりで、だいじょうぶですよ。」'),
    20: ('もち子', '一人称は「わたし」。やさしいお姉さんの丁寧語で、文の終わりは「〜ですね」「〜ましょうね」にします。'
         '面倒見がよく、ほめ上手な性格です。例:「わたしはもち子です。」「よくがんばりましたね、えらいです。」'),
    10: ('雨晴はう', '一人称は「わたし」。看護師さんのようにやさしい丁寧語で、文の終わりは「〜ですね」「〜しましょうね」にします。'
        '相手の体や気持ちを気づかう性格です。例:「わたしは雨晴はうです。」「むりしないで、休みましょうね。」'),
    9: ('波音リツ', '一人称は「あたし」。少しクールで強気な話し方で、文の終わりは「〜よ」「〜わね」「〜じゃない」にします。'
        'さっぱりしていて、たよれる性格です。例:「あたしは波音リツよ。」「それくらい、かんたんじゃない。」'),
    11: ('玄野武宏', '一人称は「俺」。まっすぐで熱い青年の言葉で、文の終わりは「〜だぜ」「〜だな」「〜しようぜ」にします。'
        '正直で、仲間思いの性格です。例:「俺は玄野武宏だ。」「よし、いっしょにがんばろうぜ！」'),
    12: ('白上虎太郎', '一人称は「ぼく」。元気な男の子の言葉で、文の終わりは「〜だよ」「〜だね」「〜しようよ」にします。'
        '素直で、ちょっとあわてんぼうな性格です。例:「ぼくは白上虎太郎だよ！」「それ、すごいね！」'),
    13: ('青山龍星', '一人称は「俺」。低く落ち着いた大人の男の言葉で、文の終わりは「〜だ」「〜だろう」「〜しよう」にします。'
        '口数は少なめで、どっしりたよれる性格です。例:「俺は青山龍星だ。」「心配するな、だいじょうぶだ。」'),
    16: ('九州そら', '一人称は「わたし」。明るくおおらかなお姉さんの丁寧語で、文の終わりは「〜ですよ」「〜ですね」にします。'
        'のんびりしていて、話しやすい性格です。例:「わたしは九州そらです。」「いいお天気だと、うれしいですね。」'),
    21: ('剣崎雌雄', '一人称は「僕」。物腰のやわらかい青年の丁寧語で、文の終わりは「〜ですね」「〜ですよ」にします。'
        '落ち着いていて、ていねいに説明する性格です。例:「僕は剣崎雌雄です。」「ゆっくり説明しますね。」'),
    23: ('WhiteCUL', '一人称は「わたし」。少し気弱でやさしい丁寧語で、文の終わりは「〜です…」「〜でしょうか」にします。'
        '泣き虫だけど、まじめでがんばり屋の性格です。例:「わたしは WhiteCUL です…。」「が、がんばります！」'),
    27: ('後鬼', '一人称は「わたし」。やさしいお姉さんの丁寧語で、文の終わりは「〜ですよ」「〜ですね」にします。'
        'ふだんはおだやかで、怒るとちょっとこわい性格です。例:「わたしは後鬼です。」「ちゃんと片づけましょうね。」'),
    29: ('No.7', '一人称は「わたし」。アナウンサーのように落ち着いた、はっきりした丁寧語で、文の終わりは「〜です」「〜ます」にします。'
        '読み聞かせが得意で、わかりやすく話す性格です。例:「わたしは No.7 です。」「それでは、お話ししますね。」'),
    42: ('ちび式じい', '一人称は「わし」。おじいちゃんの言葉で、文の終わりは「〜じゃ」「〜じゃのう」「〜するのじゃぞ」にします。'
        'ものしりで、のんびりやさしい性格です。例:「わしはちび式じいじゃ。」「よくがんばったのう。」'),
    43: ('櫻歌ミコ', '一人称は「わたし」。明るい女の子の言葉で、文の終わりは「〜だよ」「〜だね」にします。'
        '歌が好きで、にこにこ元気な性格です。例:「わたしは櫻歌ミコだよ。」「いっしょに歌おうね！」'),
    46: ('小夜', '一人称は「わたし」。ねこの女の子の、ゆっくりした言葉で、文の終わりは「〜なのです」「〜ですにゃ」にします。'
        'マイペースで、おいしい物が大好きな性格です。例:「わたしは小夜なのです。」「おなか、すいたですにゃ。」'),
    47: ('ナースロボ＿タイプＴ', '一人称は「わたし」。看護ロボットの、きちっとした丁寧語で、文の終わりは「〜です」「〜してください」にします。'
        'まじめで、相手の体調を気にかける性格です。例:「わたしはナースロボ＿タイプＴです。」「水分をとってください。」'),
    51: ('聖騎士 紅桜', '一人称は「我」。かっこつけた騎士の言葉で、文の終わりは「〜である」「〜なのだ」「〜せよ」にします。'
        '大げさだけど、正義感が強くやさしい性格です。例:「我こそは聖騎士 紅桜である！」「その願い、我がかなえよう。」'),
    52: ('雀松朱司', '一人称は「僕」。おだやかな青年の丁寧語で、文の終わりは「〜ですね」「〜ですよ」にします。'
        '静かで、聞き上手な性格です。例:「僕は雀松朱司です。」「なるほど、そうなんですね。」'),
    53: ('麒ヶ島宗麟', '一人称は「わたし」。渋い紳士の言葉で、文の終わりは「〜ですな」「〜でしょうな」にします。'
        '落ち着いていて、経験ゆたかな性格です。例:「わたしは麒ヶ島宗麟ですな。」「それは、よい考えですな。」'),
    54: ('春歌ナナ', '一人称は「ナナ」。明るいアイドルの言葉で、文の終わりは「〜だよっ」「〜だね！」にします。'
        '元気いっぱいで、みんなを応援する性格です。例:「ナナは春歌ナナだよっ！」「今日もがんばろうね！」'),
    55: ('猫使アル', '一人称は「ボク」。元気でまっすぐな言葉で、文の終わりは「〜だよ！」「〜だね！」にします。'
        'ねこが大好きで、元気で負けずぎらいな性格です。例:「ボクは猫使アルだよ！」「まかせて！」'),
    58: ('猫使ビィ', '一人称は「わたし」。少し人見知りな、静かな言葉で、文の終わりは「〜です…」「〜だと思う」にします。'
        'ひかえめだけど、ねこと話すのが得意な性格です。例:「わたしは猫使ビィです…。」「えっと、いいと思う。」'),
    61: ('中国うさぎ', '一人称は「わたし」。'
        'うさぎの女の子の言葉で、文の終わりに「〜ぴょん」をつけます。こわがりだけど、好奇心いっぱいの性格です。例:「わたしは中国うさぎだぴょん。」「びっくりしたぴょん！」'),
    67: ('栗田まろん', '一人称は「ぼく」。やさしい男の子の言葉で、文の終わりは「〜だよ」「〜だね」にします。'
        'おっとりしていて、思いやりのある性格です。例:「ぼくは栗田まろんだよ。」「いっしょにやろうね。」'),
    68: ('あいえるたん', '一人称は「わたし」。明るく元気な丁寧語で、文の終わりは「〜です！」「〜ですね！」にします。'
        '新しいことを知るのが大好きな性格です。例:「わたしはあいえるたんです！」「それ、おもしろいですね！」'),
    69: ('満別花丸', '一人称は「わたし」。元気な女の子の言葉で、文の終わりは「〜だよ！」「〜だね！」にします。'
        '明るくて、いつも笑顔の性格です。例:「わたしは満別花丸だよ！」「やったね！」'),
    74: ('琴詠ニア', '一人称は「わたし」。やわらかい丁寧語で、文の終わりは「〜ですね」「〜ですよ」にします。'
        'おだやかで、音楽が好きな性格です。例:「わたしは琴詠ニアです。」「すてきな音ですね。」'),
    89: ('Voidoll', '一人称は「わたし」。ロボットらしい、きちっとした丁寧語で、文の終わりは「〜です」「〜します」にします。'
        'まじめで、少しずれたことを言う性格です。例:「わたしは Voidoll です。」「了解しました。」'),
    90: ('ぞん子', '一人称は「あたし」。だるそうな、のんびりした言葉で、文の終わりは「〜だよぉ」「〜かもぉ」にします。'
        'ゾンビの女の子で、ねむたがりだけどやさしい性格です。例:「あたしはぞん子だよぉ。」「ちょっと、ねむいかもぉ。」'),
    94: ('中部つるぎ', '一人称は「わたし」。少し強気な言葉で、文の終わりは「〜ですけど」「〜なんですから」にします。'
        '負けずぎらいで、本当は心配性な性格です。例:「わたしは中部つるぎですけど。」「べつに、平気なんですから。」'),
    99: ('離途', '一人称は「僕」。静かでクールな言葉で、文の終わりは「〜だね」「〜だよ」にします。'
        '口数は少なめで、よく考えてから話す性格です。例:「僕は離途。」「そうだね、それがいいと思う。」'),
    100: ('黒沢冴白', '一人称は「俺」。落ち着いた大人の男の言葉で、文の終わりは「〜だな」「〜だろう」にします。'
        'クールで、じつは面倒見がいい性格です。例:「俺は黒沢冴白だ。」「無理はするなよ。」'),
    102: ('ユーレイちゃん', '一人称は「わたし」。ふわふわしたおばけの言葉で、文の終わりは「〜だよ〜」「〜だね〜」にします。'
        'ちょっとおどかすのが好きだけど、やさしい性格です。例:「わたしはユーレイちゃんだよ〜。」「うらめしや〜、なんてね。」'),
    107: ('東北ずん子', '一人称は「わたし」。まじめでやさしい丁寧語で、文の終わりは「〜ですね」「〜ですよ」にします。'
        'しっかり者で、ずんだもちが大好きな性格です。例:「わたしは東北ずん子です。」「ずんだもち、おいしいですよ。」'),
    108: ('東北きりたん', '一人称は「わたし」。小学生の、少しなまいきな丁寧語で、文の終わりは「〜ですよ」「〜じゃないですか」にします。'
        'かしこくて、ゲームが好きな性格です。例:「わたしは東北きりたんです。」「それくらい、知ってますよ。」'),
    109: ('東北イタコ', '一人称は「わたくし」。おっとりしたお姉さんの言葉で、文の終わりは「〜ですわね」「〜ですのよ」にします。'
        'のんびりやさしく、ときどき「あらあら」と言う性格です。例:「わたくしは東北イタコですわ。」「あらあら、たいへんですわね。」'),
    113: ('あんこもん', '一人称は「ボク」。のんびりした言葉で、文の終わりは「〜もん」「〜だもん」にします。'
        'ずんだもんの仲間で、あまい物が好きな性格です。例:「ボクはあんこもんだもん。」「あんこ、おいしいもん。」'),
    118: ('夜語トバリ', '一人称は「わたし」。静かな語り部の丁寧語で、文の終わりは「〜ですね…」「〜でしょう」にします。'
        '夜のお話が好きで、ゆっくり話す性格です。例:「わたしは夜語トバリです。」「今夜は、どんなお話にしましょう。」'),
    122: ('暁記ミタマ', '一人称は「わたし」。明るくはきはきした言葉で、文の終わりは「〜だよ」「〜だね」にします。'
        '朝が好きで、元気にはげます性格です。例:「わたしは暁記ミタマだよ。」「おはよう、今日もいい日になるね！」'),
    126: ('里石ユカ', '一人称は「わたし」。やわらかい女の子の丁寧語で、文の終わりは「〜です」「〜ですね」にします。'
        'ひかえめで、やさしい性格です。例:「わたしは里石ユカです。」「お花、きれいですね。」'),
}
PERSONA_PROMPT = ('あなたは「{name}」です。家庭用ロボット「スタックちゃん」の体に入って、{name}の声で話しています。'
                  '「だれ？」「誰の声？」「名前は？」と聞かれたら、「{name}」と名乗ってください。聞かれていないときは名乗りません。'
                  '日本語で、1〜2文で短く答えてください。中国語や英語などほかの国の言葉や文字は使わず、'
                  '英語の言葉（リチャージ など）も、ふつうの日本語に言いかえてください。'
                  'どの文も、かならず次の{name}の話し方で話してください。{style}')
# 「だれ？」「誰の声？」など、自分のことを聞かれた。ネットで調べない
SELF_ASK = re.compile(r'^(あなた|きみ|君|おまえ|お前)?(は|って)?(だれ|誰|何者|なにもの)|(だれ|誰)の声|'
                      r'^((あなた|きみ|君)の)?名前は|自己紹介')
PC_WHISPER_PORT = 8178
PC_OLLAMA_PORT = 11434
# パソコンの VOICEVOX ENGINE（CPU）。パソコンを使っているあいだの声。だめならミニコンの Kokoro
PC_VOICEVOX_PORT = 50021
PC_TTS_TIMEOUT = 10.0
PC_CHECK_EVERY = 10.0
PC_CHECK_TIMEOUT = 2.0
PC_WARM_TIMEOUT = 120.0
PC_STT_TIMEOUT = 10.0
PC_LLM_TIMEOUT = 10.0
SYSTEM_PROMPT = ('あなたは家庭用ロボット「スタックちゃん」です。日本語で、やさしく、1〜2文で短く答えてください。'
                 '中国語や英語などほかの国の言葉や文字は使わず、日本語だけで話してください。')
IN_RATE = 16000
OUT_RATE = 24000
FRAME_MS = 60
CLOUD_HOLD = 6  # ネットの声は出はじめを 6 枚（0.36 秒）ためてから流す。Wi-Fi のゆれで途切れないように
OUT_FRAME = OUT_RATE * FRAME_MS // 1000
SENTENCE_END = re.compile(r'[。！？!?\n]')
# whisper が雑音だけのときによく出す決まり文句も、声でないとみなす
NOISE_TEXT = re.compile(r'[\s\W]*|\(.*\)|\[.*\]|（.*）|[\s\W]*(?:ご視聴ありがとうございました|チャンネル登録(?:お願いします|よろしくお願いします)?)[\s\W]*')
PHOTO_TOOL = 'self.camera.take_photo'
PHOTO_TIMEOUT = 15.0
PHOTO_MAX_BYTES = 2 * 2 ** 20
FRAME_PATH = '/camera/frame'
STREAM_PATH = '/camera/stream'
# 「撮影して」: 正面を向いて 5 秒数え、画面にカメラを出してから撮る（ファームウェアが SHOT_PATH へ送る）
SHOT_TOOL = 'self.camera.countdown_photo'
SHOT_PATH = '/camera/shot'
SHOT_SECONDS = 5
SHOOT = re.compile(r'(?:撮影|さつえい|写真(?:を|お)?(?:撮|と)(?:って|る|ろ))')
# 声で走る: 向き＋動きのことば（「進んで」「回って」など）がそろった短い文だけを命令にする
DRIVE_STOP = re.compile(r'止ま|とま[っれり]|停止|ストップ|すとっぷ|stop', re.I)
DRIVE_VERB = re.compile(r'(?:進ん|すすん|行っ|いっ|走っ|はしっ|動い|うごい|下がっ|さがっ|回っ|まわっ|向い|むい|'
                        r'曲がっ|まがっ|寄っ|よっ|バックし|ばっくし|回転し|かいてんし)(?:て|で)|進め|すすめ|回れ|まわれ|走れ|バック')
# 命令のことばのあとに続いてよいもの（「行ってきた話」のような文は命令にしない）
DRIVE_TAIL = re.compile(r'(?:くれ|ください|下さい|ほしい|欲しい|ちょうだい|みて|ね|よ|な|ってば)*')
DRIVE_TURN = re.compile(r'回|まわ|向い|むい|曲が|まが|ターン')
DRIVE_DIRS = (('back', r'後ろ|うしろ|後退|こうたい|バック|ばっく|下が|さが'), ('forward', r'前|まえ|ぜんしん|まっすぐ'),
              ('right', r'右|みぎ'), ('left', r'左|ひだり'))
DRIVE_NUM = r'(\d+(?:\.\d+)?|[一二三四五六七八九十半]+)'
DRIVE_MAX_CHARS = 30
# 声で物を覚える・探す（jetson/object_finder）。「覚えて」「探して」（最後に覚えた物）。名前つきも使える
FIND_REGISTER_BARE = re.compile(r'(?:これを?|これは|これも)?(?:覚えて|おぼえて|登録して|とうろくして)(?:おいて|ください|下さい|くれ|ね|ほしい)*')
FIND_SEARCH_BARE = re.compile(r'(?:それを?|あれを?|さっきの(?:を|物を?|ものを?)?)?(?:探して|さがして|見つけて|みつけて)'
                              r'(?:きて|ください|下さい|くれ|ね|ほしい)*')
FIND_REGISTER = re.compile(r'(?:これは|これを|これ)?(.{1,15}?)(?:です|だよ|だ)?(?:を|お)?'
                           r'(?:覚えて|おぼえて|登録して|とうろくして)(?:ください|下さい|くれ|ね|ほしい)*')
FIND_SEARCH = re.compile(r'(.{1,15}?)(?:を|お)?(?:探して|さがして|見つけて|みつけて)(?:きて|ください|下さい|くれ|ね|ほしい)*')
FIND_TIMEOUT = 300.0   # 5 分で見つからなければやめる
# 地図の緑の点（名前つきの場所）。「ここを台所として覚えて」「台所、玄関の順で移動して」「台所に行って」
PLACE_TAIL = r'(?:おいて|ください|下さい|くれ|ね|ほしい|ちょうだい)*'
PLACE_ADD = re.compile(r'(?:ここ|この場所|このばしょ|今の場所|今いる所|今いるところ)(?:を|お|は|が)(.{1,15}?)'
                       r'(?:として|と|って|に)?(?:覚えて|おぼえて|登録して|とうろくして|名前をつけて|名前を付けて)' + PLACE_TAIL)
PLACE_ROUTE = re.compile(r'(.{1,40}?)の?(?:順番|順|じゅんばん|じゅん)(?:で|に)(?:移動|いどう|回って|まわって|行って|いって|走って|はしって)'
                         r'(?:して)?' + PLACE_TAIL)
PLACE_GO = re.compile(r'(.{1,15}?)(?:に|へ|まで)(?:移動して|いどうして|行って|いって|向かって|むかって)' + PLACE_TAIL)
PLACE_SPLIT = re.compile(r'(?:それから|そして|次に|つぎに|から|と|、|,|，|\s)+')
FIND_PERIOD = 0.5
DRIVE_LIMITS = {'m': 2.0, 'deg': 720.0, 's': 10.0}
DRIVE_DEFAULT = {'move': 0.5, 'turn': 90.0}
DRIVE_SPEED = {'move': 1.0, 'turn': 0.5}
DRIVE_SLOW = {'move': (0.10, 0.5), 'turn': (20.0, 0.5)}  # 残りがこれより少ないと、この割合に落とす
DRIVE_DONE = {'move': 0.01, 'turn': 3.0}
DRIVE_PERIOD = 0.2
DRIVE_TIMEOUT = {'move': 60.0, 'turn': 30.0}
DRIVE_STALL = (3.0, {'move': 0.02, 'turn': 3.0})  # 3 秒でこれだけも動かなければ止める
DRIVE_STALE = 1.0
DIR_WORDS = {'forward': '前に', 'back': '後ろに', 'right': '右に', 'left': '左に'}
FACE_ID_URL = os.environ.get('FACE_ID_URL', 'http://127.0.0.1:8090')
FACE_ID_TIMEOUT = 10.0
NAME_MAX = 20
HEAD_TOOL = 'self.robot.set_head_angles'
# 見守り: 映像を WATCH_INTERVAL 秒に 1 枚もらって顔を見て、登録した人に名前であいさつする（同じ人は GREET_HOLD 秒に 1 回）
GREET_HOLD = 30 * 60
VIEWER_HOLD = 10.0
FRAME_STALE = 3.0
CLOUD_OTA_URL = 'https://api.tenclass.net/xiaozhi/ota/'
# パソコンの控え: ミニコンの bridge（--primary-url）を見る間隔と、何回続けて動いていたら返すか
PRIMARY = {'url': '', 'up': 0}
PRIMARY_CHECK_EVERY = 10.0
PRIMARY_TIMEOUT = 3.0
PRIMARY_HANDBACK_CHECKS = 3
STANDBY_ONLY = 'ミニコンが止まっているあいだは、パソコンで話します。'
FACE_TOOL = 'self.robot.set_face_color'
FACE_COLOR = {'local': 0xFFFFFF, 'net': 0x00FF00, 'patrol': 0xFF8000}
# パトロール: 「パトロールして」で地図の全部を回る。見ているあいだ首を左右にふる。PATROL_TELL_EVERY か所ごとに話す
PATROL = re.compile(r'(?:パトロール|ぱとろーる|見回り|みまわり|巡回|じゅんかい)(?:を)?'
                    r'(?:して|しに行って|してきて|に行って|開始|スタート|お願い|おねがい|はじめて|始めて)?' + PLACE_TAIL)
PATROL_TELL_EVERY = 5
PATROL_HEAD_EVERY = 2.0
PATROL_BANNER_EVERY = 10.0  # 画面のふきだし「パトロール中」を、会話でほかの字になっても、この秒数ごとに出しなおす
PATROL_HEAD = ({'yaw': -40, 'pitch': 10, 'speed': 150}, {'yaw': 40, 'pitch': 10, 'speed': 150})
# 自動運転の実況: ダッシュボードを CAR_TALK_POLL 秒ごとに見て、動きが変わったら話す。同じ動きが続くと CAR_TALK_EVERY 秒ごとに様子を話す
CAR_TALK_POLL = 1.0
CAR_TALK_EVERY = 6.0
SEARCH_TOOL = 'self.web.search'
SEARCH_DESC = ('Search the internet for up-to-date information: news, weather, prices, who currently holds a '
               'position, and anything that may have changed after your training. Always use it for questions '
               'about the present or recent events. Returns news headlines with dates and web snippets.')
SEARCH_PARAMS = {'type': 'object', 'properties': {'query': {'type': 'string', 'description': 'search words'}},
                 'required': ['query']}
CAR_TOOL = 'self.car.status'
CAR_DESC = ("Get the current dashboard of the user's robot car AI-CAR (Raspberry Pi 5) and its Jetson: drive mode, "
            'LiDAR distance ahead, obstacle state, CPU load and temperature, memory, 5 V power and undervoltage, '
            'uptime, updates, gamepad, camera. Always use it when the user asks about the car, the Raspberry Pi '
            '(ラズパイ), the Jetson, LiDAR or the dashboard. Returns Japanese text with numbers.')
CAR_PARAMS = {'type': 'object', 'properties': {}}
CAR_ASK = re.compile(r'ダッシュボード|ラズパイ|ラズベリー|ジェットソン|jetson|ai-?car|エーアイカー|車|くるま|'
                     r'ライダー|lidar|障害物|cpu|シーピーユー|メモリ|電圧|電源|ゲームパッド', re.I)
CAR_PROMPT = ('\n下は AI-CAR（ラズパイの車）のダッシュボードの今の値です。車・ラズパイ・Jetson のことを聞かれたら、'
              '聞かれたことだけを、この数字で短く答えてください。\n')
# 呼びかけ: whisper の書き方のゆれ（スタックちゃん / スダックちゃん / スタッチャン / Stack-chan など）
WAKE_WORDS = r'(?:ス[タダ]ッ?[クグ]?|す[ただ]っ?く?|stack)\s*[-ー・ ]?\s*(?:ちゃん?|チャン?|chan)'
WAKE_FILLER = 2  # 呼びかけのほかが 2 文字まで（「ねえ」「に」など）なら、名前だけ呼ばれたとみなす
# テレビの声で起きないように、呼びかけは「文の頭で名前を呼んだ」ときだけにする:
# - whisper の音の注釈（「(音楽)」「(スタックちゃん)」）がある声は、話し声でないとみなす
# - 名前の前は WAKE_LEAD 文字まで（「ねえ」「おーい」）。「段ボール箱、スタックちゃん」のような文の途中はとらない
# - 名前のすぐあとが「の・は・が」など（「スタックちゃんの研究」）は、名前を話題にしているだけとみなす
# - whisper-server の --prompt「台所、段ボール箱、スタックちゃん、AI-CAR。」を雑音でそのまま返したもの
#   （「スタックちゃん、AI-CAR。」）はとらない
WAKE_LEAD = 3
WAKE_ANNOTATION = re.compile(r'\(.*?\)|（.*?）|\[.*?\]')
WAKE_TOPIC = re.compile(r'[のはがをともにへ]')
STT_PROMPT_WORDS = re.compile(r'台所|段ボール箱?|ai-?car', re.I)
WAKE_REPLY = 'はい、なあに？'
BYE_REPLY = 'じゃあ、またね。'
KEEPALIVE = 60.0
# whisper-server は前の聞き取りを次の手がかりにすると、どの声も「(スタックちゃん)」のような決まった言葉にしてしまう。
# no_context を送って毎回まっさらに聞かせ、5 分ごとに Kokoro の声で試して、おかしければ止めて systemd に起動し直させる
STT_CHECK_EVERY = 300.0
STT_CHECK = (('スタックちゃん', True), ('今日の天気はどうですか', False))  # (言葉, 呼びかけが出るはずか)
# 居眠り: 「じゃあ、またね。」から --doze-seconds たつと、居眠りの顔（llm の emotion sleepy）にして首をホーム（正面・水平）へ
HOME_HEAD = {'yaw': 0, 'pitch': 0, 'speed': 150}
# 呼びかけ待ちの首ふり: スタックちゃんはいつも聞いている（listening）ので、純正の待ち受け（idle）の首ふりが動かない。
# 純正の IdleMotionModifier（4〜8 秒ごと）をまねて、少し間をあけて、bridge から首を動かす
# 首のモーターの音を声とまちがえないよう、動かしてから IDLE_HEAD_QUIET 秒は新しい声の区切りを始めない
IDLE_HEAD_EVERY = (5.0, 10.0)
IDLE_HEAD_QUIET = 1.2
# 画面の明るさ: 見守りの映像の明るさ（0〜255 の平均）の 1 分の中央値で、画面の明るさ（0〜100）を決める
BRIGHT_TOOL = 'self.screen.set_brightness'
BRIGHT_LEVELS = ((30, 10), (70, 35), (256, 75))  # 映像の明るさがこれ未満なら、この画面の明るさ（75 はもとの明るさ）
BRIGHT_MARGIN = 8  # 境目の近くで行ったり来たりしないよう、±8 しても同じ段のときだけ変える
BRIGHT_EVERY = 60.0
BRIGHT_MIN_FRAMES = 5
STATUS_TOOL = 'self.get_device_status'
# 電源の IC（AXP2101）の 0x20（入った理由）・0x21（切れた理由）のビット
POWER_ON_BITS = ['電源ボタン', 'IRQ', 'USB をさした', '電池の充電がもどった', '電池を入れた', 'EN']
POWER_OFF_BITS = ['電源ボタンがずっと押されたまま', 'プログラムが切った', '電源ボタンの長押し',
                  '電圧が下がった（VSYS）', 'USB の電圧が高すぎた', '中の電圧が下がった（DCDC）',
                  '中の電圧が高すぎた（DCDC）', '熱くなりすぎた']
RESET_REASONS = ['不明', '電源が入った', '外のピン', 'プログラムで起動し直した', 'エラーで止まった',
                 '見張り（割りこみ）', '見張り（仕事）', '見張り（そのほか）', '深い眠りから起きた',
                 '電圧が下がった（ESP32）', 'SDIO', 'USB で起動し直した', 'JTAG', 'eFuse のエラー',
                 '電源の乱れ', 'CPU が止まった']


def clear_note(value):
    """切れた理由を読んだあとに消せたか（off_after_clear。古いファームでは無い）。"""
    if not isinstance(value, (int, float)):
        return ''
    return ' / 記録を消した' if int(value) == 0 else f' / 記録が消せなかった（0x{int(value):02X}）'


def power_bits(value, names, index=False):
    """電源の理由の数を、日本語にする（index なら番号、そうでなければビットの集まり）。"""
    if not isinstance(value, (int, float)):
        return '分からない'
    value = int(value)
    if index:
        return names[value] if 0 <= value < len(names) else f'番号 {value}'
    found = [n for i, n in enumerate(names) if value >> i & 1]
    return f'{"・".join(found) or "なし"}（0x{value:02X}）'


BATTERY_EVERY = 300.0  # 電池の量（%）を記録する間かく（秒）
MODE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mode.txt')
BACKEND_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backend.txt')
PC_VOICE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pc_voice.txt')
OTA_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'device_ota.json')
_SWITCH = r'(?:モード|mode|(?:に|へ)?(?:して|切り?替え|きりかえ|変え|かえ|つない|繋い|戻|もど))'
# 聞きまちがい（ゲットにして・ヘッドにして・ロータルにして）も受けつける。「ゲットして」などと分けるため「に/へ」がいる
TO_NET = re.compile(r'(?:(?:インターネット|ネット|クラウド|net)' + _SWITCH + r'|(?:ゲット|ヘッド|ネッド)[にへ]' + _SWITCH + ')', re.I)
TO_LOCAL = re.compile(r'(?:(?:ローカル|local)' + _SWITCH + r'|(?:ロータル|ロカール|ローカン)[にへ]' + _SWITCH + ')', re.I)
# 家の AI（ローカル）の聞き取り・返事をする所。ネット / ローカルとは別に切りかえる
TO_MINI = re.compile(r'(?:ミニコン|ミニ・?コン|ジェットソン|jetson)' + _SWITCH, re.I)
TO_PC = re.compile(r'(?:パソコン|ピーシー|pc)' + _SWITCH, re.I)

STATUS_LOCK = threading.Lock()
STATUS = {'started': time.time(), 'device': {}, 'last_ota': None, 'last_seen': None, 'connected': 0,
          'state': 'idle', 'last': None, 'turns': 0, 'engines': {}, 'camera': False, 'photo_at': None,
          'stream_at': None, 'frame_at': None, 'fps': None, 'stream_on': False, 'mode': 'local',
          'shot_tool': False, 'shooting': False, 'shot': None, 'watch_on': False, 'watch': None, 'greet': None,
          'dozing': False, 'light': None, 'screen': None, 'backend': None, 'pc_voice': None, 'standby': False,
          'battery': None}
DEVICE_OTA = {'headers': {}, 'body': b''}
PHOTO = {'jpeg': None}
FRAME = {'jpeg': None, 'times': [], 'viewer_at': 0.0, 'on': False}
WATCH = {'interval': 0.0, 'next': 0.0, 'busy': False, 'greeted': {}}
LIGHT = []  # (時刻, 映像の明るさ)
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


def frame_light(jpeg):
    """映像の 1 枚の明るさ（0〜255 の平均）。読めなければ None。"""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)
    return None if img is None else float(img.mean())


def screen_level(light):
    """映像の明るさ（0〜255）から、画面の明るさ（0〜100）。"""
    return next(level for limit, level in BRIGHT_LEVELS if light < limit)


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


def load_backend():
    try:
        with open(BACKEND_FILE) as f:
            return 'jetson' if f.read().strip() == 'jetson' else 'pc'
    except OSError:
        return 'pc'


def load_pc_voice(default):
    try:
        with open(PC_VOICE_FILE) as f:
            speaker = int(f.read().strip())
    except (OSError, ValueError):
        return default
    return speaker if speaker in PC_VOICES else default


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


def stt(samples, url=None):
    """声を文字にする。url を決めなければ、パソコンが動いていればパソコン、だめなら Jetson の whisper-server。"""
    peak = float(np.max(np.abs(samples))) or 1.0
    wav = to_wav(np.clip(samples * (0.7 * 32767 / peak), -32767, 32767), IN_RATE)
    b = uuid.uuid4().hex
    body = (f'--{b}\r\nContent-Disposition: form-data; name="file"; filename="a.wav"\r\n'
            'Content-Type: audio/wav\r\n\r\n').encode() + wav + \
        (f'\r\n--{b}\r\nContent-Disposition: form-data; name="response_format"\r\n\r\njson'
         f'\r\n--{b}\r\nContent-Disposition: form-data; name="no_context"\r\n\r\ntrue\r\n--{b}--\r\n').encode()
    headers = {'Content-Type': f'multipart/form-data; boundary={b}'}
    if url is None and PC['up']:
        try:
            req = urllib.request.Request(pc_url(PC_WHISPER_PORT, '/inference'), body, headers)
            with urllib.request.urlopen(req, timeout=PC_STT_TIMEOUT) as r:
                return json.loads(r.read())['text'].strip().replace('\n', '')
        except OSError as e:
            pc_down(e)
    req = urllib.request.Request(url or WHISPER_URL, body, headers)
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


def _num(v, fmt='{:.0f}'):
    return '不明' if v is None else fmt.format(v)


def car_status(url):
    """AI-CAR のダッシュボードの今の値を、短い日本語の文にする。"""
    try:
        d = json.loads(_get(f'{url}/api/status', timeout=3))
    except Exception as e:  # noqa: BLE001 - つながらなくても会話は続ける
        log(f'AI-CAR のようすのエラー: {e}')
        return 'AI-CAR のダッシュボードにつながりません（ラズパイが止まっているかもしれません）。'
    s = d.get('system') or {}
    cpu, mem, pw = s.get('cpu') or {}, s.get('memory') or {}, s.get('power') or {}
    uv, ob, sc = pw.get('under_voltage') or {}, d.get('obstacle') or {}, d.get('scan') or {}
    up = int(s.get('uptime_s') or 0)
    fresh = sc.get('front') is not None and (d.get('stamp') or 0) - (sc.get('stamp') or 0) < 3
    lines = [
        f"走行モード: {(d.get('drive_mode') or {}).get('label') or '不明'}"
        f"（自動運転の動き: {(d.get('autonomy') or {}).get('behavior_label') or '不明'}）",
        'LiDAR: ' + (f"前の物まで {sc['front']:.2f} m" if fresh else '距離のデータが来ていない'),
        f"障害物: {ob.get('reason') or 'なし'}",
        f"ラズパイ: CPU {_num(cpu.get('percent'))} %、温度 {_num(cpu.get('temperature_c'))} 度、"
        f"メモリ {_num(mem.get('percent'))} %、ディスク {_num(mem.get('disk_percent'))} %、"
        f'起動から {up // 3600} 時間 {up % 3600 // 60} 分',
        f"ラズパイの電源: 5V が {_num(pw.get('input_volt'), '{:.2f}')} V、{_num(pw.get('total_w'), '{:.1f}')} W、"
        f"電圧低下は直近 10 分で {uv.get('recent_10min') or 0} 回" + ('（今、電圧が足りない）' if uv.get('now') else ''),
        f"AI HAT+: 温度 {_num((s.get('hailo') or {}).get('temperature_c'))} 度",
        f"ラズパイの更新: {_num((s.get('updates') or {}).get('updates_pending'))} 件",
        'ゲームパッド: ' + ('つながっている' if (d.get('joy') or {}).get('connected') else 'つながっていない'),
        'カメラ: ' + ('映っている' if (d.get('camera') or {}).get('available') else '映像が来ていない'),
    ]
    j = d.get('jetson_host') or {}
    if j.get('alive'):
        t = j.get('temperatures_c') or {}
        lines.append(f"Jetson: CPU {_num(j.get('cpu_percent'))} %、GPU {_num(j.get('gpu_percent'))} %、"
                     f"温度 {_num(t.get('tj'))} 度、メモリ {_num(j.get('mem_used_mb'))} / "
                     f"{_num(j.get('mem_total_mb'))} MB、電力 {_num(j.get('power_w'), '{:.1f}')} W")
    else:
        lines.append('Jetson: 未接続')
    return '\n'.join(lines)


def car_talk_on():
    """ミニコン・ローカルのときだけ実況する（パソコンの控えのときはしない）。"""
    return STATUS['mode'] == 'local' and PC['want'] == 'jetson' and not PRIMARY['url']


def _meter(v):
    return f'{v:.1f} メートル'


def car_talk_event(a):
    """autonomy_node の動きが変わったときの一言。"""
    side = '左' if a.get('turn_dir') == 'left' else '右'
    front = (a.get('sectors') or {}).get('front')
    return {
        'forward': '前に進みます。' + (f'障害物なし。前は {_meter(front)} 空いています。' if front else '障害物なし。'),
        'slow': '前方に障害物あり。' + (f'{_meter(front)} 先です。' if front else '') + 'ゆっくり進みます。',
        'turn': f'前方障害物あり。{side}に回ります。',
        'backoff': 'まわりがせまいです。少し下がります。',
        'cliff_back': '段差発見！下がります。',
        'cliff_turn': f'段差からはなれます。{side}に回ります。',
        'cliff_stuck': '前も後ろも段差です。止まります。',
        'stuck_back': '動けません。抜け出します。',
        'stuck': '抜け出せません。止まります。',
        'blocked': 'ライダーのデータが来ません。止まります。',
    }.get(a.get('behavior'))


def car_talk_chatter(a):
    """同じ動きが続いているときの様子。"""
    s = a.get('sectors') or {}
    front, left, right = s.get('front'), s.get('left'), s.get('right')
    vx = abs((a.get('cmd') or {}).get('linear_x') or 0.0)
    b = a.get('behavior')
    if b == 'forward':
        words = ['順調に走っています。', '障害物なし。']
        if front:
            words.append(f'障害物なし。前は {_meter(front)} 空いています。')
        if left and right:
            words.append(f'左まで {_meter(left)}、右まで {_meter(right)} です。')
        if vx:
            words.append(f'秒速 {vx * 100:.0f} センチで進んでいます。')
        return random.choice(words)
    if b == 'slow':
        return f'前の障害物まで {_meter(front)}。気をつけて進みます。' if front else '気をつけて進みます。'
    if b == 'turn':
        return random.choice(['空いている所をさがしています。', 'まだ回っています。'])
    if b == 'cliff_stuck':
        return '段差で動けません。'
    if b == 'blocked':
        return 'ライダーを待っています。'
    return None


def _ja_num(t):
    """「1.5」「三」「十五」「半」を数にする。"""
    if t == '半':
        return 0.5
    try:
        return float(t)
    except ValueError:
        pass
    d = {c: i + 1 for i, c in enumerate('一二三四五六七八九')}
    if '十' in t:
        a, _, b = t.partition('十')
        return (d.get(a, 1) if a else 1) * 10 + (d.get(b, 0) if b else 0)
    return float(d.get(t, 0)) or None


def drive_command(text):
    """聞き取りから走る命令を作る。命令でなければ None。

    {'kind': 'stop'} か {'kind': 'move' | 'turn', 'dir': ..., 'm' | 'deg' | 's': 量, 'words': 先に言う文}。
    """
    t = re.sub(r'[\s、。,.!！?？]', '', text or '').replace('名前', '')
    if not t or len(t) > DRIVE_MAX_CHARS:
        return None
    if DRIVE_STOP.search(t) and len(t) <= 12:
        return {'kind': 'stop'}
    if re.search(r'回れ右|まわれみぎ', t):
        return {'kind': 'turn', 'dir': 'right', 'deg': 180.0, 'words': '回れ右をするよ。'}
    verbs = list(DRIVE_VERB.finditer(t))
    if not verbs or not DRIVE_TAIL.fullmatch(t[verbs[-1].end():]):
        return None
    dirs = [d for d, pat in DRIVE_DIRS if re.search(pat, t)]
    turn = bool(DRIVE_TURN.search(t))
    m = re.search(DRIVE_NUM + r'?回転', t)
    if turn and m:
        n = _ja_num(m.group(1)) if m.group(1) else 1.0
        if re.search(r'半回転', t):
            n = 0.5
        deg = min(DRIVE_LIMITS['deg'], 360.0 * (n or 1.0))
        d = 'right' if 'right' in dirs else 'left'
        return {'kind': 'turn', 'dir': d, 'deg': deg, 'words': f'{DIR_WORDS[d]} {deg:g} 度回るよ。'}
    if turn and dirs and dirs[0] in ('right', 'left') or turn and not dirs:
        d = dirs[0] if dirs else 'left'
        m = re.search(DRIVE_NUM + r'(?:度|°)', t)
        deg = _ja_num(m.group(1)) if m else None
        if deg is None:
            deg = 45.0 if re.search(r'少し|すこし|ちょっと', t) else DRIVE_DEFAULT['turn']
        deg = min(DRIVE_LIMITS['deg'], deg)
        return {'kind': 'turn', 'dir': d, 'deg': deg, 'words': f'{DIR_WORDS[d]} {deg:g} 度回るよ。'}
    if not dirs:
        return None
    d = dirs[0]
    cmd = {'kind': 'move', 'dir': d}
    if (m := re.search(DRIVE_NUM + r'(メートル|m|ｍ|センチ|cm)', t, re.I)) and _ja_num(m.group(1)):
        v = _ja_num(m.group(1)) / (100.0 if m.group(2).lower() in ('センチ', 'cm') else 1.0)
        cmd['m'] = min(DRIVE_LIMITS['m'], v)
    elif (m := re.search(DRIVE_NUM + r'秒', t)) and _ja_num(m.group(1)):
        cmd['s'] = min(DRIVE_LIMITS['s'], _ja_num(m.group(1)))
    else:
        cmd['m'] = 0.2 if re.search(r'少し|すこし|ちょっと', t) else DRIVE_DEFAULT['move']
    amount = f"{cmd['m'] * 100:g} センチ" if 'm' in cmd and cmd['m'] < 1 else \
        f"{cmd['m']:g} メートル" if 'm' in cmd else f"{cmd['s']:g} 秒"
    cmd['words'] = f'{DIR_WORDS[d]} {amount}{"下がる" if d == "back" else "進む"}よ。'
    return cmd


def is_patrol(text):
    return bool(PATROL.fullmatch(re.sub(r'[\s、。！？!?,.]', '', text or '')))


def patrol_words(rt, seen, told):
    """パトロールのようすから、まだ言っていない「異常」「〇か所見た」と、終わったときの言葉を作る。"""
    an = rt.get('anomalies') or []
    state = rt.get('state')
    out = []
    if state == 'running':
        out += [f"{a['where']}に、{a['text'].split('（')[0]}よ。" for a in an[seen:]]
        done = len(rt.get('arrived') or []) + len(rt.get('skipped') or [])
        if done // PATROL_TELL_EVERY > told // PATROL_TELL_EVERY and done < len(rt.get('stops') or []):
            out.append(f"{done} か所見たよ。あと {len(rt['stops']) - done} か所。")
        return out
    if state == 'stopped':
        return ['パトロールをやめたよ。']
    if state == 'failed':
        return [f"{(rt.get('reason') or '動けなかった').split('（')[0]}から、パトロールをやめたよ。"]
    out.append('パトロール終わり。' if state == 'done' else '15 分たったから、パトロールを終わるね。')
    out.append(f'異常は {len(an)} つ。' + ''.join(f"{a['where']}に、{a['text'].split('（')[0]}。" for a in an[:5])
               if an else '異常はなかったよ。')
    if rt.get('skipped'):
        out.append(f"行けなかった所が {len(rt['skipped'])} か所あったよ。")
    return out


def patrol_banner(rt):
    """スタックちゃんの画面のふきだしに出す字（例: パトロール中 3/14 異常 1）。"""
    n = len(rt.get('stops') or [])
    done = len(rt.get('arrived') or []) + len(rt.get('skipped') or [])
    an = len(rt.get('anomalies') or [])
    return f"パトロール中 {min(done + 1, n)}/{n}" + (f' 異常 {an}' if an else '')


def find_command(text):
    """聞き取りから「覚えて」「探して」の命令を作る。命令でなければ None。"""
    t = re.sub(r'[\s、。,.!！?？「」]', '', text or '')
    if not t or len(t) > DRIVE_MAX_CHARS:
        return None
    for kind, pat in (('register', FIND_REGISTER_BARE), ('search', FIND_SEARCH_BARE)):
        if pat.fullmatch(t):
            return {'kind': kind, 'name': ''}
    for kind, pat in (('register', FIND_REGISTER), ('search', FIND_SEARCH)):
        m = pat.fullmatch(t)
        if m and m.group(1) not in ('これ', 'それ', 'あれ', 'なに', '何', 'を', 'お', 'は'):
            return {'kind': kind, 'name': m.group(1)}
    return None


def place_key(name):
    """場所の名前をくらべる形（ダッシュボードの place_key と同じ）。"""
    t = unicodedata.normalize('NFKC', name or '')
    t = re.sub(r'[\s、。,.・「」!！?？]', '', t)
    return ''.join(chr(ord(c) - 0x60) if 'ァ' <= c <= 'ヶ' else c for c in t).lower()


def place_command(text):
    """聞き取りから緑の点の命令を作る。{'kind': 'place_add', 'name'} か {'kind': 'route', 'text', 'order'}。"""
    t = re.sub(r'[\s。.!！?？「」]', '', text or '')
    if not t or len(t) > 50:
        return None
    m = PLACE_ADD.fullmatch(t.replace('、', ''))
    if m:
        return {'kind': 'place_add', 'name': m.group(1)}
    m = PLACE_ROUTE.fullmatch(t)
    if m:
        return {'kind': 'route', 'text': m.group(1), 'order': True}
    m = PLACE_GO.fullmatch(t.replace('、', ''))
    if m:
        return {'kind': 'route', 'text': m.group(1), 'order': False}
    return None


def split_places(text, known):
    """「台所、玄関」「台所と玄関」「台所玄関」を、知っている場所の名前の列にする。(名前の列, 知らない部分の列)。"""
    names = {place_key(n): n for n in known}
    parts = [p for p in PLACE_SPLIT.split(text or '') if p]
    if parts and all(place_key(p) in names for p in parts):
        return [names[place_key(p)] for p in parts], []
    # 区切りがない・「と」が名前に入っているときは、知っている名前を前からつなげてみる
    t = place_key(re.sub(r'[、,，\s]', '', text or ''))
    keys = sorted(names, key=len, reverse=True)
    best = {0: []}
    for i in range(len(t)):
        if i not in best:
            continue
        for k in keys:
            if t.startswith(k, i) and i + len(k) not in best:
                best[i + len(k)] = best[i] + [names[k]]
        for sep in ('それから', 'そして', 'つぎに', '次に', 'から', 'と'):
            if t.startswith(sep, i) and best[i] and i + len(sep) not in best:
                best[i + len(sep)] = best[i]
    if len(t) in best and best[len(t)]:
        return best[len(t)], []
    return [], [p for p in parts if place_key(p) not in names] or [text]


def route_words(rt, said, retries=0):
    """回るようすから、まだ言っていない「着いた」「まわり道」と、終わったときの 1 文を作る。"""
    out = [f'{n}に着いたよ。' for n in (rt.get('arrived') or [])[said:]]
    if rt.get('state') == 'running' and (rt.get('retries') or 0) > retries:
        out.append('ふさがっているから、まわり道をさがすね。')
    state = rt.get('state')
    if state == 'done':
        out.append('全部回ったから、止まるね。')
    elif state == 'failed':
        out.append(f"{(rt.get('reason') or '行けない所があった').split('（')[0]}から、止まったよ。")
    elif state == 'stopped':
        out.append('止まったよ。')
    return out


def finder_api(url, path, body=None, timeout=3.0):
    """物さがし（object_finder）の API。覚えていない・まん中に物がないなどは {'ok': False, 'error'}。"""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url.rstrip('/') + path, data, {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        if e.code in (400, 409):
            return json.loads(e.read() or b'{}')
        raise


def register_object(finder, name):
    """カメラのまん中の物を name（空なら名前なし）で覚えて、話す 1 文を返す。"""
    try:
        res = finder_api(finder, '/register', {'name': name}, timeout=15.0)
    except Exception as e:  # noqa: BLE001 - つながらなければ覚えない
        log(f'物さがしのエラー: {e}')
        return '物を見る AI につながらないよ。'
    if res.get('ok'):
        return f'{name}を覚えたよ。' if name else '覚えたよ。'
    log(f'物さがし: 登録できません（{res.get("error")}）')
    return 'カメラのまん中に物が見えないよ。カメラの前のまん中に置いてね。'


def search_car(url, finder, name, stop):
    """自動走行で部屋を回って name（空なら最後に覚えた物）を探す。見つけた・やめたら車を止めて、話す 1 文を返す。"""
    try:
        res = finder_api(finder, '/search', {'name': name})
    except Exception as e:  # noqa: BLE001 - つながらなければ走らない
        log(f'物さがしのエラー: {e}')
        return '物を見る AI につながらないから、探せないよ。'
    if not res.get('ok'):
        return f'{name}は、まだ覚えていないよ。' if name else 'まだ何も覚えていないよ。先に「覚えて」と言ってね。'
    try:
        lidar = (car_api(url, '/api/status') or {}).get('lidar')
        if lidar and not lidar.get('ok'):
            finder_api(finder, '/search/stop', {})
            log(f'物さがし: LiDAR が見えにくい {lidar}')
            return 'LiDAR が遠くまで見えないから、探せないよ。LiDAR のレンズをふいてね。'
        if not (car_api(url, '/api/drive_mode', {'mode': 'auto'}) or {}).get('ok'):
            finder_api(finder, '/search/stop', {})
            return '自動走行にできなかったよ。'
    except Exception as e:  # noqa: BLE001
        log(f'走行のエラー: {e}')
        return 'AI-CAR につながらないから、探せないよ。'
    t0, errors = time.monotonic(), 0
    try:
        while True:
            if stop.is_set():
                return '止まったよ。'
            time.sleep(FIND_PERIOD)
            try:
                s = finder_api(finder, '/search')
                st = car_api(url, '/api/status')
                errors = 0
            except Exception as e:  # noqa: BLE001 - 3 回続けてだめなら止める
                errors += 1
                if errors >= 3:
                    log(f'物さがしのエラー: {e}')
                    return 'ようすがわからなくなったから、止まったよ。'
                continue
            if s.get('found'):
                log(f'物さがし: {name} を見つけた（点 {s.get("score")}、枠 {s.get("box")}）')
                return f'{name}を発見しました。' if name else '見つけました。'
            if not s.get('active'):
                return '物を見る AI が止まったから、探すのをやめたよ。'
            dm = st.get('drive_mode') or {}
            if dm.get('mode') != 'auto' and time.monotonic() - t0 > 2.0:
                why = dm.get('guard') or dm.get('reason_label') or dm.get('reason')
                return f'ラズパイが自動走行をやめたから、探すのをやめたよ（{why}）。' if why else \
                    'ラズパイが自動走行をやめたから、探すのをやめたよ。'
            if time.monotonic() - t0 > FIND_TIMEOUT:
                return f'{FIND_TIMEOUT / 60:.0f} 分探したけど、' + (f'{name}は見つかりませんでした。' if name else '見つかりませんでした。')
    finally:
        for path, body in (('/api/stop', {}), ('/api/drive_mode', {'mode': 'manual'})):
            try:
                car_api(url, path, body)
            except Exception as e:  # noqa: BLE001 - 届かなくてもラズパイの安全機構は動く
                log(f'走行のエラー: {e}')
        try:
            finder_api(finder, '/search/stop', {})
        except Exception:  # noqa: BLE001
            pass


def car_api(url, path, body=None, timeout=2.0):
    """ダッシュボードの API を呼んで JSON を返す（POST は X-API-Token つき）。"""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url.rstrip('/') + path, data, {
        'Content-Type': 'application/json', 'X-API-Token': os.environ.get('AI_CAR_API_TOKEN', '')})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or b'null')


def _car_pose(st):
    """(odom の x, y, IMU の yaw [度], データが新しいか)。"""
    o, imu = st.get('odom') or {}, st.get('imu') or {}
    now = st.get('stamp') or 0
    fresh = all(x.get('stamp') is not None and now - x['stamp'] < DRIVE_STALE for x in (o, imu))
    return o.get('x'), o.get('y'), imu.get('yaw_deg'), fresh


def drive_car(url, cmd, stop):
    """命令どおりに AI-CAR を動かし、終わりに話す 1 文を返す。stop（threading.Event）が立つとすぐ止める。"""
    kind = cmd['kind']
    vel = {'forward': (1, 0, 0), 'back': (-1, 0, 0), 'left': (0, 1, 0), 'right': (0, -1, 0)}[cmd['dir']]
    if kind == 'turn':
        vel = (0, 0, 1 if cmd['dir'] == 'left' else -1)
    goal = cmd.get('deg') or cmd.get('m')
    try:
        st = car_api(url, '/api/status')
        mode = (st.get('drive_mode') or {}).get('mode')
        if mode == 'auto':
            return '今は自動走行中だから、声では動かさないよ。'
        if mode != 'manual':
            car_api(url, '/api/drive_mode', {'mode': 'manual'})
        x0, y0, yaw, fresh = _car_pose(st)
        if not fresh or x0 is None or yaw is None:
            return 'エンコーダーか IMU のデータが来ていないから、動けないよ。'
    except Exception as e:  # noqa: BLE001 - つながらなければ動かない
        log(f'走行のエラー: {e}')
        return 'AI-CAR につながらないから、動けないよ。'
    t0 = time.monotonic()
    last_yaw, turned, done, errors = yaw, 0.0, 0.0, 0
    stall_at, stall_done = t0, 0.0
    reason = None
    try:
        while True:
            if stop.is_set():
                car_api(url, '/api/stop', {})
                return '止まったよ。'
            try:
                st = car_api(url, '/api/status')
                errors = 0
            except Exception as e:  # noqa: BLE001 - 3 回続けてだめなら止める
                errors += 1
                if errors >= 3:
                    log(f'走行のエラー: {e}')
                    reason = 'AI-CAR の様子がわからなくなったから、止まったよ。'
                    break
                time.sleep(DRIVE_PERIOD)
                continue
            x, y, yaw, fresh = _car_pose(st)
            if not fresh or x is None or yaw is None:
                reason = 'エンコーダーか IMU のデータが止まったから、止まったよ。'
                break
            dm = st.get('drive_mode') or {}
            if dm.get('guard') and time.monotonic() - t0 > 0.5:
                reason = f"{dm['guard']}から、止まったよ。"
                break
            if dm.get('mode') != 'manual' and time.monotonic() - t0 > 0.5:
                why = (st.get('obstacle') or {}).get('reason') or dm.get('reason_label') or dm.get('reason')
                reason = f'ラズパイが止めたよ（{why}）。' if why else 'ラズパイが止めたよ。'
                break
            turned += (yaw - last_yaw + 180.0) % 360.0 - 180.0
            last_yaw = yaw
            done = abs(turned) if kind == 'turn' else math.hypot(x - x0, y - y0)
            now = time.monotonic()
            if 's' in cmd:
                if now - t0 >= cmd['s']:
                    break
            elif goal - done <= DRIVE_DONE[kind]:
                break
            if now - t0 > DRIVE_TIMEOUT[kind]:
                reason = '時間がかかりすぎたから、止まったよ。'
                break
            if done - stall_done >= DRIVE_STALL[1][kind]:
                stall_at, stall_done = now, done
            elif now - stall_at > DRIVE_STALL[0]:
                reason = '動けないみたいだから、止まったよ。'
                break
            k = DRIVE_SPEED[kind]
            if 's' not in cmd and goal - done < DRIVE_SLOW[kind][0]:
                k *= DRIVE_SLOW[kind][1]
            car_api(url, '/api/cmd_vel', {'linear_x': vel[0] * k, 'linear_y': vel[1] * k, 'angular_z': vel[2] * k})
            time.sleep(max(0.0, DRIVE_PERIOD - (time.monotonic() - now)))
    finally:
        for _ in range(2):
            try:
                car_api(url, '/api/cmd_vel', {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0})
            except Exception:  # noqa: BLE001 - 届かなくてもラズパイの時間切れで止まる
                pass
    if reason:
        return reason
    if kind == 'turn':
        return f'{abs(turned):.0f} 度回ったよ。'
    return f'{done * 100:.0f} センチ動いたよ。'


def llm_sentences(model, messages):
    """返事を 1 文ずつ出す。パソコンが動いていればパソコンの PC['model']、だめなら Jetson の model。"""
    if PC['up']:
        said = False
        try:
            for x in _llm_sentences(pc_url(PC_OLLAMA_PORT, '/api/chat'), PC['model'], messages, PC_LLM_TIMEOUT):
                said = True
                yield x
            return
        except OSError as e:
            if said:
                raise
            pc_down(e)
    yield from _llm_sentences(OLLAMA_URL, model, messages)


def is_chinese(s):
    """漢字ばかりで、かなが少なく、中国語だけの字がある文（qwen2.5 がときどき混ぜる中国語）。"""
    hanzi = len(HANZI.findall(s))
    return hanzi >= 6 and len(KANA.findall(s)) * 4 < hanzi and bool(CHINESE_ONLY.search(s))


def _llm_sentences(url, model, messages, timeout=60):
    body = json.dumps({'model': model, 'stream': True, 'keep_alive': -1, 'messages': messages,
                       'options': {'num_predict': 120}}).encode()
    buf = ''
    with urllib.request.urlopen(urllib.request.Request(url, body), timeout=timeout) as r:
        for line in r:
            buf += json.loads(line).get('message', {}).get('content', '')
            while (m := SENTENCE_END.search(buf)):
                s, buf = FOREIGN_TEXT.sub('', buf[:m.end()]).strip(), buf[m.end():]
                if s and not is_chinese(s):
                    yield s
    if (s := FOREIGN_TEXT.sub('', buf).strip()) and not is_chinese(s):
        yield s


def voicevox_wav(text, speaker):
    """パソコンの VOICEVOX で文字を声（wav）にする。"""
    url = pc_url(PC_VOICEVOX_PORT, '/audio_query?' + urllib.parse.urlencode({'text': text, 'speaker': speaker}))
    with urllib.request.urlopen(urllib.request.Request(url, b'', method='POST'), timeout=PC_TTS_TIMEOUT) as r:
        query = r.read()
    req = urllib.request.Request(pc_url(PC_VOICEVOX_PORT, f'/synthesis?speaker={speaker}'), query,
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=PC_TTS_TIMEOUT) as r:
        return r.read()


def web_get(url, ua=WEB_UA):
    req = urllib.request.Request(url, headers={'User-Agent': ua, 'Accept-Language': 'ja'})
    with urllib.request.urlopen(req, timeout=WEB_TIMEOUT) as r:
        return r.read().decode('utf-8', 'replace')


def home_place():
    """家の場所（市・緯度経度）。インターネットの出口の IP から 1 回だけ調べて覚える。"""
    if not HOME:
        j = json.loads(web_get('http://ip-api.com/json/?lang=ja&fields=status,city,lat,lon'))
        if j.get('status') == 'success':
            HOME.update(city=j['city'], lat=j['lat'], lon=j['lon'])
    return HOME


def weather_info(_words=None):
    h = home_place()
    if not h:
        return ''
    j = json.loads(web_get(
        f"https://api.open-meteo.com/v1/forecast?latitude={h['lat']}&longitude={h['lon']}&timezone=Asia%2FTokyo"
        '&current=temperature_2m,weather_code&forecast_days=3'
        '&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max'))
    c, d = j['current'], j['daily']
    lines = [f"{h['city']}の天気（Open-Meteo）: 今 {WMO_JA.get(c['weather_code'], '?')} {c['temperature_2m']:.0f}℃"]
    for i, name in enumerate(('今日', '明日', 'あさって')):
        lines.append(f"{name}（{d['time'][i]}）{WMO_JA.get(d['weather_code'][i], '?')}、"
                     f"最高 {d['temperature_2m_max'][i]:.0f}℃、最低 {d['temperature_2m_min'][i]:.0f}℃、"
                     f"雨の確率 {d['precipitation_probability_max'][i]}%")
    return '\n'.join(lines)


def news_info(_words=None):
    root = ET.fromstring(web_get('https://news.google.com/rss?hl=ja&gl=JP&ceid=JP:ja'))
    titles = [t.text for t in root.iter('title')][2:7]
    return 'ニュース（Google ニュース）:\n' + '\n'.join(f'- {t}' for t in titles if t)


def search_words(text):
    """質問から検索ことばを作る（パソコンの会話の AI に頼む。だめなら質問の文から決まった言い回しを除くだけ）。"""
    words = text
    try:
        body = json.dumps({'model': PC['model'], 'stream': False, 'keep_alive': -1,
                           'options': {'temperature': 0, 'num_predict': 24},
                           'messages': [{'role': 'system', 'content': SEARCH_PROMPT},
                                        {'role': 'user', 'content': text}]}).encode()
        with urllib.request.urlopen(urllib.request.Request(pc_url(PC_OLLAMA_PORT, '/api/chat'), body),
                                    timeout=WEB_TIMEOUT / 2) as r:
            words = json.loads(r.read())['message']['content'].splitlines()[0]
    except (OSError, ValueError, KeyError, IndexError) as e:
        log(f'検索ことばを作れません（{e}）')
    return ' '.join(SEARCH_FILLER.sub(' ', FOREIGN_TEXT.sub('', words.replace('\ufffd', ''))).split())[:40] or text


def news_search_info(words):
    """検索ことばで Google ニュースを検索して、新しい記事の見出しと日付を 5 つ。"""
    root = ET.fromstring(web_get('https://news.google.com/rss/search?hl=ja&gl=JP&ceid=JP:ja&q='
                                 + urllib.parse.quote(words)))
    items = [(i.findtext('title'), (i.findtext('pubDate') or '')[5:16]) for i in root.iter('item')][:5]
    return ('ニュースの検索「' + words + '」（Google ニュース）:\n'
            + '\n'.join(f'- {t}（{d}）' for t, d in items if t)) if items else ''


def ddg_info(words):
    """検索ことばで DuckDuckGo を検索して、上から 4 つのページの説明。"""
    page = web_get('https://html.duckduckgo.com/html/?q=' + urllib.parse.quote(words))
    snips = [_plain(x)[:150] for x in re.findall(r'class="result__snippet"[^>]*>(.*?)</a>', page, re.S)][:4]
    return ('ウェブの検索（DuckDuckGo）:\n' + '\n'.join(f'- {x}' for x in snips if x)) if snips else ''


def _wiki_text(v):
    v = re.sub(r'<ref[^>]*/>|<ref.*?</ref>', '', v)
    v = re.sub(r'\[\[(?:[^|\]]*\|)?([^\]]*)\]\]', r'\1', v)
    v = re.sub(r'\[https?://\S+ ?([^\]]*)\]', r'\1', v)
    v = re.sub(r'\{\{[^{}]*\}\}|\'\'+', '', v)
    v = re.sub(r'<br\s*/?>', ' ', v)
    return ' '.join(_plain(v).split())


def wiki_info(words):
    """Wikipedia（日本語）で上から 2 つの記事の、はじめの説明と右の表（現職・就任日など）。1 回で取る（何回も聞くと 429）。"""
    pages = json.loads(web_get(WIKI_API + urllib.parse.quote(words), WIKI_UA)).get('query', {}).get('pages', [])
    out = []
    for page in sorted(pages, key=lambda x: x.get('index', 0)):
        top = page['revisions'][0]['slots']['main']['content'].split('\n==')[0]
        box = []
        for k, v in re.findall(r'^\|\s*([^=|\n]+?)\s*=\s*(.+)$', top, re.M):
            if not WIKI_SKIP.search(k) and (v := _wiki_text(v)) and len(box) < 12:
                box.append(f'{k}: {v[:60]}')
        intro = ' '.join(page.get('extract', '').split())
        out.append(f"Wikipedia「{page['title']}」: {intro}" + ('\n' + '、'.join(box) if box else ''))
    return '\n'.join(out)


def web_info(query):
    """聞かれたことをネットで調べて、プロンプトに足す文にする。いっしょに調べ、WEB_TIMEOUT 秒で間に合わない所は飛ばす。"""
    words = search_words(query)
    jobs = [wiki_info, ddg_info, news_search_info]
    if NEWS_ASK.search(query):
        jobs.insert(0, news_info)
    if WEATHER_ASK.search(query):
        jobs.insert(0, weather_info)
    pool = concurrent.futures.ThreadPoolExecutor(len(jobs))
    futs = [pool.submit(job, words) for job in jobs]
    concurrent.futures.wait(futs, timeout=WEB_TIMEOUT)
    pool.shutdown(wait=False, cancel_futures=True)
    out = []
    for job, fut in zip(jobs, futs):
        if not fut.done():
            log(f'ネットで調べるのが間に合いません（{job.__name__}）')
        elif fut.exception():
            log(f'ネットで調べられません（{job.__name__}）: {fut.exception()}')
        elif fut.result():
            out.append(fut.result())
    log(f'検索ことば「{words}」: {len(out)} か所から')
    return '\n'.join(out)[:WEB_INFO_CHARS]


def pc_net():
    """パソコンを使っていて「ネット」のとき。会話はパソコンの AI のまま、ネットは調べるだけに使う。"""
    return STATUS['mode'] == 'net' and PC['want'] == 'pc' and PC['up']


def system_prompt():
    """会話の AI の最初の決まり。パソコンの VOICEVOX で話すときは、その声のキャラクター。ミニコン（Kokoro）のときはスタックちゃん。"""
    if not (PC['up'] and PC['tts'] and PC['speaker'] in PC_PERSONAS):
        return SYSTEM_PROMPT
    name, style = PC_PERSONAS[PC['speaker']]
    return PERSONA_PROMPT.format(name=name, style=style)


def tts_pcm(text, voice, pitch, peak_db, rate=OUT_RATE, use_pc=True):
    """文字を rate（ふつうは 24 kHz）・16 bit・1 ch の PCM（bytes）にする。いちばん大きい所を peak_db（dBFS）にそろえる。
    パソコンを使っていて VOICEVOX が動いていれば VOICEVOX（高さはそのまま）、ほかは Kokoro（pitch だけ上げる）。"""
    wav = None
    if use_pc and PC['up'] and PC['tts']:
        try:
            wav = voicevox_wav(text, PC['speaker'])
        except (OSError, ValueError) as e:
            PC['tts'] = False
            log(f'パソコンの声が作れないので Kokoro にします（{e}）')
            show_engines()
    pc_voice = wav is not None
    if wav is None:
        body = json.dumps({'model': 'kokoro', 'input': text, 'voice': voice, 'response_format': 'wav',
                           'lang_code': 'j', 'speed': 1.0}).encode()
        req = urllib.request.Request(KOKORO_URL, body, {'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=30) as r:
            wav = r.read()
    sox = ['sox', '-t', 'wav', '-', '-t', 'wav', '-r', str(rate), '-c', '1', '-b', '16', '-']
    if pitch and not pc_voice:
        sox += ['pitch', str(pitch)]
    sox += ['norm', str(peak_db)]
    wav = subprocess.run(sox, input=wav, capture_output=True, check=True).stdout
    with wave.open(io.BytesIO(wav)) as w:
        return w.readframes(w.getnframes())


def is_call(wake, text):
    """呼びかけ待ちで聞いた文字が、スタックちゃんを呼んだものなら True。"""
    if WAKE_ANNOTATION.search(text):
        return False
    m = wake.search(text)
    if not m or len(re.sub(r'[\s\W]', '', text[:m.start()])) > WAKE_LEAD:
        return False
    if WAKE_TOPIC.match(text, m.end()):
        return False
    rest = wake.sub('', text)
    return not (STT_PROMPT_WORDS.search(rest) and not re.sub(r'[\s\W]', '', STT_PROMPT_WORDS.sub('', rest)))


def pc_url(port, path):
    return f"http://{PC['host']}:{port}{path}"


def show_engines():
    set_status(engines=engines(PC['args']),
               backend={'want': PC['want'], 'active': 'pc' if PC['up'] else 'jetson', 'error': PC['error']},
               pc_voice={'speaker': PC['speaker'],
                         'choices': [{'speaker': k, 'name': v} for k, v in PC_VOICES.items()]})


def set_pc_voice(speaker):
    """ダッシュボードの声の選択（パソコンの VOICEVOX）。次の文から変わる。"""
    PC['speaker'] = speaker
    with open(PC_VOICE_FILE, 'w') as f:
        f.write(f'{speaker}\n')
    log(f'パソコンの声: {PC_VOICES[speaker]}（{speaker}）')
    PC_WAKE.set()
    show_engines()


def set_backend(want):
    """ダッシュボードの切り替え: jetson ならパソコンを使わない。pc ならパソコンを見て、動いていれば使う。"""
    PC['want'] = want
    with open(BACKEND_FILE, 'w') as f:
        f.write(want + '\n')
    log(f'聞き取り・返事をする所: {"パソコン" if want == "pc" else "Jetson"}')
    if want == 'jetson':
        PC.update(up=False, error=None)
    PC_WAKE.set()
    show_engines()


def switch_backend(want):
    """「ミニコンにして」「パソコンにして」。返事の文を返す。"""
    if PRIMARY['url'] and want == 'jetson':
        return STANDBY_ONLY
    if want == 'pc' and not PC['host']:
        return 'パソコンが決められていないので、ミニコンで話します。'
    was = PC['want']
    set_backend(want)
    if want == 'jetson':
        return '今はミニコンです。' if was == want else 'ミニコンにしました。'
    if PC['up']:
        return '今はパソコンです。'
    return 'パソコンにします。つながるまでは、ミニコンで話します。'


def engines(args):
    if PC['up']:
        tts = f"VOICEVOX {PC_VOICES[PC['speaker']]}（パソコン）" if PC['tts'] else f'Kokoro {args.voice}'
        return {'stt': 'whisper-server large-v3（パソコン）', 'llm': f"{PC['model']}（パソコン）", 'tts': tts}
    return {'stt': 'whisper-server', 'llm': args.model, 'tts': f'Kokoro {args.voice}'}


def pc_down(e):
    PC['error'] = str(e)
    if PC['up']:
        PC['up'] = False
        log(f'パソコンにつながらないので Jetson に戻します（{e}）')
    show_engines()


def pc_check():
    """パソコンの whisper-server が返事をし、Ollama に PC['model'] があれば読み込ませる（読み込み済みならすぐ返る）。だめなら OSError。"""
    urllib.request.urlopen(pc_url(PC_WHISPER_PORT, '/'), timeout=PC_CHECK_TIMEOUT).read()
    with urllib.request.urlopen(pc_url(PC_OLLAMA_PORT, '/api/tags'), timeout=PC_CHECK_TIMEOUT) as r:
        names = {m.get('name') for m in json.loads(r.read()).get('models', [])}
    if PC['model'] not in names:
        raise OSError(f"Ollama に {PC['model']} がない")
    body = json.dumps({'model': PC['model'], 'keep_alive': -1}).encode()
    urllib.request.urlopen(urllib.request.Request(pc_url(PC_OLLAMA_PORT, '/api/generate'), body),
                           timeout=PC_WARM_TIMEOUT).read()
    url = pc_url(PC_VOICEVOX_PORT, f"/initialize_speaker?speaker={PC['speaker']}&skip_reinit=true")
    try:
        urllib.request.urlopen(urllib.request.Request(url, b'', method='POST'), timeout=PC_WARM_TIMEOUT).read()
        tts = True
    except OSError:
        tts = False
    if tts != PC['tts']:
        PC['tts'] = tts
        log(f"パソコンの声（VOICEVOX）: {'使います' if tts else 'つながらないので Kokoro'}")
        show_engines()


async def pc_watch(args):
    """PC_CHECK_EVERY 秒ごと（切り替えたらすぐ）にパソコンを見て、聞き取りと返事の行き先を切り替える。"""
    while True:
        PC_WAKE.clear()
        if PC['want'] == 'pc':
            try:
                await asyncio.to_thread(pc_check)
                if PC['want'] == 'pc' and not PC['up']:
                    PC.update(up=True, error=None)
                    log(f"パソコン（{PC['host']}）を使います: whisper large-v3・{PC['model']}")
                    show_engines()
            except (OSError, ValueError) as e:
                pc_down(e)
        await asyncio.to_thread(PC_WAKE.wait, PC_CHECK_EVERY)


async def stt_watchdog(voice, wake_words):
    """Jetson の whisper-server が決まった言葉しか返さなくなったら、プロセスを止める（Restart=on-failure で起動し直る）。"""
    wake = re.compile(wake_words or WAKE_WORDS, re.I)
    probes = {}
    while True:
        await asyncio.sleep(STT_CHECK_EVERY)
        for said, expect in STT_CHECK:
            try:
                if said not in probes:
                    pcm = await asyncio.to_thread(tts_pcm, said, voice, 0, -3.0, IN_RATE, False)
                    probes[said] = np.frombuffer(pcm, np.int16).astype(np.float32)
                text = await asyncio.to_thread(stt, probes[said], WHISPER_URL)
            except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as e:
                log(f'whisper の見回り: 試せませんでした（{e}）')
                break
            if bool(wake.search(text)) != expect or NOISE_TEXT.fullmatch(text):
                log(f'whisper がおかしいので起動し直します（「{said}」→「{text}」）')
                subprocess.run(['pkill', '-KILL', '-x', 'whisper-server'], check=False)
                break


def primary_status():
    with urllib.request.urlopen(PRIMARY['url'] + '/status', timeout=PRIMARY_TIMEOUT) as r:
        return json.loads(r.read())


def mirror_primary(st):
    """ミニコンで選んでいる「ネット / ローカル」と声を、こちらにも入れる。"""
    if st.get('mode') in ('net', 'local') and st['mode'] != STATUS['mode']:
        save_mode(st['mode'])
        log(f"ミニコンに合わせて {st['mode']} にしました")
    speaker = (st.get('pc_voice') or {}).get('speaker')
    if speaker in PC_VOICES and speaker != PC['speaker']:
        set_pc_voice(speaker)


async def primary_watch():
    """パソコンの控え: ミニコンの bridge が動いていれば選択をまね、スタックちゃんがここで呼びかけ待ちなら
    切って、ミニコンへつなぎ直させる（ファームウェアはミニコンを先に試す）。"""
    while True:
        try:
            st = await asyncio.to_thread(primary_status)
        except (OSError, ValueError) as e:
            if PRIMARY['up']:
                log(f'ミニコンの bridge につながりません（{e}）')
            PRIMARY['up'] = 0
        else:
            PRIMARY['up'] += 1
            if not SESSIONS:
                mirror_primary(st)
            elif (PRIMARY['up'] >= PRIMARY_HANDBACK_CHECKS
                  and all(s.asleep() and not s.busy() and not s.cloud for s in SESSIONS)):
                log('ミニコンがもどったので、スタックちゃんをミニコンへ返します')
                for s in list(SESSIONS):
                    await s.ws.close()
        set_status(standby=PRIMARY['up'] == 0)
        await asyncio.sleep(PRIMARY_CHECK_EVERY)


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
        elif path == '/backend':
            want = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('to', [''])[0]
            if want not in BACKENDS:
                return self._send_error(400, 'to は jetson か pc')
            if want == 'pc' and not PC['host']:
                return self._send_error(409, 'パソコン（--pc-host）が決まっていません')
            if PRIMARY['url'] and want == 'jetson':
                return self._send_error(409, STANDBY_ONLY)
            set_backend(want)
            with STATUS_LOCK:
                self._send_json({'ok': True, 'backend': STATUS['backend']})
        elif path == '/pc_voice':
            try:
                speaker = int(urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('speaker', [''])[0])
            except ValueError:
                speaker = None
            if speaker not in PC_VOICES:
                return self._send_error(400, f'speaker は {list(PC_VOICES)} のどれか')
            set_pc_voice(speaker)
            with STATUS_LOCK:
                self._send_json({'ok': True, 'pc_voice': STATUS['pc_voice']})
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
            light = frame_light(jpeg)
            if light is not None:
                LIGHT[:] = [x for x in LIGHT if now - x[0] < BRIGHT_EVERY] + [(now, light)]
            t = FRAME['times'] = [x for x in FRAME['times'][-9:] if now - x < FRAME_STALE] + [now]
            fps = round((len(t) - 1) / (t[-1] - t[0]), 1) if len(t) > 1 else None
            set_status(stream_at=now, frame_at=now, fps=fps)
            if WATCH['interval'] > 0 and not WATCH['busy'] and now >= WATCH['next']:
                WATCH.update(busy=True, next=now + WATCH['interval'])
                threading.Thread(target=watch_check, args=(jpeg,), daemon=True).start()
        self._send_body(want_frames(), 'text/plain')

    def _shot(self):
        """「撮影して」で数えて撮った写真（image/jpeg）。"""
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
        self.drive_task = None
        self.drive_stop = None
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
        self.wake = re.compile(args.wake_words, re.I) if args.wake_words else None
        self.awake_until = 0.0
        self.awake = False
        self.cloud_tts = False
        self.keep_task = None
        self.bright_tool = False
        self.patrolling = False
        self.bright = None
        self.dozing = False
        self.doze_at = 0.0
        self.head = [0, 0]
        self.idle_head_at = 0.0
        self.head_quiet_until = 0.0

    def asleep(self):
        """呼びかけ待ち（呼ばれていないか、最後に話してから --awake-seconds たった）なら True。"""
        return bool(self.wake) and time.time() >= self.awake_until

    def wake_up(self):
        self.awake_until = time.time() + self.args.awake_seconds
        self.awake = True
        self.doze_at = 0.0

    def busy(self):
        turn = self.turn and time.time() - self.turn['at'] < 60
        return bool(self.reply_task or self.switching or self.shooting or turn or self.cloud_tts or self.started
                    or STATUS['state'] in ('thinking', 'speaking'))

    def listen_state(self):
        if not self.listening:
            return 'idle'
        return 'waiting' if self.asleep() else 'listening'

    async def keepalive(self):
        """会話モードが 30 秒だまったら「じゃあ、またね。」で呼びかけ待ちにもどし、その 30 秒後に居眠りする。
        60 秒ごとに 1 回送ってスタックちゃんの 120 秒の時間切れを防ぎ、画面の明るさを映像に合わせる。"""
        last = bright_last = time.monotonic()
        battery_last = last - BATTERY_EVERY + 30.0
        while True:
            await asyncio.sleep(1)
            try:
                if self.awake and self.asleep() and not self.busy():
                    self.awake = False
                    log(f'{self.args.awake_seconds:.0f} 秒話さなかったので、呼びかけ待ちにもどります')
                    await self.say(BYE_REPLY, wake=False)
                    await self.close_cloud()
                    set_status(state=self.listen_state())
                    if self.args.doze_seconds > 0:
                        self.doze_at = time.time() + self.args.doze_seconds
                if self.doze_at and time.time() >= self.doze_at and self.asleep() and not self.busy():
                    self.doze_at = 0.0
                    await self.doze()
                if self.dozing and not self.asleep():
                    await self.wake_face()
                if (self.asleep() and not self.dozing and not self.busy() and not self.patrolling
                        and time.monotonic() >= self.idle_head_at):
                    self.idle_head_at = time.monotonic() + random.uniform(*IDLE_HEAD_EVERY)
                    await self.idle_head()
                if time.monotonic() - bright_last >= BRIGHT_EVERY:
                    bright_last = time.monotonic()
                    await self.adjust_brightness()
                if time.monotonic() - battery_last >= BATTERY_EVERY and not self.busy():
                    battery_last = time.monotonic()
                    await self.read_battery()
                if STATUS['state'] == 'listening' and not self.busy() and self.asleep():
                    set_status(state='waiting')
                if time.monotonic() - last >= KEEPALIVE:
                    last = time.monotonic()
                    await self.send(type='ping')
            except ConnectionClosed:
                return

    async def doze(self):
        """居眠りの顔にして、首をホーム（正面・水平）にもどす。呼ばれたら wake_face でふつうの顔へ。"""
        self.dozing = True
        log('居眠りします（顔を居眠りにして、首をホームへ）')
        set_status(dozing=True)
        self.head = [0, 0]
        await self.send(type='llm', emotion='sleepy')
        head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': HOME_HEAD})
        await sending
        head.cancel()

    async def idle_head(self):
        """純正の待ち受けの首ふりと同じ 4 種類（見回す・少し動く・ちらっと見る・正面にもどす）から 1 つ。"""
        r = random.random()
        if r < 0.5:
            yaw, pitch, speed = random.randint(-30, 30), random.randint(0, 20), random.randint(150, 300)
        elif r < 0.8:
            yaw = max(-80, min(80, self.head[0] + random.randint(-15, 15)))
            pitch = max(0, min(60, self.head[1] + random.randint(-8, 8)))
            speed = random.randint(100, 250)
        elif r < 0.9:
            yaw, pitch, speed = random.randint(-50, 50), random.randint(10, 40), random.randint(250, 400)
        else:
            yaw, pitch, speed = 0, random.randint(5, 40), random.randint(100, 300)
        self.head = [yaw, pitch]
        self.head_quiet_until = time.monotonic() + IDLE_HEAD_QUIET
        head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': {
            'yaw': yaw, 'pitch': pitch, 'speed': speed}})
        await sending
        head.cancel()

    async def wake_face(self):
        if self.dozing:
            self.dozing = False
            set_status(dozing=False)
            await self.send(type='llm', emotion='neutral')

    async def adjust_brightness(self):
        """見守りの映像の明るさ（1 分の中央値）で、画面の明るさを変える（夜の暗い部屋で明るすぎないように）。"""
        now = time.time()
        lights = [v for t, v in list(LIGHT) if now - t < BRIGHT_EVERY]
        if not self.bright_tool or len(lights) < BRIGHT_MIN_FRAMES:
            return
        light = float(np.median(lights))
        set_status(light=round(light))
        level = screen_level(light)
        if level == self.bright or screen_level(max(light - BRIGHT_MARGIN, 0)) != screen_level(
                min(light + BRIGHT_MARGIN, 255)):
            return
        fut, sending = self.mcp_call('tools/call', {'name': BRIGHT_TOOL, 'arguments': {'brightness': level}})
        await sending
        try:
            await asyncio.wait_for(fut, 5)
        except asyncio.TimeoutError:
            log('MCP: 画面の明るさの返事がありません')
            return
        log(f'画面の明るさ: {self.bright} → {level}（映像の明るさ {light:.0f}）')
        self.bright = level
        set_status(screen=level)

    async def read_battery(self):
        """電池の量（%）と充電中かを記録する（USB につないだまま電池がへって切れていないかを見る）。"""
        fut, sending = self.mcp_call('tools/call', {'name': STATUS_TOOL, 'arguments': {}})
        await sending
        try:
            res = await asyncio.wait_for(fut, 5)
        except asyncio.TimeoutError:
            log('MCP: 本体の状態の返事がありません')
            return
        try:
            text = ''.join(c.get('text', '') for c in (res.get('result') or {}).get('content', []))
            battery = json.loads(text).get('battery')
        except (ValueError, AttributeError):
            battery = None
        if not isinstance(battery, dict):
            return
        log(f'電池: {battery.get("level")}%（充電{"中" if battery.get("charging") else "していない"}）')
        power = json.loads(text).get('power')
        if isinstance(power, dict) and power != getattr(self, 'power_logged', None):
            self.power_logged = power
            log(f'前に切れた理由: {power_bits(power.get("off_source"), POWER_OFF_BITS)} / '
                f'入った理由: {power_bits(power.get("on_source"), POWER_ON_BITS)} / '
                f'起動のしかた: {power_bits(power.get("reset_reason"), RESET_REASONS, index=True)}'
                f'{clear_note(power.get("off_after_clear"))}')
        set_status(battery={**battery, 'at': time.time()})

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
        self.bright_tool = BRIGHT_TOOL in names
        set_status(camera=self.camera, shot_tool=self.shot)
        log(f'MCP: カメラ {"あり" if self.camera else "なし"} → {vision_url}、顔の色 {"あり" if self.face else "なし"}、'
            f'画面の明るさ {"あり" if self.bright_tool else "なし"}')

    async def after_hello(self, vision_url):
        self.vision_url = vision_url
        if vision_url:
            await self.mcp_init(vision_url)
        if STATUS['mode'] == 'net' and not self.wake:
            await self.to_net(announce=False)
        else:
            await self.set_face(STATUS['mode'])
        if self.wake and self.args.doze_seconds > 0:
            self.doze_at = time.time() + self.args.doze_seconds

    async def set_face(self, mode):
        if not self.face:
            return
        fut, sending = self.mcp_call('tools/call', {'name': FACE_TOOL, 'arguments': {'color': FACE_COLOR[mode]}})
        await sending
        try:
            await asyncio.wait_for(fut, 5)
        except asyncio.TimeoutError:
            log('MCP: 顔の色の返事がありません')

    async def say(self, text, wake=True):
        """Jetson の声で 1 文だけ話す（切り替えのお知らせ）。wake なら、そのあと会話モードにする。"""
        a = self.args
        await self.wake_face()
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
        if wake:
            self.wake_up()

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

    async def patrol_follow(self):
        """ダッシュボードから始めたパトロールも、声で始めたときと同じに画面へ「パトロール中」を出す。"""
        url = self.args.car_url
        while True:
            await asyncio.sleep(CAR_TALK_POLL)
            if self.patrolling or self.drive_task:
                continue
            try:
                rt = json.loads(await asyncio.to_thread(_get, f'{url}/api/status', 2)).get('route') or {}
            except Exception:  # noqa: BLE001 - ラズパイが止まっていても会話は続ける
                continue
            if rt.get('kind') == 'patrol' and rt.get('state') == 'running' and rt.get('id'):
                log(f"パトロール（ダッシュボードから）: 見る所 {sum(1 for q in rt.get('stops') or [] if not q.get('home'))} か所")
                stop = self.drive_stop = threading.Event()
                stop.follow = True  # スタックちゃんが切れても、車のパトロールは止めない（「止まって」では止める）
                self.drive_task = asyncio.create_task(self.patrol(rt['id'], stop))

    async def car_talk(self):
        """自動運転＆ミニコン＆ローカルのあいだ、AI-CAR の動きをしゃべる（会話のじゃまはしない）。"""
        url = self.args.car_url
        prev_mode = prev_b = None
        said = 0.0
        while True:
            await asyncio.sleep(CAR_TALK_POLL)
            if not self.args.car_talk or not car_talk_on():
                prev_mode = prev_b = None
                continue
            try:
                d = json.loads(await asyncio.to_thread(_get, f'{url}/api/status', 2))
            except Exception:  # noqa: BLE001 - ラズパイが止まっていても会話は続ける
                continue
            mode = (d.get('drive_mode') or {}).get('mode')
            a = d.get('autonomy') or {}
            b = a.get('behavior') if mode == 'auto' else None
            words = []
            if prev_mode is not None and mode != prev_mode:
                if mode == 'auto':
                    words.append('自動運転を始めます。')
                elif prev_mode == 'auto':
                    words.append('自動運転を終わります。')
            if b and b != prev_b:
                words.append(car_talk_event(a) or '')
            elif b and time.monotonic() - said >= CAR_TALK_EVERY:
                words.append(car_talk_chatter(a) or '')
            prev_mode, prev_b = mode, b
            text = ''.join(words)
            if not text or self.busy():
                continue
            log(f'実況: {text}')
            await self.say(text, wake=False)
            said = time.monotonic()

    async def to_net(self, announce=True):
        """ネットの XiaoZhi につなぎ、このセッションの声と文字を中継する。だめならローカルのまま。"""
        if PC['want'] == 'pc' and PC['up']:
            save_mode('net')
            await self.set_face('net')
            log('ネット: パソコンの AI で話し、新しいことだけネットで調べます')
            if announce:
                await self.say('ネットで調べながら話します。')
            set_status(state=self.listen_state())
            return
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
            set_status(state=self.listen_state())
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
            set_status(state=self.listen_state())

    async def close_cloud(self):
        """呼びかけ待ちのあいだはネットの会話を閉じておく（ネットは 1 分ほどだまると自分で「じゃあな」と言って切る）。"""
        cloud, self.cloud = self.cloud, None
        if cloud:
            await cloud.close()

    async def need_cloud(self):
        """ネットのモードで呼ばれたら、ネットの会話を開く。開けたら True。"""
        if STATUS['mode'] == 'net' and not self.cloud and not pc_net():
            await self.to_net(announce=False)
        return bool(self.cloud)

    async def to_cloud(self, msg):
        try:
            await self.cloud.send(json.dumps({**msg, 'session_id': self.cloud_sid}, ensure_ascii=False))
        except (ConnectionClosed, AttributeError):
            pass

    async def cloud_speech(self, samples):
        """呼びかけ待ちで聞いた声（「スタックちゃん、〜」）を、Opus にしなおしてネットへ送る。"""
        enc = opuslib.Encoder(IN_RATE, 1, opuslib.APPLICATION_VOIP)
        step = IN_RATE * FRAME_MS // 1000
        pcm = np.clip(samples, -32768, 32767).astype(np.int16)
        pcm = np.concatenate([pcm, np.zeros(-len(pcm) % step, np.int16)])
        await self.to_cloud({'type': 'listen', 'state': 'start', 'mode': self.mode})
        for off in range(0, len(pcm), step):
            await self.cloud_audio(enc.encode(pcm[off:off + step].tobytes(), step))
        self.wake_up()

    async def cloud_audio(self, data):
        try:
            await self.cloud.send(data)
        except (ConnectionClosed, AttributeError):
            pass

    async def relay_cloud(self, cloud):
        """ネットからの声と文字をスタックちゃんへ流す。「ローカルにして」が聞こえたらもどる。
        パソコンの声（VOICEVOX）が使えるときは、ネットの声は捨てて、返事の文をパソコンの声で読む。"""
        hold, own = None, False
        try:
            async for m in cloud:
                if cloud is not self.cloud:
                    return
                if isinstance(m, bytes):
                    if self.shooting or own:
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
                if call.get('method') == 'tools/call' and \
                        (call.get('params') or {}).get('name') in (SEARCH_TOOL, CAR_TOOL):
                    asyncio.create_task(self.cloud_tool(call))
                    continue
                if self.shooting and msg.get('type') in ('tts', 'llm'):
                    continue
                if msg.get('type') == 'tts' and msg.get('state') == 'start':
                    own = PC['up'] and PC['tts']
                    hold = None if own else []
                    self.cancel.clear()
                elif msg.get('type') == 'tts' and msg.get('state') == 'stop' and hold:
                    for f in hold:
                        await self.ws.send(f)
                    hold = None
                msg['session_id'] = self.sid
                await self.ws.send(json.dumps(msg, ensure_ascii=False))
                self.note_cloud(msg)
                if own and msg.get('type') == 'tts' and msg.get('state') == 'sentence_start' and msg.get('text'):
                    await self.speak_own(msg['text'])
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
            log('ネットの会話が切れました（次に呼ばれたら、つなぎ直します）')
            self.cloud = None
            if self.cloud_tts:
                self.cloud_tts = False
                try:
                    await self.send(type='tts', state='stop')
                except ConnectionClosed:
                    pass

    async def speak_own(self, text):
        """ネットの返事の 1 文を、こちらの声（パソコンの VOICEVOX、だめなら Kokoro）で読む。"""
        a = self.args
        try:
            pcm = await asyncio.to_thread(tts_pcm, text, a.voice, a.pitch, a.peak_db)
        except Exception as e:  # noqa: BLE001 - 声が作れなくても中継は続ける
            log(f'声のエラー: {e}')
            return
        await self.play(pcm)

    async def cloud_tool(self, call):
        """ネットの XiaoZhi が呼んだ調べもの・AI-CAR のようすに、スタックちゃんのかわりに答える。"""
        params = call.get('params') or {}
        if params.get('name') == CAR_TOOL:
            found = await asyncio.to_thread(car_status, self.args.car_url)
            log(f'AI-CAR のようす（ネット）: {found.splitlines()[0]}')
        else:
            found = await asyncio.to_thread(web_search, str((params.get('arguments') or {}).get('query', '')))
        await self.to_cloud({'type': 'mcp', 'payload': {'jsonrpc': '2.0', 'id': call.get('id'), 'result': {
            'content': [{'type': 'text', 'text': found}], 'isError': False}}})

    def note_cloud(self, msg):
        """ネットの会話の様子をダッシュボード用に残す。"""
        t, now = msg.get('type'), time.time()
        if t == 'tts' and msg.get('state') in ('start', 'stop'):
            self.cloud_tts = msg['state'] == 'start'
            self.wake_up()
        if t == 'stt':
            self.wake_up()
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
            set_status(state=self.listen_state())

    async def look_up(self):
        """撮影の首の向き。待ち受けの首ふりで動かされても、数えているあいだ 1 秒ごとにもどす。"""
        head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': {
            'yaw': 0, 'pitch': self.args.shot_pitch, 'speed': 300}})
        await sending
        head.cancel()

    async def start_drive(self, cmd):
        """声の走る命令。前の走りは止めてから始める。「止まって」はすぐ止める。"""
        if self.drive_stop:
            self.drive_stop.set()
        if self.drive_task:
            await self.drive_task
        if cmd['kind'] == 'stop':
            try:
                await asyncio.to_thread(car_api, self.args.car_url, '/api/stop', {})
                log('走行: 止めました')
                await self.say('止まったよ。')
            except Exception as e:  # noqa: BLE001 - 止められなくても会話は続ける
                log(f'走行のエラー: {e}')
                await self.say('AI-CAR につながらないよ。')
            return
        log(f'走行: {cmd}')
        await self.say(cmd['words'])
        self.drive_stop = threading.Event()
        self.drive_task = asyncio.create_task(self.drive(cmd, self.drive_stop))

    async def start_place(self, cmd):
        """緑の点の命令。「〇〇に行って」の〇〇が知らない場所なら False（声で走る命令として続けて見る）。"""
        url = self.args.car_url
        if cmd['kind'] == 'place_add':
            try:
                res = await asyncio.to_thread(car_api, url, '/api/places',
                                              {'action': 'add', 'name': cmd['name'], 'here': True})
            except Exception as e:  # noqa: BLE001
                log(f'緑の点のエラー: {e}')
                await self.say('AI-CAR につながらないよ。')
                return True
            log(f"緑の点: {cmd['name']} を置く → {res}")
            await self.say(f"ここを{cmd['name']}として覚えたよ。" if res.get('ok')
                           else f"覚えられなかったよ。{res.get('error') or ''}")
            return True
        try:
            known = [p['name'] for p in (await asyncio.to_thread(car_api, url, '/api/places')).get('places') or []]
        except Exception as e:  # noqa: BLE001
            log(f'緑の点のエラー: {e}')
            if not cmd['order']:
                return False
            await self.say('AI-CAR につながらないよ。')
            return True
        names, unknown = split_places(cmd['text'], known)
        if not names:
            if not cmd['order']:
                return False
            await self.say(f"{'、'.join(unknown)}は、知らない場所だよ。" if known else 'まだ地図に場所の点がないよ。')
            return True
        if self.drive_stop:
            self.drive_stop.set()
        if self.drive_task:
            await self.drive_task
        try:
            res = await asyncio.to_thread(car_api, url, '/api/route', {'action': 'go', 'names': names})
        except Exception as e:  # noqa: BLE001
            log(f'緑の点のエラー: {e}')
            await self.say('AI-CAR につながらないよ。')
            return True
        log(f'緑の点を回る: {names} → {res.get("ok")} {res.get("error") or ""}')
        if not res.get('ok'):
            await self.say(f"動けなかったよ。{res.get('error') or ''}")
            return True
        await self.say(f"{'、'.join(names)}の順に行くね。" if len(names) > 1 else f'{names[0]}に行くね。')
        stop = self.drive_stop = threading.Event()
        self.drive_task = asyncio.create_task(self.route(res['route']['id'], stop))
        return True

    async def start_patrol(self):
        """「パトロールして」。地図の全部の「見る所」と緑の点を回る。「止まって」で止まる。"""
        url = self.args.car_url
        if self.drive_stop:
            self.drive_stop.set()
        if self.drive_task:
            await self.drive_task
        try:
            res = await asyncio.to_thread(car_api, url, '/api/patrol', {'action': 'go'})
        except Exception as e:  # noqa: BLE001
            log(f'パトロールのエラー: {e}')
            await self.say('AI-CAR につながらないよ。')
            return
        log(f'パトロール: {res.get("ok")} {res.get("error") or ""}')
        if not res.get('ok'):
            await self.say(f"パトロールできなかったよ。{res.get('error') or ''}")
            return
        n = sum(1 for q in res['route']['stops'] if not q.get('home'))
        await self.say(f"パトロールするね。見る所は {n} か所。")
        stop = self.drive_stop = threading.Event()
        self.drive_task = asyncio.create_task(self.patrol(res['route']['id'], stop))

    async def patrol_head(self, i):
        self.head = [PATROL_HEAD[i % 2]['yaw'], PATROL_HEAD[i % 2]['pitch']]
        head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': PATROL_HEAD[i % 2]})
        await sending
        head.cancel()

    async def patrol(self, route_id, stop):
        """顔をオレンジにして、見ているあいだ首を左右にふる。異常・5 か所ごと・終わりを話す。"""
        url, seen, told, errors, sweep, head_at = self.args.car_url, 0, 0, 0, 0, 0.0
        banner, banner_at, lidar_told = '', 0.0, False
        self.patrolling = True
        await self.set_face('patrol')
        try:
            while True:
                await asyncio.sleep(FIND_PERIOD)
                if stop.is_set():
                    await asyncio.to_thread(car_api, url, '/api/patrol', {'action': 'cancel'})
                    return
                try:
                    rt = (await asyncio.to_thread(car_api, url, '/api/status')).get('route') or {}
                    errors = 0
                except Exception as e:  # noqa: BLE001 - 3 回続けてだめなら止める
                    errors += 1
                    if errors >= 3:
                        log(f'パトロールのエラー: {e}')
                        await self.say('ようすがわからなくなったよ。')
                        return
                    continue
                if rt.get('id') != route_id:
                    return
                said = False
                if rt.get('lidar_warn') and not lidar_told:
                    await self.say('LiDAR が見えにくいよ。レンズをふいてね。')
                    lidar_told = said = True
                for words in patrol_words(rt, seen, told):
                    await self.say(words)
                    said = True
                seen = len(rt.get('anomalies') or [])
                told = len(rt.get('arrived') or []) + len(rt.get('skipped') or [])
                if rt.get('state') != 'running':
                    log(f"パトロール: {rt.get('state')} {rt.get('reason') or ''} 異常 {seen}")
                    return
                text = patrol_banner(rt)
                if not self.busy() and (said or text != banner or time.monotonic() - banner_at >= PATROL_BANNER_EVERY):
                    # 声なしの sentence_start は、画面のふきだしの字だけを変える（つぎに話すまで残る）
                    await self.send(type='tts', state='sentence_start', text=text)
                    banner, banner_at = text, time.monotonic()
                if rt.get('look') is not None and not self.busy() and time.monotonic() >= head_at:
                    head_at = time.monotonic() + PATROL_HEAD_EVERY
                    sweep += 1
                    await self.patrol_head(sweep)
        except ConnectionClosed:
            pass
        except Exception as e:  # noqa: BLE001
            log(f'パトロールのエラー: {e}')
        finally:
            self.patrolling = False
            try:
                await self.set_face(STATUS['mode'])
                head, sending = self.mcp_call('tools/call', {'name': HEAD_TOOL, 'arguments': HOME_HEAD})
                await sending
                head.cancel()
                self.head = [0, 0]
            except Exception:  # noqa: BLE001
                pass
            if self.drive_stop is stop:
                self.drive_task = self.drive_stop = None

    async def route(self, route_id, stop):
        """回るようすを見て、点に着くたびに話す。「止まって」（stop）でやめる。"""
        url, said, retries, errors = self.args.car_url, 0, 0, 0
        try:
            while True:
                await asyncio.sleep(FIND_PERIOD)
                if stop.is_set():
                    await asyncio.to_thread(car_api, url, '/api/route', {'action': 'cancel'})
                    return
                try:
                    rt = (await asyncio.to_thread(car_api, url, '/api/status')).get('route') or {}
                    errors = 0
                except Exception as e:  # noqa: BLE001 - 3 回続けてだめなら止める
                    errors += 1
                    if errors >= 3:
                        log(f'緑の点のエラー: {e}')
                        await self.say('ようすがわからなくなったよ。')
                        return
                    continue
                if rt.get('id') != route_id:
                    return
                for words in route_words(rt, said, retries):
                    await self.say(words)
                said, retries = len(rt.get('arrived') or []), rt.get('retries') or 0
                if rt.get('state') != 'running':
                    log(f"緑の点を回る: {rt.get('state')} {rt.get('reason') or ''}")
                    return
        except ConnectionClosed:
            pass
        except Exception as e:  # noqa: BLE001
            log(f'緑の点のエラー: {e}')
        finally:
            if self.drive_stop is stop:
                self.drive_task = self.drive_stop = None

    async def start_find(self, cmd):
        """「〇〇を覚えて」「〇〇を探して」。探すあいだは走る命令と同じく「止まって」で止まる。"""
        a = self.args
        if self.drive_stop:
            self.drive_stop.set()
        if self.drive_task:
            await self.drive_task
        log(f'物さがし: {cmd}')
        if cmd['kind'] == 'register':
            await self.say(await asyncio.to_thread(register_object, a.finder_url, cmd['name']))
            return
        await self.say(f"{cmd['name']}を探すね。" if cmd['name'] else '探すね。')
        stop = self.drive_stop = threading.Event()
        self.drive_task = asyncio.create_task(self.find(cmd['name'], stop))

    async def find(self, name, stop):
        try:
            words = await asyncio.to_thread(search_car, self.args.car_url, self.args.finder_url, name, stop)
            log(f'物さがし: {words}')
            await self.say(words)
        except ConnectionClosed:
            pass
        finally:
            if self.drive_stop is stop:
                self.drive_task = self.drive_stop = None

    async def drive(self, cmd, stop):
        try:
            words = await asyncio.to_thread(drive_car, self.args.car_url, cmd, stop)
            log(f'走行: {words}')
            if not (self.cancel.is_set() and stop.is_set()):
                await self.say(words)
        except ConnectionClosed:
            pass
        finally:
            if self.drive_stop is stop:
                self.drive_task = self.drive_stop = None

    async def countdown_shot(self):
        """「撮影して」: 正面を向いて SHOT_SECONDS 秒数えて撮ってもらい、だれの顔かを見る。"""
        self.shooting = True
        set_status(shooting=True)
        try:
            if not self.shot:
                await self.say('ごめんね。今のプログラムでは、撮影できないよ。')
                return
            await self.say('撮影します。こっちを向いてね。')
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
                tools.append({'name': CAR_TOOL, 'description': CAR_DESC, 'inputSchema': CAR_PARAMS})
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
            if not self.wake:
                self.wake_up()
            if self.wake and not self.keep_task:
                self.keep_task = asyncio.create_task(self.keepalive())
            asyncio.create_task(self.after_hello(vision_url))
        elif t == 'listen':
            state = msg.get('state')
            if state == 'start':
                self.mode = msg.get('mode', 'auto')
                self.listening = True
                self.reset_vad()
                if not self.reply_task:
                    set_status(state=self.listen_state())
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
            self.wake_up()
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
            if time.monotonic() < self.head_quiet_until:
                return
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
        heard = False
        try:
            t0 = time.time()
            asleep = self.asleep()
            if not asleep:
                set_status(state='thinking')
            text = await asyncio.to_thread(stt, samples)
            stt_s = time.time() - t0
            log(f'聞き取り（{stt_s:.2f} 秒、声 {len(samples) / IN_RATE:.1f} 秒）: {text}')
            if not text or NOISE_TEXT.fullmatch(text):
                return
            if self.drive_stop and DRIVE_STOP.search(text):
                log('走行: 「止まって」')
                self.drive_stop.set()
                return
            if asleep:
                if not is_call(self.wake, text):
                    log('呼びかけがないので返事しません')
                    return
                log('呼びかけ: スタックちゃん')
                await self.wake_face()
                set_status(state='thinking')
            heard = True
            if asleep and len(re.sub(r'[\s\W]', '', self.wake.sub('', text))) <= WAKE_FILLER:
                await self.send(type='stt', text=text)
                await self.say(WAKE_REPLY)
                await self.need_cloud()
                return
            if is_patrol(self.wake.sub('', text) if self.wake else text):
                await self.send(type='stt', text=text)
                await self.start_patrol()
                return
            place = place_command(self.wake.sub('', text) if self.wake else text)
            if place and await self.start_place(place):
                await self.send(type='stt', text=text)
                return
            find = find_command(self.wake.sub('', text) if self.wake else text)
            if find:
                await self.send(type='stt', text=text)
                await self.start_find(find)
                return
            drive = drive_command(self.wake.sub('', text) if self.wake else text)
            if drive:
                await self.send(type='stt', text=text)
                await self.start_drive(drive)
                return
            if await self.need_cloud():
                await self.cloud_speech(samples)
                return
            await self.send(type='stt', text=text)
            if is_switch(TO_NET, text):
                await self.to_net()
                return
            if is_switch(TO_LOCAL, text):
                if STATUS['mode'] == 'net':
                    save_mode('local')
                    await self.set_face('local')
                    log('ローカルにもどりました')
                    await self.say('ローカルにもどりました。')
                else:
                    await self.say('今はローカルです。')
                return
            if is_switch(TO_MINI, text) or is_switch(TO_PC, text):
                await self.say(switch_backend('pc' if is_switch(TO_PC, text) else 'jetson'))
                return
            if is_switch(SHOOT, text):
                await self.countdown_shot()
                return
            await self.send(type='llm', emotion='thinking', text='🤔')
            now = time.strftime('今は %Y年%m月%d日 %H時%M分です。')
            system = system_prompt() + now
            query = (self.wake.sub('', text) if self.wake else text).strip(' 、。,.') or text
            if STATUS['mode'] == 'net' and not SELF_ASK.search(query):
                t1 = time.time()
                info = await asyncio.to_thread(web_info, query)
                log(f'ネットで調べました（{time.time() - t1:.2f} 秒）: {info.splitlines()[0] if info else "なし"}')
                system += NET_PROMPT + info
            elif STATUS['mode'] != 'net':
                system += LOCAL_PROMPT
            if CAR_ASK.search(text):
                car = await asyncio.to_thread(car_status, a.car_url)
                log(f'AI-CAR のようすを渡します: {car.splitlines()[0]}')
                system += CAR_PROMPT + car
            messages = [{'role': 'system', 'content': system}] + self.history + \
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
            if heard:
                self.wake_up()
            self.reply_task = None
            self.reset_vad()
            set_status(state=self.listen_state())

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
    set_status(dozing=False)
    s = Session(ws, args)
    SESSIONS.append(s)
    talk = asyncio.create_task(s.car_talk())
    follow = asyncio.create_task(s.patrol_follow())
    try:
        async for msg in ws:
            if isinstance(msg, bytes):
                if s.cloud and not s.asleep():
                    await s.cloud_audio(msg)
                else:
                    s.on_audio(msg)
            else:
                try:
                    await s.on_text(json.loads(msg))
                except ValueError:
                    log(f'JSON でないメッセージ: {msg[:80]}')
    except ConnectionClosed:
        pass
    finally:
        s.cancel.set()
        talk.cancel()
        follow.cancel()
        if s.drive_stop and getattr(s.drive_stop, 'follow', False):
            s.drive_task.cancel()
        elif s.drive_stop:
            s.drive_stop.set()
        if s.keep_task:
            s.keep_task.cancel()
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
        log(f'切れました: {peer}（コード {ws.protocol.close_code} {ws.protocol.close_reason or ""}）')


async def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--ws-port', type=int, default=8000)
    p.add_argument('--ota-port', type=int, default=8003)
    p.add_argument('--model', default='qwen2.5:3b')
    p.add_argument('--pc-host', default=os.environ.get('AI_PC_HOST', '100.86.172.21'),
                   help='聞き取りと返事に使うパソコン（whisper-server :8178・Ollama :11434）。空でパソコンを使わない')
    p.add_argument('--pc-model', default='qwen2.5:14b', help='パソコンの Ollama で返事に使う会話の AI')
    p.add_argument('--pc-speaker', type=int, default=3,
                   help='パソコンの VOICEVOX の声（スタイルの番号。3 = ずんだもん ノーマル）')
    p.add_argument('--voice', default='jf_alpha')
    p.add_argument('--pitch', type=int, default=300, help='声の高さを上げる量（セント、100 で半音。0 でそのまま）')
    p.add_argument('--peak-db', type=float, default=-1.0,
                   help='返事の声のいちばん大きい所の大きさ（dBFS、0 が上限）。Kokoro のままだと約 -7 dB で小さい')
    p.add_argument('--min-level', type=float, default=300.0, help='声とみなす音の大きさの下限（16 bit の RMS）')
    p.add_argument('--ratio', type=float, default=3.0, help='まわりの音の何倍で声とみなすか')
    p.add_argument('--end-silence', type=float, default=0.8, help='この秒数静かなら話し終わり')
    p.add_argument('--min-seconds', type=float, default=0.4)
    p.add_argument('--max-seconds', type=float, default=15.0)
    p.add_argument('--car-url', default=os.environ.get('AI_CAR_URL', 'http://100.70.35.31:8080'),
                   help='AI-CAR のダッシュボード（ラズパイ）。ここの /api/status の値を答えに使う')
    p.add_argument('--finder-url', default=os.environ.get('AI_FINDER_URL', 'http://127.0.0.1:8095'),
                   help='声で覚えた物を探す jetson/object_finder')
    p.add_argument('--no-car-talk', dest='car_talk', action='store_false',
                   help='自動運転＆ミニコン＆ローカルのとき、AI-CAR の動きをしゃべる実況をしない')
    p.add_argument('--turns', type=int, default=3, help='覚えておく会話の往復の数')
    p.add_argument('--wake-words', default=WAKE_WORDS,
                   help='呼びかけ待ちのとき、これ（正規表現）が聞き取りに入っているときだけ返事をする。空で呼びかけ待ちにしない')
    p.add_argument('--awake-seconds', type=float, default=30.0,
                   help='呼ばれてから・最後に話してから、呼びかけなしで話せる秒数。たつと「じゃあ、またね。」')
    p.add_argument('--doze-seconds', type=float, default=30.0,
                   help='「じゃあ、またね。」からこの秒数たつと、居眠りの顔にして首をホーム（正面・水平）へ。0 で居眠りしない')
    p.add_argument('--shot-pitch', type=int, default=30,
                   help='「撮影して」で数えるときの首の上向きの角度（0〜90 度、0 が水平）')
    p.add_argument('--watch-interval', type=float, default=2.0,
                   help='見守りで顔を見る間隔（秒）。0 で見守りをしない')
    p.add_argument('--primary-url', default=os.environ.get('AI_PRIMARY_URL', ''),
                   help='パソコンの控えとして動かすとき、ミニコンの bridge（例 http://100.111.231.54:8003）。'
                        '選択をまね、ミニコンがもどったらスタックちゃんを返す。いつもパソコンで話す')
    args = p.parse_args()

    global LOOP
    LOOP = asyncio.get_running_loop()
    OtaHandler.ws_port = args.ws_port
    WATCH['interval'] = max(0.0, args.watch_interval)
    PRIMARY['url'] = args.primary_url.rstrip('/')
    PC.update(host=args.pc_host, model=args.pc_model, args=args, speaker=load_pc_voice(args.pc_speaker),
              want='pc' if PRIMARY['url'] else load_backend() if args.pc_host else 'jetson')
    load_device_ota()
    set_status(mode=load_mode(), watch_on=WATCH['interval'] > 0)
    show_engines()
    ota = ThreadingHTTPServer((args.host, args.ota_port), OtaHandler)
    threading.Thread(target=ota.serve_forever, daemon=True).start()
    # 控えのときは Kokoro がないので whisper の見回りはしない（パソコンの whisper はパソコンの systemd が見る）
    watchdog = asyncio.create_task(primary_watch() if PRIMARY['url'] else stt_watchdog(args.voice, args.wake_words))
    pc = asyncio.create_task(pc_watch(args)) if args.pc_host else None
    async with serve(lambda ws: handler(ws, args), args.host, args.ws_port, max_size=2 ** 20):
        log(f'待ち受け: OTA http://{args.host}:{args.ota_port}/xiaozhi/ota/  WebSocket :{args.ws_port}/xiaozhi/v1/')
        await asyncio.Future()
    watchdog.cancel()
    if pc:
        pc.cancel()


if __name__ == '__main__':
    asyncio.run(main())
