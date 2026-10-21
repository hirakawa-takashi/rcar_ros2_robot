# HARDWARE_BOM.md — AI-CAR 部品構成表

## 概要
AI-CAR（自律走行ロボットカー）のハードウェア構成を記録する。

---

## 部品リスト

| No | 部品名 | 仕様・備考 | 接続方法 |
|----|--------|-----------|---------|
| 1 | **Raspberry Pi 5** | 8GB RAM（メインボード） | — |
| 2 | **Raspberry Pi AI HAT+** | Hailo-8・26TOPS NPU。今は使っていない（YOLO は `hef_path` を空にして止めている） | Pi 5 GPIO ヘッダー |
| 3 | **Adafruit DC & Stepper Motor HAT**（Adafruit 2348） | モーター駆動制御（4ch、PCA9685 0x60 + TB6612 ×2）。2026/10/12 に同型品へ交換 | Pi 5 GPIO ヘッダー（I²C） |
| 4 | **Raspberry Pi Camera Module 3 Wide**（SC0874、IMX708） | 横の画角 102°。960×540・30fps・JPEG 品質 80（`camera_ros`） | CSI バス |
| 5 | **LiDAR（RPLIDAR A1M8）** | 2D レーザースキャナ（最大距離8m） | USB シリアル |
| 6 | **9軸 IMU（GY-BNO055）** | 加速度・ジャイロ・地磁気融合 | I²C |
| 8 | **Logicool F710 ゲームパッド** | 手動操作（XInput モード、`/dev/input/js0`） | USB 無線レシーバー（2.4GHz） |
| 9 | **メカナムホイール駆動系** | オムニディレクショナル移動（4輪）。OSOYOO 520 エンコーダー付きモーター ×4（2024005900、Ø37 × 72 mm、軸 Ø4 D カット、56:1）、メカナムホイール Ø80.59 × 幅 39.1 mm（六角穴 6.5 mm）、37 mm モーター台 ×4（天板の下、穴 24 × 30 mm） | モーター HAT 経由（エンコーダーは Pi GPIO 直結） |
| 9a | **エンコーダー用ジャンパー線** ×2 | 25cm、2.54mm ピッチ、メス-メス 6ピン to 6ピン。OSOYOO 520 モーターエンコーダー↔Pi 接続専用。型番 2024006000 | モーター 6ピン → HAT M端子（M+/M−）と Pi ヘッダー（VCC/GND/A/B） |
| 10 | **バッテリー（CIO SMARTCOBY Pro SLIM 35W）** ×2 | 10000mAh、USB-C PD（12V 出力）。3D プリントの箱に 2 個重ねて収納 | USB-C PD 12V → DROK 降圧コンバーター / Motor HAT |
| 10a | **DROK 降圧コンバーター**（B09JKHD3SH） | DC 9〜36V → 5.2V / 5A、USB-A 出力。基板 63×27 mm | 12V 入力（DC ジャック / 端子）→ USB-A → Pi 5 USB-C |
| 10b | **落下防止センサー（Dovhmoh VL53L1X）** ×2 | ToF 測距、3〜5V。前後バンパーに真下から進行方向へ 45° 傾けて取付。I²C 初期アドレス 0x29（BNO055 と重なるため、`cliff_node` が XSHUT を 1 台ずつ上げて前 0x30 / 後 0x31 に変更）。**配線済み**（後ろ・前とも 2026/10/17。付け直したあと（同じ日）、床の上で前 0.087 m・後ろ 0.082 m（20 秒の中央）を読める。平らな床の目安は 0.095 m）。配線色は VIN 白・GND 黄（エンコーダーの電源と同じ）・SCL 灰・SDA 青・XSHUT 緑、GPIO1 はつながない。VIN・GND は前後とも Motor HAT (3V3 power / Ground)、XSHUT は前 pin 11 / 後 pin 13、SDA / SCL は pin 3 / 5 を共有 | I²C バス 1 ＋ XSHUT GPIO17 / GPIO27 |
| 10c | **USB スイッチ（LOYELEY、USB-C、ON/OFF ランプ付き、28 cm）** ×2 | 箱 47.2 × 17.4 × 13.5 mm、ボタン Ø13・高さ 3 mm。下が DROK（ラズパイ）、上が Motor HAT（モーター）の電源の線につける。Pi の右に横 2 段（`hardware/3d` の `sw_holder.stl`）、カバーの右の壁の窓から押す | バッテリー USB-C → スイッチ → DROK / Motor HAT |
| 11 | **配線ケーブル一式** | 接続用 | — |
| 12 | **シャーシ（ロボット車体）** | 機械構造。カバー・バッテリー箱・センサーの台などは 3D プリント（`hardware/3d/`） | — |

### AI-CAR の外の機械

| 機械 | 仕様・役割 | つながり |
|------|-----------|---------|
| **Jetson Orin Nano Super**（メモリ 8GB、JetPack 6.2、Yahboom ケース・OLED） | スタックちゃんの会話（whisper・Ollama qwen2.5:3b・Kokoro）・顔の見分け・状態の送信（`jetson/`） | Wi-Fi / Tailscale。ラズパイのダッシュボードへ状態・速度の指令 |
| **M5 スタックちゃん**（M5Stack CoreS3） | 話しかける顔・カメラ・首のサーボ | Wi-Fi で Jetson の `stackchan_bridge` |
| **パソコン**（Windows 11 + WSL、RTX 4080 SUPER） | 動いているときだけ、聞き取り（whisper large-v3）・会話（qwen2.5:14b）・声（VOICEVOX） | Tailscale で Jetson から使う |

---

## システム構成図（論理）

```
┌─────────────────────────────────────────────┐
│              Raspberry Pi 5 (8GB)            │
│                                             │
│  ┌──────────┐   ┌──────────┐               │
│  │ ROS2     │   │ FastAPI  │               │
│  │ Jazzy    │   │ Server   │               │
│  │ SLAM     │   │ (8080)   │               │
│  │ Toolbox  │   └──────────┘               │
│  └──────────┘                               │
│                                             │
│  AI HAT+ (Hailo-8 26TOPS)                   │
└──┬───────┬──────────┬──────────┬───────┬────┘
   │       │          │          │       │
 CSIバス   I²C        USB        GPIO    I²C
   │       │          │          │       │
 カメラ   IMU      RPLIDAR    エンコーダー Motor HAT
 3 Wide  BNO055    A1M8       ×4      (0x60) → M1〜M4
        VL53L1X ×2（0x30 / 0x31）
```

Nav2（地図で行き先へ行く、Level 2）は ros-jazzy-navigation2 を入れて、ラズパイで動かしている（`nav2.launch.py`）。

---

## 電源構成

| 系統 | 供給元 | 供給先 |
|------|--------|--------|
| 5V（ラズパイ） | バッテリー USB-C PD 12V →（USB スイッチ 下）→ DROK 5.2V / 5A → USB-A → Pi 5 USB-C | Raspberry Pi 5（AI HAT+・カメラ・LiDAR・IMU・VL53L1X・エンコーダーの 3V3 はここから） |
| 12V（モーター） | バッテリー USB-C PD 12V →（USB スイッチ 上）→ Motor HAT の電源端子 | Motor HAT → M1〜M4 |

> **注意**: 12V はモータードライバー専用で、ラズパイには直接つながない。ラズパイの USB-C には USB-A からつなぐので PD のやりとりはなく、ラズパイは「3 A の電源」と見ている（`usb_max_current_enable=1` で USB 機器に 1.6 A まで流す）。2026/10/17 の時点で電圧低下（Undervoltage）は 0 回。

---

## Motor HAT モーター端子の割り付け（`src/ai_car_web/config/motor_hat.yaml`）

車輪の呼び方は進行方向基準で「**M1 前左**」のように端子名＋位置で統一する。モーター線は **黒 Motor+ / 赤 Motor−**（6 本の内訳は次節）。

| 端子 | 車輪 | Motor+ / Motor− | 回転 | TB6612 | PCA9685 ch (PWM / IN1 / IN2) |
|------|------|-----------------|------|--------|-------------------------------|
| M1 | 前左（front_left） | 黒 / 赤 | 正転 | #1 A | 8 / 10 / 9 |
| M2 | 前右（front_right） | 黒 / 赤 | 反転 | #1 B | 13 / 11 / 12 |
| M3 | 後左（rear_left） | 黒 / 赤 | 正転 | #2 A | 2 / 4 / 3 |
| M4 | 後右（rear_right） | 黒 / 赤 | 反転 | #2 B | 7 / 5 / 6 |

> 実配線（2026/10/12、Motor HAT 交換のときに 1 台ずつ回して確認）。右側の 2 輪（M2・M4）は `reversed: true`。

### モーター 1 台 6 本の接続先（`motor_hat.yaml` の `encoder` / `gpio_pins.yaml`）

各モーターの 6 ピン配線。配線色はエンコーダー基板のコネクタ横の印字 **S2 / S1 / GND / VCC / M− / M+**（線は順に 緑 / 橙 / 黄 / 白 / 赤 / 黒）で確認した（Motor+/− はテスターで黒-赤間 4Ω・手回しで発電も確認）。公式の色表（白 GND / 黄 C1 / 緑 C2 / 青 VCC）は本機のケーブルには適用せず、VCC / GND を逆に挿すとエンコーダーを壊す恐れがあるため、基板印字とコネクタのピン番号で確認する:

| 配線色 | ピン名称 | 役割 | 接続先 |
|--------|----------|------|--------|
| 黒 | Motor+ (M+) | モーター駆動電源（正極 DC 12V） | Motor HAT M端子 + |
| 赤 | Motor− (M−) | モーター駆動電源（負極） | Motor HAT M端子 − |
| 白 | VCC | エンコーダー用電源（DC 3.3〜5V） | Motor HAT (3V3 power) |
| 黄 | GND | エンコーダー用グランド | Motor HAT (Ground) |
| 橙 | Encoder A (S1) | A 相パルス出力 | Pi GPIO |
| 緑 | Encoder B (S2) | B 相パルス出力 | Pi GPIO |

> **注意**: 赤（Motor−）と黒（Motor+）はモーター端子の 2 本で、一般的な「赤 = 電源 + / 黒 = GND」ではない。どちらも Motor HAT の M 端子以外（Pi や電池の GND など）にはつながない。入れ替えても回転方向が逆になるだけで故障しない。

Adafruit Motor HAT にエンコーダー入力はないため、Motor+/Motor− は HAT の M 端子へ、VCC / GND は Motor HAT (3V3 power) / Motor HAT (Ground) へ、Encoder A / B は Pi GPIO へエンコーダー用ジャンパー線で接続する（OSOYOO の資料では PWM HAT 経由だが本機は Pi 直結）。

| モーター | Motor+ 黒 | Motor− 赤 | VCC 白 | GND 黄 | Encoder A 橙 | Encoder B 緑 |
|----------|-----------|-----------|--------|--------|--------------|--------------|
| M1 前左 | HAT M1 + | HAT M1 − | Motor HAT (3V3 power) | Motor HAT (Ground) | Pi pin 29 (GPIO5) | Pi pin 31 (GPIO6) |
| M2 前右 | HAT M2 + | HAT M2 − | Motor HAT (3V3 power) | Motor HAT (Ground) | Pi pin 33 (GPIO13) | Pi pin 35 (GPIO19) |
| M3 後左 | HAT M3 + | HAT M3 − | Motor HAT (3V3 power) | Motor HAT (Ground) | Pi pin 37 (GPIO26) | Pi pin 32 (GPIO12) |
| M4 後右 | HAT M4 + | HAT M4 − | Motor HAT (3V3 power) | Motor HAT (Ground) | Pi pin 36 (GPIO16) | Pi pin 38 (GPIO20) |

- VCC / GND は Motor HAT (3V3 power) / Motor HAT (Ground) を使う（`motor_hat.yaml` の `power_board: Motor HAT`）。VCC は **3V3**（4 台共通）。仕様上は 5V も可だが Encoder A/B 出力が GPIO 直結のため 5V は使わない
- I2C（pin 3/5）は避けている
- HAT の M 端子のネジの並びは、M1・M3 が左から 黒 / 赤、M2・M4 が左から 赤 / 黒（実配線。真ん中の GND には挿さない。`motor_hat.yaml` の `minus_left`）。こうすると、正転指令で左（M1・M3）は前進、右（M2・M4）は後退し（`reversed: true`）、エンコーダーのカウントは 4 台とも負になる（2026/10/12 確認）
- GPIO ヘッダー表（ダッシュボード）では VCC / GND のピンは Motor HAT 側に挿すため空き表示、Encoder A / B のみ使用中

> Pi 側のピン割り付けは実配線（2026/10/12 に M1〜M4 の A/B 相パルスを確認）。`encoder_odom_node` は 4 輪の A/B を GPIO v2 uAPI で両エッジとも数え（4 逓倍）、床で 1.8 m 走って合わせた **1 回転 1,408 カウント**（`counts_per_rev`）を使う。前進でカウントが負になる向きは `encoder_odom_node` で符号を合わせる。後ろ右は手で速く回しても数えそこね約 0.3 %（2026/10/17）。

---

## メモ
- カメラ解像度は 960×540 / 30fps・JPEG 品質80 に設定（CPU 負荷とダッシュボード表示の滑らかさのバランス）
- AI HAT+ は NPU 推論用（`perception_node`。今は止めている）
- GY-BNO055 は `/imu/data`（20 Hz）と `/imu/mag`（磁気は表示だけ、向きの制御には使わない）を公開
