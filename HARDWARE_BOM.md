# HARDWARE_BOM.md — AI-CAR 部品構成表

## 概要
AI-CAR（自律走行ロボットカー）のハードウェア構成を記録する。

---

## 部品リスト

| No | 部品名 | 仕様・備考 | 接続方法 |
|----|--------|-----------|---------|
| 1 | **Raspberry Pi 5** | 8GB RAM（メインボード） | — |
| 2 | **Raspberry Pi AI HAT+** | 26TOPS NPU（推論アクセラレーション） | Pi 5 GPIO ヘッダー |
| 3 | **Adafruit DC & Stepper Motor HAT** | モーター駆動制御（4ch） | Pi 5 GPIO ヘッダー（I²C） |
| 4 | **Raspberry Pi カメラモジュール v3** | 960×540・30fps | CSI バス |
| 5 | **LiDAR（RPLIDAR A1M8）** | 2D レーザースキャナ（最大距離8m） | USB シリアル |
| 6 | **9軸 IMU（GY-BNO055）** | 加速度・ジャイロ・地磁気融合 | I²C |
| 7 | **状態表示液晶 ZJY-IPS130-V2.0** | 1.3 インチ 240×240 IPS、ST7789、7 ピン（CS なし）。旧 TM1637 7 セグ LED を置き換え | SPI0（pin 19/23）＋ GPIO24/25/23（RES/DC/BLK） |
| 8 | **Logicool G ゲームパッド** | 手動操作ジョイスティック | USB Bluetooth |
| 9 | **メカナムホイール駆動系** | オムニディレクショナル移動（4輪）。OSOYOO 520 エンコーダー付きモーター ×4 | モーター HAT 経由（エンコーダーは Pi GPIO 直結） |
| 9a | **エンコーダー用ジャンパー線** ×2 | 25cm、2.54mm ピッチ、メス-メス 6ピン to 6ピン。OSOYOO 520 モーターエンコーダー↔Pi 接続専用。型番 2024006000 | モーター 6ピン → HAT M端子（M+/M−）と Pi ヘッダー（VCC/GND/A/B） |
| 10 | **バッテリー** | 5V / 12V 2系統出力 | — |
| 11 | **配線ケーブル一式** | 接続用 | — |
| 12 | **シャーシ（ロボット車体）** | 機械構造 | — |

---

## システム構成図（論理）

```
┌─────────────────────────────────────────────┐
│              Raspberry Pi 5 (8GB)            │
│                                             │
│  ┌──────────┐   ┌──────────┐               │
│  │ ROS2     │   │ FastAPI  │               │
│  │ Jazzy    │   │ Server   │               │
│  │ Nav2     │   │          │               │
│  │ SLAM     │   └──────────┘               │
│  │ Toolbox  │                               │
│  └──────────┘                               │
│                                             │
│  AI HAT+ (NPU 26TOPS) ← 推論オフロード      │
└─────┬──────────┬──────────┬─────────────────┘
      │          │          │
   CSIバス     I²C        USB シリアル
      │          │          │
  カメラv3    IMU       RPLIDAR A1M8
            BNO055
```

---

## 電源構成

| 系統 | 供給元 | 供給先 |
|------|--------|--------|
| 5V | バッテリー（5V系統） | Raspberry Pi 5、センサー類 |
| 12V | バッテリー（12V系統） | Adafruit Motor HAT → メカナムホイール駆動系 |

> **注意**: 電源はバッテリーから 5V と 12V の2系統で供給される。12V はモータードライバー専用であり、Raspberry Pi 側には接続しない。

---

## Motor HAT モーター端子の割り付け（`src/ai_car_web/config/motor_hat.yaml`）

車輪の呼び方は進行方向基準で「**M1 前左**」のように端子名＋位置で統一する。モーター線は **赤 Motor+ / 黒 Motor−**（6 本の内訳は次節）。

| 端子 | 車輪 | Motor+ / Motor− | 回転 | TB6612 | PCA9685 ch (PWM / IN1 / IN2) |
|------|------|-----------------|------|--------|-------------------------------|
| M1 | 前左（front_left） | 赤 / 黒 | 正転 | #1 A | 8 / 10 / 9 |
| M2 | 前右（front_right） | 赤 / 黒 | 反転 | #1 B | 13 / 11 / 12 |
| M3 | 後左（rear_left） | 赤 / 黒 | 正転 | #2 A | 2 / 4 / 3 |
| M4 | 後右（rear_right） | 赤 / 黒 | 反転 | #2 B | 7 / 5 / 6 |

> 暫定値。実配線と照合し、モーター制御ノードで回転方向を確認して `reversed` を修正する。

### モーター 1 台 6 本の接続先（`motor_hat.yaml` の `encoder` / `gpio_pins.yaml`）

各モーターの 6 ピン配線。モーター側 PH2.0 6 ピンの並びは公式データシートで **1 M1 / 2 GND / 3 C1 / 4 C2 / 5 VCC / 6 M2**。配線色はケーブルのロットで異なる（公式表は 白 GND / 黄 C1 / 緑 C2 / 青 VCC）ため、下表は本機の実物ケーブル（緑・橙・黄・白・赤・黒、青の代わりに橙）での割り付け。VCC / GND を逆に挿すとエンコーダーを壊す恐れがあるため、色が違うケーブルではコネクタのピン番号で確認する:

| 配線色 | ピン名称 | 役割 | 接続先 |
|--------|----------|------|--------|
| 赤 | Motor+ | モーター駆動電源（正極 DC 12V） | Motor HAT M端子 + |
| 黒 | Motor− | モーター駆動電源（負極） | Motor HAT M端子 − |
| 橙 | VCC | エンコーダー用電源（DC 3.3〜5V） | Motor HAT 上の 3V3 ピン（Pi pin 17 の引き出し） |
| 白 | GND | エンコーダー用グランド | Motor HAT 上の GND ピン（Pi GND の引き出し） |
| 黄 | Encoder A (C1) | A 相パルス出力 | Pi GPIO |
| 緑 | Encoder B (C2) | B 相パルス出力 | Pi GPIO |

Adafruit Motor HAT にエンコーダー入力はないため、Motor+/Motor− は HAT の M 端子へ、VCC / GND は Motor HAT 上に多数ある 3V3 / GND ピン（Pi 40 ピンヘッダーの引き出しで番号は共通）へ、Encoder A / B は Pi GPIO へエンコーダー用ジャンパー線で接続する（OSOYOO の資料では PWM HAT 経由だが本機は Pi 直結）。

| モーター | Motor+ 赤 | Motor− 黒 | VCC 橙 | GND 白 | Encoder A 黄 | Encoder B 緑 |
|----------|-----------|-----------|--------|--------|--------------|--------------|
| M1 前左 | HAT M1 + | HAT M1 − | HAT pin 17 (3V3) | HAT pin 30 | Pi pin 29 (GPIO5) | Pi pin 31 (GPIO6) |
| M2 前右 | HAT M2 + | HAT M2 − | HAT pin 17 (3V3) | HAT pin 34 | Pi pin 33 (GPIO13) | Pi pin 35 (GPIO19) |
| M3 後左 | HAT M3 + | HAT M3 − | HAT pin 17 (3V3) | HAT pin 39 | Pi pin 37 (GPIO26) | Pi pin 32 (GPIO12) |
| M4 後右 | HAT M4 + | HAT M4 − | HAT pin 17 (3V3) | HAT pin 25 | Pi pin 36 (GPIO16) | Pi pin 38 (GPIO20) |

- VCC / GND は Motor HAT 上の 3V3 / GND ピンを使う（`motor_hat.yaml` の `power_board: Motor HAT`）。VCC は **3V3**（pin 17、4 台共通）。仕様上は 5V も可だが Encoder A/B 出力が GPIO 直結のため 5V は使わない
- 使用済みの液晶ピン（pin 16/18/19/22/23、ZJY-IPS130）と I2C（pin 3/5）は避けている
- GPIO ヘッダー表（ダッシュボード）では VCC / GND のピンは Motor HAT 側に挿すため空き表示、Encoder A / B のみ使用中

> Pi 側のピン割り付けは暫定値。実配線と照合し、A/B の逆相（カウント方向）はエンコーダー読み取りノードで確認して修正する。

> Pi 側のピン割り付けは暫定値。実配線と照合し、A/B の逆相（カウント方向）はエンコーダー読み取りノードで確認して修正する。

---

## メモ
- カメラ解像度は 960×540 / 30fps・JPEG 品質80 に設定（CPU 負荷とダッシュボード表示の滑らかさのバランス）
- AI HAT+ は NPU 推論用（ROS2 ノードと連携）
- GY-BNO055 は ROS2 の `imu` トピックを公開
