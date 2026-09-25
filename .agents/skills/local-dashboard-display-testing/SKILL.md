---
name: local-dashboard-display-testing
description: ROS 2や実機を使用せず、AI-CARダッシュボードの設定由来の表示を安全に確認する方法
---

# ローカル表示確認

## 安全境界
- 表示だけの依頼ではRaspberry PiにSSH接続しない。実機用blueprintの接続手順を実行しない。
- 代替APIは依頼者が許可した場合のみ使い、最終報告に実FastAPI/ROS統合を確認していないことを記載する。
- 表示用サーバーは127.0.0.1にbindし、制御用POST APIを提供しない。

## 起動
- パッケージルートは`src/ai_car_web`。設定はその中の`config`にある。
- ROS 2がなくても、Python標準ライブラリのHTTPサーバーと既存PyYAMLで表示確認できる。
- `/`で`static/index.html`をそのまま配信する。
- パッケージルートをPythonパスに追加し、次の実関数をGETエンドポイントに割り当てる。
  - `/api/motor_hat`: `ai_car_web.motor_hat.load_motor_hat(config/motor_hat.yaml)`
  - `/api/gpio`: `ai_car_web.gpio_pinout.build_pinout(config/gpio_pins.yaml)`
  - `/api/architecture`: `ai_car_web.architecture.load_architecture(config/architecture.yaml, {})`
  - `/api/status`: `{}`。ライブテレメトリーではないことを明示する。
- WSが利用できなければUIはstatusのポーリングに切り替わる。カメラなど未提供APIの表示は対象外とする。
- 日本語が豆腐になる場合は`fonts-noto-cjk`を導入し、Chromeを再起動する。単純な再読み込みでは反映されない場合がある。

## UI導線と証拠
- 「プロジェクト説明」→「ハードウェア構成」→「配線詳細」。
- GPIOカードの下にMotor HATカードがある。モーダル内をスクロールする。
- SVG、表、割付バッジと末尾の警告注記をそれぞれ確認する。
- カードの文字が小さいため、全画面スクリーンショットに加え、computer.zoomの実解像度画像が有用。
- 録画前に`wmctrl -r :ACTIVE: -b add,maximized_vert,maximized_horz`で最大化する。

## Devin Secrets Needed
- ローカル表示確認には不要。
